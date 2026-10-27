# Manual del pipeline IMU + OpenCap — de los sensores a OpenSim

Todo lo que hace falta para **repetir de punta a punta** lo que se hizo en la
sesión del 2026-09-11 (`resultados/sesion_2026-09-11/`): grabar 6 IMU con una Raspberry Pi al
mismo tiempo que OpenCap, abrir la toma en la app de marcha, generar los tres
`.mot` para OpenSim y compararlos. Pensado para leerse sin haber seguido el
proyecto.

Los tres productos, y qué programa los hace:

| Producto | Qué es | Programa |
|---|---|---|
| **La app** | ventana con gráficas y tabla de los parámetros de marcha **validados** que miden los IMU | `pc/app_marcha.py` (y `parametros_marcha.py` desde la terminal) |
| **Los tres `.mot`** | (1) OpenCap puro · (2) solo IMU, reconstruido · (3) OpenCap + IMU incrustado (con y sin pelvis) | OpenCap · `pc/reconstruir_toma.py` · `pc/fusionar_mot.py` |
| **La comparación** | los tres `.mot` cortados en los mismos ciclos de marcha: curvas, ROM, r y RMSE | `pc/comparar_mot.py` (+ `sincronizar.py`) |

`pc/pipeline.py` corre todo lo anterior con un solo comando (§5).

---

## 1. Qué hace falta

### Hardware
- **6 sensores MbientLab MetaMotion** (chip BMI160; el script detecta el chip
  y avisa si se mezclan modelos). Cargados: un sensor con poca batería conecta
  peor y se cae antes.
- **Raspberry Pi 4B** con Raspbian **32-bit** (en 64-bit el SDK de MbientLab
  falla por los bindings C++), el SDK instalado en `~/MetaWear-SDK-Python/`
  (`python3 -c "import mbientlab"` tiene que dar ok).
- **Tres adaptadores Bluetooth**: el integrado de la Pi + **2 dongles USB**,
  en puertos **USB 2.0** (los negros: el USB 3.0 irradia justo en 2.4 GHz),
  con alargue y antena vertical. El reparto es **2+2+2** (dos sensores por
  adaptador; tres por adaptador es donde se cae la tercera conexión).
- **OpenCap**: 2 iPhones + tablero de calibración, según las instrucciones de
  [opencap.ai](https://www.opencap.ai/). El export se baja de la web.
- Un pasillo. **Con 3.5 m salen 3 zancadas por pasada** y quedan 3-5 zancadas
  típicas por lado para comparar; con 8-10 m serían 5-6 en régimen. Es la
  limitación que más pesó en los datos que hay.

### Software en la PC
- Python 3.10 o más nuevo (probado con 3.12) con `tkinter` (viene con el
  instalador oficial).
- `pip install -r pc/requirements.txt` → numpy, scipy, pandas, matplotlib, vqf.
- **OpenSim** (la aplicación de escritorio, 4.x) sólo para *ver* los `.mot`.
  **No** hace falta el paquete Python `opensim`.
- La biblioteca **Mythos MECH** (trabajo previo del IIT): la parte que se usa
  ya viene copiada en `pc/mythos_mech/` (ver `ORIGEN.md` ahí).

### Montaje de los sensores (montaje S, "sparse-6")

| Sensor | Dónde | Qué mide en el pipeline |
|---|---|---|
| cabeza | banda elástica, centrado | inclinación de cabeza (análisis detallado) |
| espalda | **sacro (S2), pegado a la piel** con cinta doble faz, no sobre el pantalón | `pelvis_tilt` / `pelvis_list`; referencia de la cadena de la pierna |
| mano_der / mano_izq | dorso de la mano o cúbito distal junto a la muñeca (rígido con el antebrazo) | cadena de brazo `arm_flex + elbow_flex` |
| pie_der / pie_izq | empeine, bajo los cordones (nunca en el talón) | cadena de pierna `hip − knee + ankle`, eventos de marcha, ZUPT |

Montaje rígido, misma orientación siempre. **No hay sensor en muslo ni
pierna**: cadera, rodilla y tobillo **no se miden por separado** (§8).
Anotar en la planilla qué MAC va a qué segmento: **eso es lo que después se
escribe en `pc/mapa_sensores.py`** (§6).

---

## 2. Dónde está cada cosa en el repositorio

```
repositorio/
├── raspberry/        acquire.py, log.py, fusionar.py + README (lo que va a la Pi)
├── pc/               pipeline.py, app_marcha.py y los módulos de análisis
│   └── mythos_mech/  copia parcial de Mythos MECH (core/, kinematics/)
├── datos/
│   └── OPENCAP/sesion_2026-09-11/
│       ├── Datos imu de esta sesion/   los dos CSV de log.py + run_log.csv
│       └── OpenCapData/                MarkerData/, OpenSimData/ (sin videos)
├── resultados/sesion_2026-09-11/       1_para_la_app/, 2_para_opensim/, 3_analisis/
└── docs/             este manual y los documentos del método
```

Los scripts de `pc/` encuentran los datos solos: suben carpetas hasta
encontrar `datos/OPENCAP/` (o `OPENCAP/`). Si se mueven, fijar la carpeta de
datos a mano:

```
set INDICATIC_RAIZ=C:/ruta/a/datos        (Windows)
export INDICATIC_RAIZ=/ruta/a/datos       (Linux / macOS)
```

---

## 3. Grabar una sesión (Pi + OpenCap)

### 3.1 Preparar la Pi (una vez)

```bash
scp raspberry/acquire.py raspberry/log.py raspberry/fusionar.py pi@raspberrypi.local:~/indicatic/
```

Detalle de todos los modos y opciones: `raspberry/README.md`.

Los tres en la **misma carpeta** (`log.py` busca a `acquire.py` al lado). Si
los sensores son otros, la lista de MAC está en **dos** lugares y hay que
cambiar los dos: `acquire.py` (`DEFAULT_MACS`, ~línea 102) y `log.py`
(`MACS`). El orden de la lista es el orden de conexión: con 2+2+2, las dos
primeras van al Bluetooth integrado, las dos siguientes al primer dongle y
las dos últimas al segundo.

> Desde el 2026-09-01 el sensor `EE:9D:A0:09:C6:11` (que no grababa) está
> reemplazado por `DB:A1:10:94:0E:FD`; la lista de los scripts ya lo refleja.
> **Revisarla contra los sensores que se tengan en la mano.**

### 3.2 Los comandos, en orden

```bash
cd ~/indicatic
python3 acquire.py --mode check      # diagnóstico: adaptadores, WiFi, USB. No seguir si se queja.
python3 acquire.py --mode battery    # batería y modelo de cada sensor
python3 log.py caminata01 90         # graba 90 s en la memoria de los sensores y descarga al final
```

**Nunca con `sudo`** (deja la caché `.metawear/` de root y la corrida
siguiente se cuelga en `Descriptors found`). `log.py` da 10 s de cuenta
regresiva, prende los LED **verdes** cuando graba (rojo parpadeando = ese
sensor no arrancó), y **al terminar la cuenta descarga por Bluetooth: hay que
estar cerca de la Pi en ese momento**. Durante la toma el sujeto puede
alejarse lo que quiera: los datos van a la flash de cada sensor (por eso el
modo log da **0 % de pérdida**).

Parámetros con los que se grabó todo: 100 Hz, acelerómetro + giroscopio,
packed, intervalo de conexión 30 ms, 2+2+2. Quedan en el nombre del archivo:
`<tag>_log_split_6dev_raw_100hz_ci30_<fecha>_<hora>.csv`.

Si un sensor no se descargó, **sus datos siguen en la flash**:
`python3 log.py rescate C7` (un trozo inconfundible de la MAC) y después
`python3 fusionar.py <sesion>.csv <rescate>.csv`.

### 3.3 Qué hacer delante de las cámaras (por toma)

Esto es lo que produjo los mejores datos (TEST2 del 2026-09-11) y lo que
todavía faltó:

1. Arrancar `log.py` **antes** que OpenCap y que su cuenta termine **después**
   de parar OpenCap: la ventana del IMU tiene que **contener** a la de
   OpenCap. Una toma sin solape no se recupera. (duración de `log.py` =
   duración de OpenCap + ~20 s).
2. **Parado quieto 5-10 s** al principio (sesgo de giroscopio; y es lo que
   destraba el IMUPlacer de OpenSense, todavía sin usar).
3. **3 golpes secos con el talón derecho contra el piso**, parado, dentro del
   campo de las cámaras, **al principio y al final** de la toma. Es el evento
   de sincronización: con él el arranque IMU↔OpenCap sale con **26-27 ms de
   precisión**. TEST2 (golpes al inicio y al final) fue la mejor toma por eso.
4. *(Pendiente, no se hizo el 2026-09-11 y es lo que le falta al brazo)*
   **calibración funcional, 15 s**: 3 flexiones lentas de tronco (~40°) y 3
   balanceos de brazos adelante-atrás con el codo extendido.
5. Caminar **atravesando** el cuadro (lateral a las cámaras), ya en marcha al
   entrar en cámara: los primeros segundos con el sujeto entrando OpenCap los
   resuelve mal (TEST2: `pelvis_rotation` de −260°, |ω| del pie de 2900 °/s
   en los primeros 4 s) y se descartan.
6. Ida y vuelta las veces que dé la toma. La vuelta (giro con pasitos) se
   filtra sola en el análisis.

Planilla por toma: nombre del tag del IMU, nombre del trial en OpenCap, qué
sensor en qué segmento, y notas.

### 3.4 Sacar los datos

```bash
sync                                       # en la Pi, SIEMPRE antes de copiar
scp -r pi@raspberrypi.local:~/datos_v3 ./  # en la PC
```

(Si se usa memoria USB: copiar, `sync`, y recién ahí sacarla. Se perdió una
corrida entera por no hacerlo.)

Formato del CSV: una fila por muestra, `timestamp_host, mac, type (acc|gyro),
x, y, z, epoch_device`. `epoch_device` es el reloj **del sensor** (ms); cada
sensor corre a su propio Hz (98.4-101.3 medidos), así que alinearlos es
trabajo del análisis, no de la grabación (`sparse6.cargar_toma` lo hace).

---

## 4. El export de OpenCap y dónde ponerlo

Bajar la sesión de OpenCap entera. Trae `MarkerData/<trial>.trc`,
`OpenSimData/Kinematics/<trial>.mot`, `OpenSimData/Model/LaiUhlrich2022_scaled.osim`
(el modelo escalado al sujeto, con `Geometry/`), `sessionMetadata.yaml`,
`CalibrationImages/`, `Videos/`. El `.mot` viene a 60 Hz con 33 coordenadas,
en grados.

Convención que usa el pipeline:

```
datos/OPENCAP/
└── sesion_<fecha>/                             una carpeta por sesión
    ├── OpenCapData/                            la carpeta interna del zip de OpenCap
    │   ├── MarkerData/  OpenSimData/  ...
    └── Datos imu de esta sesion/               <- los CSV crudos de log.py, copiados acá
        ├── <tag>_log_split_6dev_raw_100hz_ci30_<fecha>_<hora>.csv
        └── run_log.csv
```

Los videos (`Videos/`) no hacen falta para el pipeline y no se suben al
repositorio (pesan cientos de MB y muestran al sujeto). Conviene también
anonimizar `subjectID` en `sessionMetadata.yaml`.

Toda carpeta `OPENCAP/*/Datos imu de esta sesion/` se encuentra sola; no hay
que registrarla. Lo que sí se registra es la sesión y el emparejamiento (§6).

---

## 5. Correr el pipeline sobre una toma

Con la toma ya registrada (las dos del 2026-09-11 ya lo están):

```bash
cd pc
pip install -r requirements.txt            # una vez
python pipeline.py testnorma2l9112026 --salida ../resultados/sesion_2026-09-11
python pipeline.py test1normal9112026 --salida ../resultados/sesion_2026-09-11
```

Tarda ~1 minuto por toma. Verificado: regenera bit a bit los `.mot`, tablas y
CSV de `resultados/sesion_2026-09-11/`.

`--salida` es la carpeta raíz de salida; adentro crea (o reusa)
`1_para_la_app/`, `2_para_opensim/` y `3_analisis/`. Con `--solo app`,
`--solo mot` o `--solo comparacion` corre una parte. Etapas, en orden — cada
una es un script que se puede correr solo:

| # | Script | Entrada | Salida | Para qué |
|---|---|---|---|---|
| 1 | `recortar_toma.py <csv> --tramo --salida A` | CSV crudo | `1_para_la_app/<toma>_..._tramo.csv` | quedarse con el tramo de marcha continua más largo (el resto es parado/acomodarse y estorba) |
| 2 | `parametros_marcha.py <csv> --salida N --figuras` | CSV recortado | `3_analisis/<toma>_figura.png`, `_parametros.json`, `_series.csv`, `_eventos.csv` | **lo de la app**, a archivo (`--detalle` agrega la figura de análisis completa) |
| 3 | `analizar_toma.py <csv> --fig O` | CSV recortado | `2_para_opensim/<toma>_orientations.sto` + QC en pantalla | control por sensor (sesgo, marco, % reposo); el `.sto` es para OpenSense (todavía sin pose estática: no usable) |
| 4 | `reconstruir_toma.py <csv> --salida O --sin-excluir` | CSV recortado + OpenCap de la sesión | `2_para_opensim/<toma>_reconstruido.mot` | **.mot solo IMU** (§7.2) |
| 5 | `fusionar_mot.py <toma> --salida O` | CSV **crudo** + `.trc`/`.mot` de OpenCap | `2_para_opensim/<toma>_opencap_mas_imu.mot` | **.mot OpenCap + IMU** (pierna y brazos) (§7.3) |
| 6 | `fusionar_mot.py <toma> --salida O --canales cadena,brazos,pelvis --sufijo _con_pelvis` | ídem | `..._opencap_mas_imu_con_pelvis.mot` | lo mismo, más la pelvis del sacro compensando lumbar y caderas |
| 7 | `comparar_mot.py <toma> --mot O --salida N` | los tres `.mot` | `3_analisis/<toma>_comparacion_mot.png`, `_tabla.csv`, `_curvas.csv` | la comparación por ciclo (§9) |
| 8 | `sincronizar.py <csv crudo> <trc> <png>` | CSV crudo + `.trc` | `3_analisis/<toma>_sincronizacion.png` | ver el arranque por golpes de talón y su precisión |

`pipeline.py` además copia el `.mot` de OpenCap y el `.osim` de la sesión a
`2_para_opensim/`, para tener los tres `.mot` y el modelo en un solo lugar.

**Ojo con los pasos 5-6:** usan el CSV **crudo**, no el recortado. El trial de
OpenCap cubre la sesión entera y con el recortado el solape se achica y la
sincronización se cae (r pasa de 0.93 a 0.29). El recorte es para la app y
para el `.mot` reconstruido.



---

## 6. Registrar una sesión nueva (tres ediciones)

Lo único que no se deduce de los datos: qué sensor fue a qué segmento, y qué
toma del IMU corresponde a qué trial de OpenCap. Se escriben una vez por
sesión, en `pc/`.

**a) `mapa_sensores.py` — el mapa MAC → segmento.** Agregar un mapa y su fecha:

```python
MAPA_20261005 = {
    "D7:C0:2D:7B:6F:C9": "cabeza",
    "F8:D1:BE:CA:3E:1B": "espalda",     # sacro
    "DB:A1:10:94:0E:FD": "mano_der",
    "C7:30:05:86:B0:6F": "mano_izq",
    "F0:F3:13:F7:E0:B4": "pie_der",
    "FB:34:B5:D8:F3:4E": "pie_izq",
}
MAPAS = {..., "20261005": MAPA_20261005}     # la fecha del nombre del CSV (YYYYMMDD)
```

La fecha sale del sello que `acquire.py` pone al final del nombre del archivo.
`analizar_toma.py` imprime el mapa que deduce de la señal
(`identificar()`) al lado del declarado: sirve de control, pero **no
distingue mano derecha de izquierda** — eso lo tiene que decir la planilla.

**b) `reconstruir_toma.py` — la sesión de OpenCap y los pares.**

```python
SESIONES = [
    ("9112026", os.path.join(DIR_OC, "sesion_2026-09-11", "OpenCapData"),
     ["TEST1_1", "TEST2"]),
    ("5102026", os.path.join(DIR_OC, "sesion_2026-10-05", "OpenCapData"),   # la nueva
     ["Marcha1", "Marcha2"]),
]

PARES = {"test1normal9112026": "TEST1_1",
         "testnorma2l9112026": "TEST2",
         "marcha01_5102026": "Marcha1",       # tag de la toma -> trial de OpenCap
         "marcha02_5102026": "Marcha2"}
```

Cada entrada de `SESIONES` tiene tres partes. La primera es el texto que
identifica la sesión dentro del tag de la toma (la fecha tal como se escribió
al grabar, por eso los tags tienen que terminar en esa fecha). La segunda es la
carpeta interna del export de OpenCap. La tercera son los trials de OpenCap con
marcha a ritmo normal, que se usan para construir la variedad (§7.2).

**c) Verificar el emparejamiento antes de creer en él.** La correlación de
|ω| del pie derecho entre los dos sistemas dice si la toma y el trial son la
misma pasada:

```bash
python sincronizar.py                                          # todas las tomas contra todos los trials de su sesión
python sincronizar.py "<csv crudo>" "<ruta al .trc del trial>"  # una toma contra un trial
```

Sin argumentos marca con `OK` cada toma cuyo mejor trial coincide con el
registrado en `PARES`. Con dos argumentos imprime `lag_s r`. Con golpes de talón sale también la precisión. Un `r` de
0.3 contra todos los trials = esa toma no tiene par (le pasó a TEST1, la
pasada fallida). En la sesión del 2026-09-11: TEST1_1 r = 0.67 con arranque
+16.8 s; TEST2 r = 0.47, +4.07 s; los golpes confirmaron los dos con 27 y
26 ms de dispersión.

Después de esto, `pipeline.py <tag> --salida CARPETA`.

---

## 7. Los tres `.mot` — qué es cada uno

Los tres tienen las mismas 33 coordenadas del modelo `LaiUhlrich2022` y se
cargan igual en OpenSim:

1. `File > Open Model…` → `2_para_opensim/LaiUhlrich2022_scaled.osim` (**el de
   la misma sesión**: está escalado a ese sujeto).
2. `File > Load Motion…` → el `.mot`.
3. Play. Cargar los tres del mismo trial uno tras otro y se ve la diferencia.

### 7.1 OpenCap puro — `TEST2.mot`, `TEST1_1.mot`
Tal cual lo exporta OpenCap: 100 % video markerless, 60 Hz. **Es la
referencia** de la comparación (con las reservas del §8: OpenCap tiene ~4-5°
de RMSE contra mocap y su pelvis no es confiable en estas tomas).

### 7.2 Solo IMU — `<toma>_reconstruido.mot` (`reconstruir_toma.py`)
Generado **únicamente** a partir del CSV de los 6 sensores. El montaje S mide
la inclinación del sacro y la suma `cadera − rodilla + tobillo` de cada
pierna, pero no cada articulación. Entonces el programa **elige, entre las
caminatas que este mismo sujeto hizo delante de OpenCap** (la "variedad de
marcha", una PCA de sus ciclos), la que mejor explica lo que midieron los IMU,
y la ancla a lo medido: cuándo camina, cadencia, duración de cada zancada,
inclinación de la pelvis.

| | coordenadas |
|---|---|
| **medido** | `pelvis_tilt`, `pelvis_list`; la suma `hip − knee + ankle`; cadencia, ciclos, apoyo |
| **repartido** (la suma es medida, el reparto lo decide el prior) | `hip_flexion`, `knee_angle` |
| **generado** (de la marcha del propio sujeto) | `ankle_angle`, aducción/rotación de cadera, subtalar, mtp, lumbar, brazos, `pelvis_rotation/ty/tz` |
| **sintetizado** | `pelvis_tx` (avance por cadencia; siempre en línea recta, sin magnetómetro no hay rumbo) |

Validado (leave-one-take-out contra OpenCap): en exactitud por articulación
queda **en paridad con reproducir el ciclo medio del sujeto** a la cadencia
medida (rodilla +6 % en promedio, +24 % en la única toma de marcha continua
del 10-sep). **Sirve para animar la caminata real en OpenSim, no como
medición de rodilla o cadera.** `--sin-excluir` (que usa `pipeline.py`) usa
también el trial pareado en la variedad — es para la animación; para sacar
un número nunca.

### 7.3 OpenCap + IMU — `<toma>_opencap_mas_imu.mot` y `_con_pelvis.mot` (`fusionar_mot.py`)
Parte del `.mot` de OpenCap y **le incrusta sólo lo que el IMU mide de
verdad**, dejando el resto intacto. Sirve para aislar el aporte del sensor:

- **pierna** (`cadena`): la suma `hip − knee + ankle` = pitch del pie − pitch
  del sacro. El residuo respecto de OpenCap se reparte 50/50 entre cadera y
  rodilla conservando la suma, respetando el rango del modelo (la rodilla no
  hiperextiende; lo que no cabe va a la cadera).
- **brazos** (`brazos`): `arm_flex + elbow_flex` = pitch de la mano respecto
  de la gravedad. Mismo reparto entre hombro y codo.
- **pelvis** (`pelvis`, sólo en `_con_pelvis`): `pelvis_tilt`/`pelvis_list`
  del sacro. Cada grado que cambia la pelvis **se descuenta** de lumbar y
  caderas para conservar el torso y los muslos que OpenCap sí ve.
- **Nunca**: rotación de pelvis, traslaciones, tobillo aparte, subtalar,
  mtp, `arm_add/arm_rot/pro_sup`. El montaje no los mide.

Sin pose estática el IMU mide bien la **forma** de onda pero no el cero
absoluto: cada señal se lleva a la **mediana** de la de OpenCap antes de
escribirla (`--sin-offset` lo desactiva). La sincronización la hace
`sincronizar.py` (§9.1).

---

## 8. Qué es medido y qué no — para no decir de más

- **La app y `parametros_marcha.py` muestran solo lo medido y validado**
  (cadencia, zancada de las zancadas típicas, apoyo/balanceo, simetría,
  inclinación del pie y cadena pelvis→pie). El largo de zancada por ZUPT, el
  CV sobre todas las zancadas, la pelvis, la cabeza y el braceo quedan en el
  `.json` y en la figura `--detalle`, no en la app.
- **No hay ángulo de rodilla, cadera ni tobillo medido con este montaje.** La
  frase que sí se puede decir: *"reconstruimos la cinemática proyectando la
  cadena sagital medida sobre una variedad de marcha del propio sujeto
  aprendida de video markerless"*.
- **Sin magnetómetro no hay rumbo** (deriva ~1 °/s): nada que dependa del
  yaw (rotación de pelvis, ángulo de progresión del pie, trayectoria). Pitch
  y roll están anclados por la gravedad y no derivan.
- **La pelvis de OpenCap no es referencia** en estas tomas: `pelvis_tilt`
  deriva 25° en 12 s y correlaciona r = −0.98 con `lumbar_extension`
  (reparte la inclinación del torso de forma arbitraria). El r ≈ 0.1 del
  sacro contra ella mide ese artefacto, no al sensor. Por eso la pelvis va en
  un `.mot` aparte.
- **El brazo del IMU sale con menos amplitud** que en OpenCap (12° vs 20° en
  TEST2 izq) con la misma forma (r 0.87-0.91): el eje de flexión de la mano
  sale de una PCA poco nítida. Lo arregla la calibración funcional (§3.3.4).
- Todo esto es **acuerdo entre dos métodos**, ninguno gold-standard.

---

## 9. Cómo leer la comparación (`3_analisis/`)

### 9.1 `_sincronizacion.png`
Arriba, |ω| del pie derecho en los dos sistemas ya alineados. En el medio,
zoom en cada bloque de golpes. Abajo, el arranque que da cada golpe: la
**dispersión es la precisión** (26-27 ms en las dos tomas), la **pendiente
entre bloques es la deriva de reloj** Pi–iPhone (−326 ppm = −11 ms en 35 s
en TEST2: por debajo de la precisión, no medible en una toma de un minuto).
Los golpes con r < 0.6 se descartan (sujeto todavía fuera de cámara).

### 9.2 `_comparacion_mot.png` / `_tabla.csv` / `_curvas.csv`
Los tres `.mot` cortados en **los mismos ciclos** (contacto inicial detectado
en el pie por el IMU y llevado al tiempo de OpenCap con el arranque de la
sincronización), sólo **zancadas típicas en los dos sistemas** (se descartan
los pasitos de las vueltas y los ciclos donde OpenCap es atípico). Una fila
por coordenada, derecha/izquierda, media ± DE. La tabla da por coordenada,
lado y versión: n ciclos, ROM, máximo y mínimo por ciclo, y contra OpenCap
**r** (forma) y **RMSE** (grados).

Lectura: las versiones "+IMU" parten de OpenCap, así que en lo que el IMU no
toca (tobillo) la diferencia es 0 por construcción, y en lo que sí toca **la
diferencia ES el aporte del IMU**. En TEST2 (5 zancadas por lado): rodilla
r = 0.98-0.99 y RMSE 3-4° (2.6° con la pelvis del IMU); cadena de pierna
r = 0.90 / 0.80; cadena de brazo r = 0.91 / 0.87; cadera de OpenCap con
DE entre ciclos de 8° que el IMU baja a 5°.

### 9.3 `_figura.png` (la app)
**A** qué hace el sujeto (marcha / parado / giro / otro; los parámetros salen
solo de "marcha"). **B** ciclo promedio ± DE de la inclinación del pie,
derecha vs izquierda, 0 % = contacto del talón, línea = despegue. **C** lo
mismo para la cadena pelvis→pie (≈ cadera − rodilla + tobillo). **D** el
resumen: cadencia, zancada de las zancadas típicas, apoyo/balanceo, simetría
(Robinson: 0 % = simétrico) y rango del pie y de la cadena.

---

## 10. La app

```bash
cd pc
python app_marcha.py                        # abre; elegir la toma en el desplegable
python app_marcha.py testnorma2l9112026     # abre esa toma directo
```

Dos pestañas: **Gráficas** (la figura de 4 paneles, con la barra de
matplotlib para hacer zoom o guardarla) y **Parámetros** (la misma tabla con
qué significa cada número). El desplegable lista las tomas que encuentra en
`datos/OPENCAP/*/Datos imu de esta sesion/`; «Abrir CSV…» abre cualquier otro
CSV de `acquire.py`/`log.py`. El botón **«Generar .mot para OpenSim»** corre la
reconstrucción (§7.2), avisa que esos ángulos son reconstruidos y deja el
archivo en `resultados/`.

![La app](figuras/app_ventana_graficas.png)

---

## 11. Problemas comunes

| Síntoma | Causa / qué hacer |
|---|---|
| `No encuentro la biblioteca de Mythos MECH` | falta `pc/mythos_mech/`; o `pip install vqf` |
| `no encuentro la toma '<tag>'` | el CSV no está en ninguna carpeta conocida: pasar la ruta completa, o ponerlo en `datos/OPENCAP/<sesión>/Datos imu de esta sesion/` |
| `no tiene par de OpenCap` | el tag no está en `reconstruir_toma.PARES` (§6b) |
| `falta el sensor de sacro` | sin sacro no hay cadena ni pelvis; revisar el mapa (§6a) o el CSV |
| `no se detectó ningún ciclo de marcha` | la toma no tiene marcha continua, o falta un pie en el CSV |
| Signo distinto en los dos pies (aviso de `reconstruir_toma`) | una pierna puede salir espejada: la variedad no representa esa marcha; revisar `TRIALS_VARIEDAD` |
| El modelo camina siempre derecho aunque el sujeto giró | esperado: sin magnetómetro no hay rumbo |
| `recortado al rango del modelo: subtalar_angle_l 13-25 %` | ya viene así del `.mot` de OpenCap (pegado a ±35°), no lo causa la fusión |
| La app da `KeyError: 'mac'` | se abrió un `_series.csv`; abrir un CSV de IMU |
| Sincronización con r < 0.4 y sin golpes | la toma y el trial no son la misma pasada, o poco solape; cruzar contra los otros trials (§6c) |
| Windows no ve archivos dentro de `datos/OPENCAP/...` | rutas de más de 260 caracteres: usar nombres de carpeta cortos (`sesion_<fecha>`) y no poner el repositorio muy adentro |
| En la Pi, se cuelga en `Descriptors found` | caché `.metawear/` de root: `sudo rm -rf ~/indicatic/.metawear`, y nunca más `sudo` |

---

## 12. Los números de referencia de la sesión 2026-09-11

Para saber si una sesión nueva salió parecida (todo en
`resultados/sesion_2026-09-11/3_analisis/`; los valores de la app salen del
tramo recortado):

| | TEST1_1 | TEST2 |
|---|---|---|
| duración útil (tramo) · marcha | 22.6 s · 96 % | 25.1 s · 87 % |
| sincronización | +16.778 s, 7 golpes, 27 ms | +4.066 s, 10 golpes, 26 ms |
| cadencia · zancadas típicas | 68 pasos/min · 2.1 s | 93 pasos/min · 1.77 ± 0.12 s |
| apoyo der / izq | 65 / 70 % | 64 / 62 % |
| ROM pie der / izq (ciclo) | 66 / 69° | 77 / 72° |
| ROM cadena der / izq (ciclo) | 66 / 69° | 79 / 73° |
| rodilla +IMU+pelvis vs OpenCap | r 0.98 / 0.95, RMSE 3.2 / 5.5° (3 ciclos) | r 0.99 / 0.99, RMSE 2.6 / 2.6° (5 ciclos) |
| brazo +IMU vs OpenCap | r 0.44 / 0.71 | r 0.91 / 0.87 |

Lo que más mejoraría la próxima: pasillo de 8-10 m, calibración funcional
de 15 s, golpes al inicio **y** al final siempre, sujeto ya caminando al
entrar en cámara, sacro pegado a la piel, planilla MAC → segmento confirmada
(el mapa de manos del 2026-09-11 se asumió igual al del 01-sep).
