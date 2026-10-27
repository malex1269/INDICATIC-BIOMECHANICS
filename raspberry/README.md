# raspberry/ — programa de adquisición

Lo que corre **en la Raspberry Pi**. Conecta los seis sensores MetaMotion por
Bluetooth de baja energía (BLE), los configura, graba y deja un CSV por toma.

| Archivo | Qué es |
|---|---|
| `acquire.py` | El programa completo: diagnóstico, transmisión en vivo, grabación en la memoria del sensor, descarga y todas las pruebas de caracterización. |
| `log.py` | **Lo que se corre en una sesión.** Envoltorio de `acquire.py --mode log` con la configuración validada ya puesta. |
| `fusionar.py` | Junta el CSV de un sensor rescatado con el CSV de la sesión. Solo si falló una descarga. |

Los tres van **en la misma carpeta** de la Pi (`log.py` busca a `acquire.py`
al lado). Sugerido: `~/indicatic/`.

```bash
scp acquire.py log.py fusionar.py pi@raspberrypi.local:~/indicatic/
```

---

## Requisitos

- **Raspberry Pi 4B con Raspbian de 32 bits.** En 64 bits el SDK de MbientLab
  falla por sus bindings de C++.
- **SDK de MbientLab** en `~/MetaWear-SDK-Python/`.
  Comprobar: `python3 -c "import mbientlab; print('ok')"`.
- **Tres adaptadores Bluetooth:** el integrado de la Pi y dos dongles USB, en
  puertos **USB 2.0** (los negros). El controlador USB 3.0 irradia en 2.4 GHz,
  la misma banda de BLE. Con alargue y la antena vertical.
- **Seis sensores MetaMotion** (chip BMI160) cargados. La lista de MAC está en
  `acquire.py` (`DEFAULT_MACS`) y en `log.py` (`MACS`); si se cambian los
  sensores hay que editar las dos. El orden es el orden de conexión: con
  2+2+2 los dos primeros van al Bluetooth integrado, los dos siguientes al
  primer dongle y los dos últimos al segundo.

| MAC | Uso en las sesiones del 01, 10 y 11/09 |
|---|---|
| `F8:D1:BE:CA:3E:1B` | sacro |
| `D7:C0:2D:7B:6F:C9` | cabeza (el 10/09 se intercambió con `C7:30`) |
| `DB:A1:10:94:0E:FD` | muñeca derecha (reemplazó a `EE:9D:A0:09:C6:11`, que no grababa) |
| `C7:30:05:86:B0:6F` | muñeca izquierda |
| `F0:F3:13:F7:E0:B4` | pie derecho |
| `FB:34:B5:D8:F3:4E` | pie izquierdo |

El mapa sensor → segmento de cada sesión se declara en
`../pc/mapa_sensores.py`; anotarlo siempre en la planilla de la sesión.

---

## Grabar una sesión

```bash
cd ~/indicatic
python3 acquire.py --mode check      # 1. diagnóstico: adaptadores, WiFi, USB. No seguir si se queja.
python3 acquire.py --mode battery    # 2. batería y modelo de cada sensor
python3 log.py caminata01 90         # 3. graba 90 s y descarga al final
```

- `log.py <tag> <segundos>` da **10 s de cuenta regresiva**, prende el LED
  **verde** de cada sensor mientras graba (rojo parpadeando = ese sensor no
  arrancó y no va a tener datos) y al terminar **descarga por Bluetooth**.
- Durante la toma el sujeto puede alejarse todo lo que quiera: los datos van a
  la memoria de cada sensor. **Hay que volver cerca de la Pi antes de que
  termine la cuenta**, porque la descarga es inalámbrica.
- Configuración que aplica: modo log, 100 Hz, acelerómetro + giroscopio,
  intervalo de conexión 30 ms, dos sensores por adaptador. Queda en el nombre
  del archivo: `<tag>_log_split_6dev_raw_100hz_ci30_<fecha>_<hora>.csv`.
- **Nunca con `sudo`.** Con `sudo` la caché `.metawear/` queda de root y la
  corrida siguiente se cuelga para siempre en `Descriptors found`.
  `log.py` lo detecta; si ya pasó: `sudo rm -rf ~/indicatic/.metawear`.

### Si un sensor no se descargó

Los datos **siguen en su memoria**. Se rescatan con un trozo inconfundible de
la MAC y se juntan con la sesión:

```bash
python3 log.py rescate C7                 # un sensor (o varios: rescate C7 F0)
python3 fusionar.py <sesion>.csv <rescate>.csv     # escribe <sesion>_completo.csv
```

### Copiar los datos a la PC

```bash
sync                                        # en la Pi, SIEMPRE antes de copiar
scp -r pi@raspberrypi.local:~/datos_v3 ./   # en la PC
```

Si se usa una memoria USB: copiar, `sync`, y recién ahí sacarla. Raspbian
cachea las escrituras y se perdió una corrida entera por no hacerlo.

---

## Los modos de `acquire.py`

| Modo | Qué hace | Cuándo |
|---|---|---|
| `--mode check` | Diagnóstico sin conectar nada: adaptadores arriba, cuál es integrado y cuál USB, WiFi, puertos. | Antes de cada sesión. |
| `--mode battery` | Conecta a cada sensor, lee batería y modelo, y se desconecta. | Antes de cada sesión. |
| `--mode scan` | Busca los sensores sin conectarse (necesita `sudo`). Responde si el problema es el sensor o el adaptador. | Cuando uno no conecta. |
| `--mode stream` | **Transmisión en vivo** por BLE (por defecto). Con o sin empaquetado. | Pruebas de conectividad; datos en vivo con la Pi cerca. |
| `--mode log` | **Grabación en la memoria del sensor** y descarga al final de la misma corrida. 0 % de pérdida por distancia. | Todas las sesiones con OpenCap (vía `log.py`). |
| `--mode download` | Rescata un log que quedó en los sensores. | Si falló una descarga (vía `log.py rescate`). |

### Opciones principales

| Opción | Por defecto | Qué controla |
|---|---|---|
| `--adapter` | `split` | `split` (reparte entre adaptadores), `onboard` (solo el integrado), `dongle` (solo el USB). Los adaptadores se identifican por tipo de bus, no por `hci0/hci1`, porque Linux cambia ese número entre reinicios. |
| `--por-adaptador` | `3` | Sensores por adaptador en `split`. **Con 2 y tres adaptadores queda 2+2+2**, el reparto recomendado (es el que usa `log.py`). |
| `--sensors` / `--macs` / `--rotate` | 6 / lista / 0 | Cuántos sensores, cuáles y en qué orden de conexión. |
| `--odr` | `100` | Frecuencia de muestreo (Hz). En vivo, 100 Hz es el techo medido. |
| `--packed` / `--no-packed` | packed | Empaquetado del SDK (3 muestras por paquete BLE). Sin él, a 100 Hz el giroscopio pierde 34-89 %. |
| `--conn-interval` | `30` ms | Intervalo de conexión BLE. El original era 7.5 ms, que limita a ~3 conexiones por adaptador. |
| `--supervision` | `16000` ms | Tiempo sin paquetes antes de dar el enlace por muerto (era 6000). |
| `--tx-power` | `+4` dBm | Potencia de transmisión del sensor (venía en 0). |
| `--reconnect` / `--no-reconnect` | activada | Reconexión automática de un sensor que se cae en plena corrida. |
| `--rssi` | apagado | Guarda la potencia recibida de cada enlace una vez por segundo (necesita `sudo`). |
| `--duration`, `--espera` | 60 s, 10 s | Duración de la toma y cuenta regresiva (modo log). |
| `--tag`, `--dongle`, `--nota`, `--outdir` | — | Etiquetas que quedan en el nombre del archivo y en la bitácora. |

Ejemplos de las pruebas de caracterización (ver
`../docs/1_caracterizacion_plataforma.md`):

```bash
python3 acquire.py --tag base --duration 120 --odr 50 --no-packed     # estabilidad
python3 acquire.py --tag orden --rotate 1 --odr 50 --no-packed         # efecto del orden de conexión
python3 acquire.py --tag odr2 --odr 200                                # escalera de frecuencia
python3 acquire.py --tag drift2 --duration 900                         # 15 min en reposo
python3 acquire.py --tag ci_test --adapter onboard --conn-interval 7.5 # intervalo de conexión
python3 acquire.py --tag dongles --dongle tplink_ant --por-adaptador 2 # comparar dongles
```

---

## Salidas

- **Un CSV por corrida**, una fila por muestra:
  `timestamp_host, mac, type (acc|gyro), x, y, z, epoch_device`.
  Acelerómetro en g, giroscopio en °/s. `epoch_device` es el reloj **del propio
  sensor** (ms) y es el que vale para el análisis temporal; `timestamp_host`
  incluye el jitter de la transmisión. Cada sensor corre a su propio ritmo
  (98.4-101.3 Hz con el mismo ODR), así que alinearlos es trabajo del análisis.
- **`run_log.csv`**, una fila por sensor y por corrida: Hz efectivo, pérdida por
  canal, caídas, reconexiones, segundos sin datos, adaptador usado y bytes del
  adaptador. Se arma solo; es la base de todas las tablas de caracterización.

---

## Problemas conocidos

| Síntoma | Qué hacer |
|---|---|
| Se cuelga en `Descriptors found` | Caché `.metawear/` de root: `sudo rm -rf ~/indicatic/.metawear` y no volver a usar `sudo`. |
| Ningún sensor conecta, mismo error en todos | Enlace colgado en BlueZ después de un Ctrl-C: `sudo hcitool -i hci0 con` para verlo, `sudo systemctl restart bluetooth` y `sudo hciconfig hciN reset`. |
| Crash de C++ (`BLEEP::SocketConnectFailed`) | El dongle quedó en un puerto USB 3.0: pasarlo a USB 2.0. |
| `Segmentation fault` al terminar | Es de la librería del SDK al cerrar; los datos ya quedaron guardados. Dejar unos segundos entre corridas. |
| Un sensor "conectado" con 0 % de datos | Falló la configuración en silencio; reiniciar el adaptador y repetir. `log.py` lo avisa con el LED rojo. |
| Los tres primeros sensores dejan de conectar después de apagar el WiFi | El WiFi y el Bluetooth integrado son el mismo chip: `sudo hciconfig hci0 down; sudo hciconfig hci0 up`. |

Orden de diagnóstico cuando un sensor no conecta: batería (`--mode battery`) →
¿se ve? (`sudo python3 acquire.py --mode scan`) → ¿enlace colgado?
(`sudo hcitool -i hci0 con`) → reiniciar adaptadores → reiniciar la Pi.
