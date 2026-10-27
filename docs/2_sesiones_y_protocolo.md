# 2. Sesiones con sensores inerciales y OpenCap

Los sensores grabaron **en modo log** (`raspberry/log.py`) al mismo tiempo que
**OpenCap** capturaba con dos iPhone. OpenCap devuelve la cinemática por
cinemática inversa sobre el modelo **LaiUhlrich2022** escalado al sujeto
(`.trc` de marcadores a 60 Hz, `.mot` de 33 coordenadas en grados, `.osim`).

## 2.1 Montaje

Se diseñaron dos montajes de seis sensores, porque seis no alcanzan para todo
a la vez:

| Montaje | Sensores | Para qué | Estado |
|---|---|---|---|
| **S** ("sparse-6") | cabeza, sacro, 2 muñecas, 2 pies | equilibrio, tronco, braceo, eventos y tiempos de la marcha | **usado en todas las sesiones** |
| L | sacro, 2 muslos, 2 piernas, pie derecho | ángulos articulares directos (OpenSense) | propuesto, no grabado |

| Sensor | Dónde | Qué aporta |
|---|---|---|
| cabeza | banda elástica, centrado | inclinación de cabeza |
| sacro | S2, **pegado a la piel** con cinta doble faz | inclinación de pelvis; referencia de la cadena de la pierna |
| muñecas | cúbito distal, rígido con el antebrazo | cadena de brazo (hombro + codo) |
| pies | empeine, bajo los cordones | eventos de marcha, inclinación del pie, cadena pelvis→pie |

**No hay sensor en muslo ni pierna:** cadera, rodilla y tobillo no se miden
por separado (ver documento 3).

## 2.2 Sesiones

| Fecha | Tomas IMU | Trials OpenCap | Qué se aprendió |
|---|---|---|---|
| 27-ago | 5 tomas de prueba en modo log | — | el modo log funciona de punta a punta (0 % de pérdida) |
| **01-sep** | 6 caminatas buenas (`caminata03`-`08`) de 63-93 s | 8 (`test3`-`test8_90s`) | el emparejamiento toma↔trial se verificó cruzando todas contra todas (la diagonal gana 5 de 6). Las tomas tenían **solo 29-47 % de marcha** (paradas, giros, acomodos): 42 ciclos en total. El sensor `C7:30` falló entero en 2 tomas. |
| **10-sep** | 3 (`caminata01_4`, `02`, `03`) | 3 (`Caminata1Normal6_1`, `Caminata2Lento1`, `Caminata3Problema`) | primera **marcha continua** (21.4 s). Pasillo de 3.2 m: 3 zancadas, vuelta con pasitos, 3 zancadas. Cabeza y mano izquierda intercambiadas en el montaje. |
| **11-sep** | 2 (`test1normal9112026`, `testnorma2l9112026`) | 3 (`TEST1`, `TEST1_1`, `TEST2`) | **golpes de talón** como evento de sincronización: precisión de 26-27 ms. `TEST1` fue la pasada fallida y no tiene toma de IMU. El sujeto caminó lento (zancada 1.7-2.2 s). |

**Datos incluidos en este repositorio:** la sesión del 11-sep completa
(`datos/OPENCAP/sesion_2026-09-11/`: los dos CSV de los sensores, los `.trc`,
los `.mot` y el modelo escalado de OpenCap), sin los videos. Las sesiones del
01 y 10-sep no se incluyen; para agregar una sesión ver
[`MANUAL_PIPELINE.md`](MANUAL_PIPELINE.md), sección 6.

| Toma IMU | Trial OpenCap | Cómo se verificó |
|---|---|---|
| `test1normal9112026` (70 s) | `TEST1_1` (42 s) | correlación r = 0.67, arranque +16.8 s; 7 golpes alinean con 27 ms |
| `testnorma2l9112026` (63 s) | `TEST2` (52 s) | correlación r = 0.47, arranque +4.1 s; 10 golpes alinean con 26 ms |

## 2.3 Protocolo por toma

Es lo que produjo la mejor toma (`TEST2`) más lo que faltó:

1. Arrancar `log.py` **antes** que OpenCap y que termine **después**: la ventana
   del IMU tiene que contener a la de OpenCap (duración de `log.py` = duración
   de OpenCap + ~20 s).
2. **Parado quieto 5-10 s** (sesgo del giroscopio).
3. **3 golpes secos con el talón derecho**, parado y dentro del campo de las
   cámaras, **al inicio y al final** de la toma.
4. *(No se hizo; es lo que le falta al brazo)* **Calibración funcional, 15 s:**
   3 flexiones lentas de tronco y 3 balanceos de brazos con el codo extendido.
5. Caminar atravesando el cuadro, **ya en marcha al entrar en cámara**: los
   primeros segundos con el sujeto entrando, OpenCap los resuelve mal.
6. Ida y vuelta; las vueltas se filtran solas en el análisis.

Planilla por toma: tag del IMU, nombre del trial de OpenCap, qué sensor va en
qué segmento, y notas.

## 2.4 Para la próxima grabación

1. **Pasillo de 8-10 m.** Con 3 m cada pasada da tres zancadas; es la
   limitación que más pesó en todos los resultados.
2. Tramos de **marcha continua de 20-30 s** a ritmo normal.
3. Calibración funcional dentro de la grabación de los sensores.
4. Golpes de talón al inicio **y** al final, siempre.
5. Confirmar en la planilla cuál muñeca es cuál: la señal no distingue
   derecha de izquierda.
6. Llevar la plataforma a **condiciones reales de transporte** (objetivo de la
   semana 5 del cronograma, que no llegó a hacerse con este montaje).
