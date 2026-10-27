# INDICATIC Biomechanics

Plataforma de seis sensores inerciales MetaMotionRL conectados por Bluetooth a
una Raspberry Pi 4B, caracterizada e integrada con OpenCap (captura de
movimiento por video) y OpenSim (modelo musculoesquelético). Es el trabajo de
la pasantía de investigación en biomédica realizada en INDICATIC AIP del 17 de
agosto al 25 de septiembre de 2026, bajo la supervisión de la Dra. Milagros
Jaén Vargas.

La plataforma parte de un trabajo previo del grupo que demostró que se podían
combinar sensores portátiles con OpenCap para estudiar el equilibrio de
pasajeros con discapacidades que limitan el uso del transporte. En esta
pasantía se midió cómo rinde la plataforma, se corrigió lo que la limitaba y se
llevaron sus datos hasta un resultado biomecánico.

![La aplicación de parámetros de marcha](docs/figuras/app_ventana_graficas.png)

## Qué se encontró

El límite de tres sensores por adaptador Bluetooth que reportaba el trabajo
previo no era del hardware, sino del intervalo de conexión que usaba el
programa original. Repartiendo dos sensores en cada uno de tres adaptadores la
pérdida de datos baja a menos de 0.2 %, y con el modo empaquetado los seis
sensores transmiten a 100 Hz. Con el sujeto en movimiento la transmisión en
vivo se corta, por eso las sesiones se grabaron en la memoria de cada sensor,
lo que elimina la pérdida por completo.

Los sensores se sincronizan con OpenCap mediante tres golpes de talón, con una
precisión cercana a 27 ms. Con los sensores en la cabeza, el sacro, las muñecas
y los pies se miden de forma confiable los tiempos de la marcha. Incrustados en
el archivo de OpenCap, los sensores reproducen el movimiento de la rodilla con
un error cercano a 3°.

## Qué hay en el repositorio

| Carpeta | Contenido |
|---|---|
| [`raspberry/`](raspberry/README.md) | El programa de adquisición que corre en la Raspberry Pi. |
| `pc/` | La aplicación de parámetros de marcha y el análisis que lleva los datos a OpenSim. |
| [`analisis_conectividad/`](analisis_conectividad/README.md) | Los scripts de las pruebas de conectividad. |
| `datos/` | Una sesión completa de ejemplo (sensores y OpenCap del 11/09) y las bitácoras de las pruebas. |
| `resultados/` | Lo que produce el análisis con esa sesión, incluidos los archivos para OpenSim. |
| [`docs/`](docs/MANUAL_PIPELINE.md) | La explicación del método, los resultados y el manual paso a paso. |
| `informe/` | El informe final de la pasantía. |

## Cómo usarlo

Hace falta Python 3.10 o más nuevo. En Windows conviene clonar el repositorio
en una ruta corta, por ejemplo `C:\indicatic`, porque algunos archivos de datos
quedan cerca del límite de 260 caracteres.

```bash
git clone https://github.com/malex1269/INDICATIC-BIOMECHANICS.git
cd INDICATIC-BIOMECHANICS/pc
pip install -r requirements.txt
```

Para abrir la aplicación con una toma de ejemplo:

```bash
python app_marcha.py testnorma2l9112026
```

Para correr el análisis completo de una toma, que genera la figura de la
aplicación, los tres archivos de movimiento y su comparación (tarda alrededor
de un minuto):

```bash
python pipeline.py testnorma2l9112026 --salida ../resultados/sesion_2026-09-11
```

Para ver la caminata en OpenSim se abre el modelo
`resultados/sesion_2026-09-11/2_para_opensim/LaiUhlrich2022_scaled.osim` con
`File > Open Model` y después cualquiera de los `.mot` de esa carpeta con
`File > Load Motion`.

Para grabar una sesión nueva con los sensores, las instrucciones están en
[`raspberry/README.md`](raspberry/README.md). El proceso completo, desde la
grabación hasta OpenSim, está en [`docs/MANUAL_PIPELINE.md`](docs/MANUAL_PIPELINE.md).

## Los tres archivos de movimiento

El primero es el que entrega OpenCap y sirve de referencia. En el segundo, las
partes del movimiento que miden los sensores (la cadena de la pierna, la del
brazo y, en una variante, la pelvis) reemplazan a las del video. El tercero se
genera solo con los sensores y anima el modelo completo. Como el montaje no
tiene sensores en muslos ni piernas, en ese tercer archivo la cadera, la
rodilla y el tobillo se reconstruyen a partir de la marcha del propio sujeto,
así que sirve para animar el modelo y no como medición.

## Documentación

1. [Caracterización de la plataforma](docs/1_caracterizacion_plataforma.md)
2. [Sesiones y protocolo de grabación](docs/2_sesiones_y_protocolo.md)
3. [Análisis y validación](docs/3_analisis_y_validacion.md)
4. [Integración con OpenSim](docs/4_integracion_opensim.md)
5. [Manual del pipeline](docs/MANUAL_PIPELINE.md)

## Limitaciones

Todo se probó con un solo sujeto y en un pasillo de 3 m, que da pocas zancadas
seguidas por pasada. Sin magnetómetro la orientación horizontal deriva, así que
no se reporta nada que dependa de ella. La plataforma todavía no se probó en
condiciones reales de transporte.

## Autoría

Martín Saldaña, Licenciatura en Ingeniería Biomédica, Universidad Latina de
Panamá. Caracterización de la plataforma, programa de adquisición, análisis de
las sesiones con OpenCap, aplicación de parámetros de marcha e integración con
OpenSim. Las sesiones de las semanas cinco y seis las condujo Obed Aparicio
(UDELAS) y no forman parte de este repositorio.

La biblioteca de `pc/mythos_mech/` proviene de la pasantía del autor en
Illinois Institute of Technology.
