# mythos_mech/ — copia parcial de la biblioteca «Mythos MECH» (IIT Gait)

Trabajo previo del IIT (pasantía de investigación de Martín Saldaña). Sólo se
copian los dos subpaquetes que `parametros_marcha.py` importa:

| Carpeta | Qué aporta al pipeline |
|---|---|
| `core/fusion_vqf.py` | fusión de orientación VQF (Laidig & Seel 2023) |
| `kinematics/gait_events.py` | eventos de marcha por giroscopio (Aminian 2002 / Salarian 2004) |
| `kinematics/joint_angles.py` | ROM por ciclo, inclinación vs gravedad |
| `kinematics/parameters.py` | parámetros temporales y espaciales, giros |
| `kinematics/results.py` | el objeto de resultados y su export CSV/JSON |

No se copian los datasets (4.8 GB), el visor 3D ni el export de OpenSense de
Mythos: el pipeline no los usa. Requiere `pip install vqf`.
