"""
mot_io.py -- escribe un .mot que OpenSim abre sin quejarse.

El formato es exactamente el del export de OpenCap (verificado contra
OPENCAP/MOT/test4_60s.mot), asi que el archivo que sale de aca es
intercambiable con uno de OpenCap: mismas 34 columnas, mismo orden, mismas
unidades, angulos en grados y traslaciones de pelvis en metros.

Para verlo caminar:  OpenSim GUI -> abrir LaiUhlrich2022_scaled.osim
                                 -> File > Load Motion > este archivo

No hace falta el paquete `opensim`: un .mot son datos, no una simulacion.
"""
import math
import re

import numpy as np

# Orden EXACTO del export de OpenCap. No reordenar: OpenSim mapea por nombre,
# pero mantenerlo igual hace que los dos archivos se puedan comparar y
# diffear directamente.
COLUMNAS = [
    "pelvis_tilt", "pelvis_list", "pelvis_rotation",
    "pelvis_tx", "pelvis_ty", "pelvis_tz",
    "hip_flexion_r", "hip_adduction_r", "hip_rotation_r",
    "knee_angle_r", "ankle_angle_r", "subtalar_angle_r", "mtp_angle_r",
    "hip_flexion_l", "hip_adduction_l", "hip_rotation_l",
    "knee_angle_l", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l",
    "lumbar_extension", "lumbar_bending", "lumbar_rotation",
    "arm_flex_r", "arm_add_r", "arm_rot_r", "elbow_flex_r", "pro_sup_r",
    "arm_flex_l", "arm_add_l", "arm_rot_l", "elbow_flex_l", "pro_sup_l",
]

CABECERA = (
    "Coordinates\n"
    "version=1\n"
    "nRows=%d\n"
    "nColumns=%d\n"
    "inDegrees=yes\n"
    "\n"
    "Units are S.I. units (second, meters, Newtons, ...)\n"
    "If the header above contains a line with 'inDegrees', this indicates "
    "whether rotational values are in degrees (yes) or radians (no).\n"
    "endheader\n"
)


def limites_osim(ruta_osim):
    """{coordenada: (min, max)} en las unidades del .mot: grados y metros.

    Los limites los define el modelo, no nosotros. Importa de verdad: en
    `LaiUhlrich2022` la rodilla es **[0, 140] grados** -- la hiperextension
    no existe -- y lo mismo `elbow_flex` [0, 150] y `pro_sup` [0, 119.7]. Un
    valor negativo de rodilla no es "un poco raro": es imposible en el
    modelo, y en la animacion se ve como una pierna quebrada al reves.
    """
    s = open(ruta_osim).read()
    lim = {}
    for m in re.finditer(r'<Coordinate name="([^"]+)">(.*?)</Coordinate>', s, re.S):
        nom, cuerpo = m.group(1), m.group(2)
        r = re.search(r"<range>([^<]+)</range>", cuerpo)
        if not r or nom not in COLUMNAS:
            continue
        a, b = [float(x) for x in r.group(1).split()]
        if not nom.endswith(("_tx", "_ty", "_tz")):
            a, b = math.degrees(a), math.degrees(b)
        lim[nom] = (a, b)
    return lim


def escribir_mot(ruta, t, datos, limites=None):
    """Escribe el .mot.

    t        : [N] tiempo en segundos (se recentra para arrancar en 0).
    datos    : dict nombre_columna -> [N]. Las que falten se escriben en 0,
               que es la pose neutra del modelo; asi nunca sale un archivo
               incompleto que OpenSim rechace a la mitad.
    limites  : dict de `limites_osim()`. Si se pasa, recorta cada columna a
               su rango legal. Devuelve tambien cuanto hubo que recortar:
               un porcentaje alto es una senal de que algo esta mal, no un
               detalle cosmetico.
    """
    t = np.asarray(t, float)
    t = t - t[0]
    n = len(t)
    faltan = [c for c in COLUMNAS if c not in datos]
    recortado = {}

    M = np.zeros((n, len(COLUMNAS)))
    for j, c in enumerate(COLUMNAS):
        if c in datos:
            x = np.asarray(datos[c], float)
            if len(x) != n:
                raise ValueError("la columna %s tiene %d filas, se esperaban %d"
                                 % (c, len(x), n))
            x = np.nan_to_num(x, nan=0.0)
            if limites and c in limites:
                a, b = limites[c]
                fuera = float(np.mean((x < a) | (x > b)) * 100)
                if fuera > 0.05:
                    recortado[c] = fuera
                x = np.clip(x, a, b)
            M[:, j] = x

    with open(ruta, "w") as f:
        f.write(CABECERA % (n, len(COLUMNAS) + 1))
        f.write("time\t" + "\t".join(COLUMNAS) + "\n")
        for i in range(n):
            f.write("%.6f\t" % t[i])
            f.write("\t".join("%.6f" % x for x in M[i]) + "\n")
    return ruta, faltan, recortado
