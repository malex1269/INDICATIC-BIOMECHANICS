"""
mapa_sensores.py -- MAC -> segmento del cuerpo, y el identificador automatico.

Los mapas los confirma el operador, uno por sesion: **el mismo sensor no
siempre va al mismo segmento**, y eso ya paso una vez (ver MAPA_20260910).
Por eso las tomas se resuelven con `mapa_de(csv)`, que elige el mapa por la
fecha del nombre del archivo, y no importando un mapa fijo.

`identificar()` deduce el mapa solo de la senal; se dejo porque reproduce el
del 2026-09-01 sin planilla y sirve de control. Ojo con su limite: distingue
las manos por el braceo, asi que en una toma donde el sujeto no bracea no
sirve para separar mano de cabeza.
"""
import os
import re

import numpy as np

# Sesion 2026-09-01, montaje S (sparse-6). Confirmado por el operador:
# cabeza d7 / espalda f8 / mano_der db / mano_izq c7 / pie_der f0 / pie_izq fb
MAPA_20260901 = {
    "D7:C0:2D:7B:6F:C9": "cabeza",
    "F8:D1:BE:CA:3E:1B": "espalda",     # sacro / L5
    "DB:A1:10:94:0E:FD": "mano_der",
    "C7:30:05:86:B0:6F": "mano_izq",
    "F0:F3:13:F7:E0:B4": "pie_der",
    "FB:34:B5:D8:F3:4E": "pie_izq",
}

# Sesion 2026-09-10, mismo montaje S. Confirmado por el operador: el sensor de
# la CABEZA y el de la MANO IZQUIERDA se pusieron intercambiados respecto del
# 2026-09-01. Los otros cuatro no se movieron.
#
# La senal lo respalda: |w| mediana de c7 = 12-16 deg/s en las tres tomas, el
# rango de la cabeza del 2026-09-01 (12-23), mientras que d7 da 40-60 deg/s,
# el rango de las manos (48-69). `identificar()` sobre la toma con mas marcha
# (caminata01_4, 88 %) tambien lo reproduce.
MAPA_20260910 = dict(MAPA_20260901,
                     **{"C7:30:05:86:B0:6F": "cabeza",
                        "D7:C0:2D:7B:6F:C9": "mano_izq"})

# Sesion 2026-09-11 (tomas `test1normal9112026`, `testnorma2l9112026`). El
# montaje volvio al del 2026-09-01: `identificar()` da C7 = mano y D7 = cabeza
# en las dos tomas (|w| con los pies plantados: C7 5-12 deg/s, DB 7-17, D7 4-6;
# D7 solo se mueve al agacharse). Cual mano es cual no sale de la senal; se
# asume DB = derecha, C7 = izquierda como el 2026-09-01. PENDIENTE de confirmar
# por el operador.
MAPA_20260911 = dict(MAPA_20260901)

# fecha de sesion (YYYYMMDD) -> mapa. La fecha sale del sello de corrida que
# `acquire.py` pone al final del nombre: <...>_YYYYMMDD_HHMMSS.csv
MAPAS = {"20260901": MAPA_20260901,
         "20260910": MAPA_20260910,
         "20260911": MAPA_20260911}


def mapa_de(csv, aviso=True):
    """Mapa MAC->segmento de la toma, elegido por la fecha del nombre.

    Una sesion sin mapa declarado cae en el mas reciente y **avisa**: elegir
    en silencio es justo lo que hizo pasar desapercibido el intercambio
    cabeza/mano_izq del 2026-09-10 durante todo un procesamiento.
    """
    m = re.search(r"_(\d{8})_\d{6}", os.path.basename(csv))
    if m and m.group(1) in MAPAS:
        return MAPAS[m.group(1)]
    ultima = max(MAPAS)
    if aviso:
        print("  AVISO: '%s' no tiene mapa de sensores declarado; uso el de %s."
              " Si el montaje cambio, agregalo a mapa_sensores.MAPAS."
              % (os.path.basename(csv), ultima))
    return MAPAS[ultima]


# Nombre del frame de OpenSense por segmento (modelo de OpenCap/OpenSim).
FRAME_OPENSENSE = {
    "cabeza": "torso_imu",
    "espalda": "pelvis_imu",
    "mano_der": "hand_r_imu",
    "mano_izq": "hand_l_imu",
    "pie_der": "calcn_r_imu",
    "pie_izq": "calcn_l_imu",
}


def identificar(R, fs=100.0):
    """Deduce el segmento de cada MAC solo de la senal.

    Discriminantes, en orden:
      1. Pies: unicos con ZUPT verdadero (|w| < 5 deg/s y |a| clavado en 1 g
         durante mas de 0.3 s). Ningun otro segmento se detiene asi.
      2. Manos: durante esas mismas ventanas de pie plantado, siguen con
         |w| alto (el sujeto mueve los brazos aunque este parado).
      3. Espalda vs cabeza: la espalda es la mas quieta de las dos con los
         pies plantados, y la que tiene MAS varianza de |a| caminando (la
         cabeza atenua el choque del talon).
    """
    macs = sorted(R)
    W = {m: np.linalg.norm(R[m][2], axis=1) for m in macs}
    A = {m: np.linalg.norm(R[m][1], axis=1) for m in macs}
    n_min = int(0.3 * fs)

    quieto = {}
    for m in macs:
        q = (W[m] < 5.0) & (np.abs(A[m] - 1.0) < 0.05)
        # exigir racha sostenida
        acum, run = np.zeros(len(q), bool), 0
        for i, v in enumerate(q):
            run = run + 1 if v else 0
            if run >= n_min:
                acum[i - run + 1:i + 1] = True
        quieto[m] = acum

    frac = {m: quieto[m].mean() for m in macs}
    pies = sorted(macs, key=lambda m: -frac[m])[:2]

    plantado = quieto[pies[0]] & quieto[pies[1]]
    if plantado.sum() < 50:
        plantado = quieto[pies[0]] | quieto[pies[1]]
    resto = [m for m in macs if m not in pies]
    mov = {m: W[m][plantado].mean() for m in resto}
    manos = sorted(resto, key=lambda m: -mov[m])[:2]
    tronco = [m for m in resto if m not in manos]

    caminando = ~(quieto[pies[0]] | quieto[pies[1]])
    var_a = {m: A[m][caminando].std() for m in tronco}
    espalda = max(tronco, key=lambda m: var_a[m])
    cabeza = [m for m in tronco if m != espalda][0]

    out = {espalda: "espalda", cabeza: "cabeza"}
    for m in manos:
        out[m] = "mano_?"
    for m in pies:
        out[m] = "pie_?"
    diag = dict(frac_zupt=frac, mov_con_pies_plantados=mov, var_acc=var_a)
    return out, diag
