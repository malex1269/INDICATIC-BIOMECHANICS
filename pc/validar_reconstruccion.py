"""
validar_reconstruccion.py -- que tan bien reconstruye, medido contra OpenCap.

    python3 validar_reconstruccion.py

Leave-one-take-out sobre las tomas emparejadas de reconstruir_toma.PARES:
para cada una, la variedad se construye con los OTROS trials de su sesion y
despues se reconstruye esa. Nunca se
reconstruye un ciclo que ayudo a construir la base -- si no, el numero mide
memoria, no metodo.

Compara dos reconstrucciones contra el `.mot` de OpenCap:

  B0  el ciclo medio de la variedad, estirado a la cadencia medida.
      Es "la caminata generica de este sujeto". La linea base honesta.
  M   el metodo: B0 + lo que aporta la cadena sagital medida por el IMU.

Lo publicable es **M contra B0**, por articulacion. Que M le gane a nada no
dice nada; que le gane al ciclo medio del propio sujeto si.

Solo se comparan los tramos donde hubo ciclo de marcha reconstruido: en el
65-75 % de la toma el sujeto esta parado, y ahi ninguna de las dos
reconstrucciones esta diciendo nada.

TECHO: la variedad se entrena contra OpenCap y se valida contra OpenCap, asi
que el techo del metodo es OpenCap (~4-5 deg RMSE contra mocap optico). Estos
numeros son "acuerdo con OpenCap", no "verdad absoluta".
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import opencap_io as oc
import sincronizar as S
import reconstruir_toma as R
import variedad as V

ARTICULACIONES = ["hip_flexion", "knee_angle", "ankle_angle"]
LADOS = [("_r", "pie_der"), ("_l", "pie_izq")]


def _rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def rutas_oc(cam, trial):
    """(trc, mot) del trial, segun la sesion a la que pertenece la toma."""
    carpeta, _trials = R.dir_oc_de(cam)
    return R.rutas_trial(carpeta, trial)


def evaluar(cam, trial, verbose=True, csv=None):
    """RMSE de B0 y de M contra OpenCap, para una toma."""
    csv = csv or R.buscar_csv(cam)
    v = R.obtener_variedad(None, excluir=trial, verbose=False, tag=cam)

    t, dM = R.reconstruir(csv, v, verbose=False)
    _, dB = R.reconstruir(csv, v, verbose=False, baseline=True)

    trc, ruta_mot = rutas_oc(cam, trial)
    sync = S.sincronizar(csv, trc, verbose=False)
    off = sync["arranque_s"]
    to, mot = oc.leer_mot(ruta_mot)

    # tiempo de OpenCap -> indice en la grilla del IMU (100 Hz)
    idx = np.clip(((to + off) * 100.0).astype(int), 0, len(t) - 1)
    marcha = dM["_marcha"][idx]
    if marcha.sum() < 200:
        if verbose:
            print("  %-12s solape insuficiente (%d muestras)" % (cam, marcha.sum()))
        return None

    fila = {}
    for suf, _ in LADOS:
        for art in ARTICULACIONES:
            col = art + suf
            if col not in mot.columns:
                continue
            ref = mot[col].to_numpy(float)[marcha]
            fila[col] = (_rmse(dB[col][idx][marcha], ref),
                         _rmse(dM[col][idx][marcha], ref))
    if verbose:
        print("  %-12s vs %-10s  %5d muestras comparadas (r_sync=%.2f)"
              % (cam, trial, int(marcha.sum()), sync["r"]))
    return fila


def main():
    print(__doc__.split("\n")[1])
    print("=" * 74)
    filas = []
    for cam, trial in R.PARES.items():
        try:
            f = evaluar(cam, trial)
        except (SystemExit, RuntimeError) as e:
            # p. ej. la variedad queda sin ciclos al sacar el unico trial
            # de la sesion con marcha a ritmo normal
            print("  %-12s %s" % (cam, e))
            continue
        if f:
            filas.append(f)

    if not filas:
        print("\nno se pudo evaluar ninguna toma")
        return

    print()
    print("=" * 74)
    print("RMSE contra OpenCap, %d tomas, leave-one-take-out" % len(filas))
    print("=" * 74)
    print("  %-16s %10s %10s %10s" % ("", "B0 (medio)", "M (metodo)", "mejora"))
    for art in ARTICULACIONES:
        for suf, _ in LADOS:
            col = art + suf
            b = [f[col][0] for f in filas if col in f]
            m = [f[col][1] for f in filas if col in f]
            if not b:
                continue
            b0, m0 = float(np.mean(b)), float(np.mean(m))
            print("  %-16s %9.1f%s %9.1f%s %9.0f %%"
                  % (col, b0, "°", m0, "°", (b0 - m0) / b0 * 100))
    print()
    for art in ARTICULACIONES:
        b = [f[c][0] for f in filas for c in f if c.startswith(art)]
        m = [f[c][1] for f in filas for c in f if c.startswith(art)]
        b0, m0 = float(np.mean(b)), float(np.mean(m))
        print("  %-16s %9.1f%s %9.1f%s %9.0f %%   <- los dos lados juntos"
              % (art, b0, "°", m0, "°", (b0 - m0) / b0 * 100))


if __name__ == "__main__":
    main()
