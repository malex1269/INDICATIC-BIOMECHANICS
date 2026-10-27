"""
recortar_toma.py -- deja solo el tramo caminado de una toma.

    python3 recortar_toma.py <csv> [<csv> ...] --salida CARPETA
    python3 recortar_toma.py <csv> --tramo            # solo el tramo continuo mas largo
    python3 recortar_toma.py <csv> --estatico 3.0     # conserva quietud para el IMUPlacer
    python3 recortar_toma.py <csv> --solo-ver         # no escribe nada, solo informa

Las tomas arrancan y terminan con el sujeto parado (acomodandose, esperando el
aviso) y ademas se frenan en el medio. Ese tiempo no aporta y estorba en tres
lugares:

  * la app reporta "parado 23 %" cuando la toma util fue toda marcha;
  * `reconstruir_toma.py` rellena los tramos sin ciclo con la pose neutra, asi
    que en el `.mot` el modelo se queda tieso y despues arranca de golpe --
    esa es buena parte de por que la animacion se ve mal;
  * el `.sto` lleva muestras que no sirven.

El corte se decide con la MISMA segmentacion de actividad que dibuja la app en
el panel A (`parametros_marcha.segmentar_actividad`), no con un umbral aparte:
asi lo que se recorta es exactamente lo que se ve en la figura.

DOS MODOS
    (por omision)  corta lo que no es marcha al principio y al final, y deja
                   el medio como esta -- incluidas las paradas intermedias.
    --tramo        deja SOLO el tramo de marcha continua mas largo. Es el que
                   conviene para parametros y para el `.mot`: da ciclos
                   encadenados de verdad, que es lo que pedia
                   docs/2_sesiones_y_protocolo.md.

SOBRE `--estatico`: OpenSense (IMUPlacer) necesita una pose estatica para
calibrar. Con `--estatico N` el recorte conserva hasta N segundos de quietud
previa a la marcha en vez de tirarlos. Para la app y el `.mot` no hace falta.

Salida: un CSV con el mismo formato de entrada, con `_recortado` en el nombre.
No se toca `epoch_device`: se filtran filas, no se reescriben relojes.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sparse6 as s6
import parametros_marcha as PM

COLUMNAS = ["timestamp_host", "mac", "type", "x", "y", "z", "epoch_device"]


def _epoch_base(df):
    """Instante (s, reloj de dispositivo) que `cargar_toma` toma como t=0.

    Hay que reproducirlo para volver de un indice de la grilla comun al
    `epoch_device` absoluto con el que se filtran las filas.
    """
    t0 = []
    for _mac, g in df.groupby("mac"):
        ini = []
        for tipo in ("acc", "gyro"):
            s = g[g["type"] == tipo]["epoch_device"]
            if len(s) < 100:
                ini = None
                break
            ini.append(s.min() / 1000.0)
        if ini:
            t0.append(max(ini))
    return max(t0)


def _tramos(mascara, t, min_s=0.4):
    """[(t_ini, t_fin, dur)] de las rachas True de al menos `min_s`."""
    d = np.diff(np.concatenate([[0], mascara.astype(np.int8), [0]]))
    ini, fin = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    out = []
    for a, b in zip(ini, fin):
        if t[b - 1] - t[a] >= min_s:
            out.append((float(t[a]), float(t[b - 1]), float(t[b - 1] - t[a])))
    return out


def analizar_tramos(csv, verbose=True):
    """Devuelve (t, mascara_marcha, tramos, res) de la toma."""
    res = PM.analizar(csv, verbose=False)
    t = res.t
    marcha = res.actividad["marcha"]
    tr = _tramos(marcha, t)
    if verbose:
        act = res.spatial["actividad_pct"]
        print("    actividad: marcha %.0f %% | parado %.0f %% | giro %.0f %% |"
              " otro %.0f %%" % (act["marcha"], act["parado"], act["giro"],
                                 act["otro"]))
        for a, b, d in sorted(tr, key=lambda x: -x[2])[:5]:
            print("    tramo de marcha  %6.2f - %6.2f s  (%5.2f s)" % (a, b, d))
    return t, marcha, tr, res


def ventana(t, marcha, tramos, modo="extremos", estatico=0.0, margen=0.5,
            R=None, verbose=True):
    """(t_ini, t_fin) del recorte."""
    if not tramos:
        raise SystemExit("no se detecto ningun tramo de marcha")
    if modo == "tramo":
        a, b, d = max(tramos, key=lambda x: x[2])
        if verbose:
            print("    modo --tramo: me quedo con el mas largo (%.2f s)" % d)
    else:
        a = min(x[0] for x in tramos)
        b = max(x[1] for x in tramos)
    t_ini, t_fin = max(0.0, a - margen), min(float(t[-1]), b + margen)

    if estatico > 0 and R is not None:
        quieto = np.ones(len(t), bool)
        for mac in R:
            quieto &= np.linalg.norm(R[mac][2], axis=1) < 15.0
        antes = quieto.copy()
        antes[int(a * s6.FS):] = False
        rachas = _tramos(antes, t, min_s=0.2)
        if rachas:
            qa, qb, qd = rachas[-1]
            t_ini = max(0.0, qb - min(estatico, qd))
            if verbose:
                print("    estatico conservado: %.2f s (habia %.2f s)"
                      % (min(estatico, qd), qd))
        elif verbose:
            print("    AVISO: no hay ventana quieta antes de la marcha; "
                  "no se puede conservar estatico para el IMUPlacer")
    return t_ini, t_fin


def recortar(csv, salida, modo="extremos", estatico=0.0, margen=0.5,
             solo_ver=False, verbose=True):
    df = pd.read_csv(csv)
    base = _epoch_base(df)
    t, marcha, tramos, _res = analizar_tramos(csv, verbose)
    R, _t = s6.cargar_toma(csv)
    t_ini, t_fin = ventana(t, marcha, tramos, modo, estatico, margen, R, verbose)

    dur_o = (df["epoch_device"].max() - df["epoch_device"].min()) / 1000.0
    if verbose:
        print("    %6.2f s -> %6.2f s   (corta %.2f s al inicio, %.2f s al final)"
              % (dur_o, t_fin - t_ini, t_ini, dur_o - t_fin))
    if solo_ver:
        return None

    e0, e1 = (base + t_ini) * 1000.0, (base + t_fin) * 1000.0
    rec = df[(df["epoch_device"] >= e0) & (df["epoch_device"] <= e1)]
    raiz, ext = os.path.splitext(os.path.basename(csv))
    sufijo = "_tramo" if modo == "tramo" else "_recortado"
    destino = os.path.join(salida, raiz + sufijo + ext)
    rec.to_csv(destino, index=False, columns=COLUMNAS)
    if verbose:
        print("    filas %d -> %d | sensores %d -> %d"
              % (len(df), len(rec), df["mac"].nunique(), rec["mac"].nunique()))
        print("    -> %s" % os.path.basename(destino))
    return destino


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("csv", nargs="+", help="CSV(s) de IMU a recortar")
    p.add_argument("--salida", default=None,
                   help="carpeta destino (por omision, la del CSV)")
    p.add_argument("--tramo", action="store_true",
                   help="quedarse SOLO con el tramo de marcha continua mas "
                        "largo, en vez de recortar los extremos")
    p.add_argument("--margen", type=float, default=0.5,
                   help="segundos a dejar a cada lado. DEFAULT 0.5")
    p.add_argument("--estatico", type=float, default=0.0,
                   help="conservar hasta N s de quietud previa a la marcha, "
                        "para calibrar con el IMUPlacer de OpenSense")
    p.add_argument("--solo-ver", action="store_true", dest="solo_ver",
                   help="informar el recorte sin escribir nada")
    a = p.parse_args()

    for csv in a.csv:
        if "run_log" in csv or os.path.getsize(csv) < 100000:
            continue
        salida = a.salida or os.path.dirname(os.path.abspath(csv))
        if not a.solo_ver and not os.path.isdir(salida):
            os.makedirs(salida)
        print(os.path.basename(csv))
        recortar(csv, salida, "tramo" if a.tramo else "extremos",
                 a.estatico, a.margen, a.solo_ver)
        print()


if __name__ == "__main__":
    main()
