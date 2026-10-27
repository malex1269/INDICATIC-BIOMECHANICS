"""
analizar_cobertura.py — que tan bien se sostiene el enlace a lo largo de
una sesion, sensor por sensor.

Para que sirve: en una sesion de laboratorio (sensores junto a la Pi) la
perdida es un numero plano y aburrido. En campo (sensores puestos en el
cuerpo, sujeto caminando) lo que importa no es el promedio sino CUANDO se
cae cada enlace y si se recupera. Un 99% de perdida promedio puede
significar "llego poquito todo el rato" o "llego bien 15 s y despues nada",
que son problemas distintos con soluciones distintas.

Este script grafica la tasa de muestras recibidas por segundo de cada
sensor a lo largo del tiempo, que responde justo eso.

Si la corrida se hizo con --rssi, pasarle tambien el CSV de RSSI agrega un
segundo panel con la potencia recibida de cada enlace. Eso convierte un
"se cayo a los 18 s" en "se cayo cuando el RSSI paso de -85 dBm", que es
un umbral reutilizable para decidir distancias y colocacion.

Uso:
    python3 analizar_cobertura.py sesion.csv --odr 100 --out cobertura.png
    python3 analizar_cobertura.py sesion.csv --rssi sesion_rssi.csv
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_session

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
# Orden categorico fijo del sistema de diseno (identidad por sensor).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]


def main():
    p = argparse.ArgumentParser(description="Cobertura del enlace a lo largo del tiempo")
    p.add_argument("csv_path")
    p.add_argument("--odr", type=int, default=100, help="ODR nominal de la sesion")
    p.add_argument("--out", default="cobertura.png")
    p.add_argument("--titulo", default=None)
    p.add_argument("--rssi", default=None,
                   help="CSV de RSSI de la misma corrida. Si no se pasa, se busca "
                        "solo el archivo <sesion>_rssi.csv al lado del CSV de datos.")
    a = p.parse_args()

    df = load_session(a.csv_path)
    g = df[df["type"] == "gyro"].copy()
    if len(g) == 0:
        print("La sesion no tiene muestras de giroscopio.")
        return 1

    t0 = g["timestamp_host"].min()
    g["s"] = (g["timestamp_host"] - t0).dt.total_seconds()
    dur = g["s"].max()
    g["seg"] = g["s"].astype(int)

    conteo = g.pivot_table(index="seg", columns="mac", values="s", aggfunc="count")
    conteo = conteo.reindex(range(int(dur) + 1)).fillna(0)

    # Si no lo pasaron a mano, buscar el CSV de RSSI hermano: acquire.py lo
    # guarda siempre con el mismo nombre mas el sufijo _rssi.
    ruta_rssi = a.rssi
    if ruta_rssi is None:
        candidato = a.csv_path[:-4] + "_rssi.csv" if a.csv_path.endswith(".csv") else None
        if candidato and os.path.exists(candidato):
            ruta_rssi = candidato
            print("RSSI encontrado: %s" % os.path.basename(candidato))

    rssi = None
    if ruta_rssi:
        rssi = pd.read_csv(ruta_rssi)
        rssi["timestamp_host"] = pd.to_datetime(rssi["timestamp_host"])
        rssi["rssi_dbm"] = pd.to_numeric(rssi["rssi_dbm"], errors="coerce")
        rssi = rssi.dropna(subset=["rssi_dbm"])
        if len(rssi) == 0:
            print("El CSV de RSSI no tiene ninguna lectura valida "
                  "(se corrio sin sudo?). Se grafica solo la cobertura.")
            rssi = None
        else:
            rssi["s"] = (rssi["timestamp_host"] - t0).dt.total_seconds()

    if rssi is None:
        fig, ax = plt.subplots(figsize=(11, 5.4), facecolor=SURFACE)
        axes_rssi = None
    else:
        fig, (ax, axes_rssi) = plt.subplots(
            2, 1, figsize=(11, 8.6), facecolor=SURFACE, sharex=True,
            gridspec_kw=dict(height_ratios=[2, 1.4]))
        axes_rssi.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    resumen = []
    for i, mac in enumerate(conteo.columns):
        serie = conteo[mac]
        ax.plot(serie.index, serie.values, lw=2,
                color=SERIES[i % len(SERIES)], label=mac[:8])
        vivos = serie[serie > a.odr * 0.5]
        ultimo = int(vivos.index.max()) if len(vivos) else 0
        malos = (serie <= a.odr * 0.5).astype(int)
        # Un "hueco" es una racha de segundos con poca o ninguna senal. Contarlos
        # distingue un enlace que se corto una vez de uno que parpadea todo el rato.
        huecos = int(((malos == 1) & (malos.shift(1, fill_value=0) == 0)).sum())
        resumen.append({
            "sensor": mac,
            "muestras": int(serie.sum()),
            "seg_con_senal_buena": int((serie > a.odr * 0.5).sum()),
            "seg_sin_senal": int(malos.sum()),
            "huecos": huecos,
            "ultimo_seg_con_senal": ultimo,
        })

    ax.axhline(a.odr, color=INK2, lw=1, ls="--", zorder=0)
    ax.text(0, a.odr * 1.03, f"{a.odr} Hz = sin pérdida", fontsize=9, color=INK2)

    if axes_rssi is None:
        ax.set_xlabel("tiempo desde el inicio (s)", fontsize=10, color=INK2)
    ax.set_ylabel("muestras de giroscopio recibidas por segundo", fontsize=10, color=INK2)
    ax.set_ylim(0, a.odr * 1.18)
    ax.grid(axis="y", color="#e6e5e2", lw=1)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(length=0, colors=INK2, labelsize=9)
    # La leyenda va debajo del ultimo panel: con dos paneles, colgarla del
    # primero la mete justo en el hueco entre los dos y queda tapada.
    ultimo = ax if axes_rssi is None else axes_rssi
    ultimo.legend(ax.get_legend_handles_labels()[0],
                  ax.get_legend_handles_labels()[1],
                  frameon=False, fontsize=9, ncol=6, loc="lower center",
                  bbox_to_anchor=(0.5, -0.30 if axes_rssi is None else -0.42))

    if axes_rssi is not None:
        macs = list(conteo.columns)
        for i, mac in enumerate(macs):
            sub = rssi[rssi["mac"] == mac].sort_values("s")
            if len(sub) == 0:
                continue
            axes_rssi.plot(sub["s"], sub["rssi_dbm"], lw=1.6,
                           color=SERIES[i % len(SERIES)])
        # -85 dBm es donde un enlace BLE empieza a perder paquetes de forma
        # sostenida: por debajo de ahi la caida deja de ser mala suerte.
        axes_rssi.axhline(-85, color=INK2, lw=1, ls="--", zorder=0)
        axes_rssi.text(0, -84, "-85 dBm = zona de riesgo", fontsize=9, color=INK2)
        axes_rssi.set_xlabel("tiempo desde el inicio (s)", fontsize=10, color=INK2)
        axes_rssi.set_ylabel("RSSI (dBm)", fontsize=10, color=INK2)
        axes_rssi.grid(axis="y", color="#e6e5e2", lw=1)
        axes_rssi.set_axisbelow(True)
        for sp in axes_rssi.spines.values():
            sp.set_visible(False)
        axes_rssi.tick_params(length=0, colors=INK2, labelsize=9)

    titulo = a.titulo or "Cobertura del enlace a lo largo de la sesión"
    fig.suptitle(titulo, fontsize=14, color=INK, x=0.02, ha="left", y=1.0)
    fig.text(0.02, 0.945 if axes_rssi is None else 0.962,
             "Cada línea es un sensor. Una caída a cero que después vuelve es una "
             "reconexión automática;\nsi no vuelve, ese enlace se perdió para el "
             "resto de la corrida.",
             fontsize=10, color=INK2, ha="left", va="top", linespacing=1.4)

    fig.savefig(a.out, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)

    r = pd.DataFrame(resumen)
    pd.set_option("display.width", 160)
    print(f"Duración de la sesión: {dur:.1f} s   ODR nominal: {a.odr} Hz")
    print(r.to_string(index=False))
    print(f"\nGuardado: {a.out}")


if __name__ == "__main__":
    sys.exit(main())
