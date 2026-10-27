"""
analizar_posicion.py — pérdida según el puesto en la cola de conexión.

Es el fenómeno central de la plataforma: la pérdida no sigue al sensor,
sigue a la POSICIÓN en la que se conectó. Este script lo muestra con una
fila por corrida, para que se vea la dispersión y no sólo el promedio —
que es justo donde se escondía el resultado: el promedio de un dongle
sube o baja según le haya tocado una corrida mala en el último puesto.

El orden de conexión es el orden en que aparecen las filas de cada corrida
en el run_log, que es como lo escribe acquire.py.

Uso:
    python3 analizar_posicion.py run_log.csv --tags dongles --out fig.png
    python3 analizar_posicion.py run_log.csv --color dongle_label
"""
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]


def main():
    p = argparse.ArgumentParser(description="Pérdida por puesto en la cola")
    p.add_argument("run_log")
    p.add_argument("--tags", default=None, help="tags a incluir, separados por coma")
    p.add_argument("--color", default="dongle_label",
                   help="columna que define el color. DEFAULT dongle_label")
    p.add_argument("--out", default="fig_posicion.png")
    a = p.parse_args()

    df = pd.read_csv(a.run_log)
    df = df[df["acc_hz"] != "connect_failed"].copy()
    if a.tags:
        df = df[df["tag"].isin([t.strip() for t in a.tags.split(",")])]
    if len(df) == 0:
        print("No hay corridas que graficar.")
        return 1
    df["loss"] = pd.to_numeric(df["gyro_loss_pct"], errors="coerce")
    df["pos"] = df.groupby("run_stamp").cumcount() + 1

    # La etiqueta del dongle sólo está en las filas del dongle: propagarla a
    # toda la corrida para poder colorear también los puntos del integrado.
    if a.color in df.columns:
        clave = (df[df[a.color].notna() & (df[a.color] != "")]
                 .groupby("run_stamp")[a.color].first())
        df["grupo"] = df["run_stamp"].map(clave).fillna("(sin etiqueta)")
    else:
        df["grupo"] = "todas"

    grupos = sorted(df["grupo"].unique())
    fig, ax = plt.subplots(figsize=(9, 5.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    ancho = 0.7 / max(len(grupos), 1)
    for i, g in enumerate(grupos):
        sub = df[df["grupo"] == g]
        desplaz = -0.35 + ancho * (i + 0.5)
        ax.scatter(sub["pos"] + desplaz, sub["loss"], s=42, alpha=0.8,
                   color=SERIES[i % len(SERIES)], label=g,
                   edgecolor=SURFACE, linewidth=0.8, zorder=3)

    # Sin margen arriba, el punto peor queda cortado justo en el borde.
    ax.set_ylim(-df["loss"].max() * 0.04, df["loss"].max() * 1.14)
    ax.set_xticks(range(1, int(df["pos"].max()) + 1))
    ax.set_xlabel("puesto en la cola de conexión", fontsize=10, color=INK2)
    ax.set_ylabel("pérdida de giroscopio (%)", fontsize=10, color=INK2)
    ax.grid(axis="y", color="#e6e5e2", lw=1)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(length=0, colors=INK2, labelsize=9)
    ax.legend(frameon=False, fontsize=9, ncol=len(grupos), loc="upper left")

    fig.suptitle("Toda la pérdida vive en el último puesto de la cola",
                 fontsize=14, color=INK, x=0.02, ha="left", y=1.0)
    fig.text(0.02, 0.935,
             "Un punto por sensor y por corrida. Los puestos 1 a 5 están "
             "pegados al cero con cualquier dongle;\nel 6 salta entre 1.5 % y "
             "66 % sin que el dongle lo explique.",
             fontsize=10, color=INK2, ha="left", va="top", linespacing=1.4)
    fig.savefig(a.out, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)

    pd.set_option("display.width", 200)
    print("=== pérdida por puesto (todas las corridas incluidas) ===")
    print(df.pivot_table(index="pos", columns="grupo", values="loss",
                         aggfunc=["mean", "max", "count"]).round(2).to_string())
    print("\nGuardado: %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
