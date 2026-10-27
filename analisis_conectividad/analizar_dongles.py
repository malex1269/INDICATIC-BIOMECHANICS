"""
analizar_dongles.py — compara los dongles BLE entre si.

La trampa de comparar dongles: cada dongle se prueba en un momento distinto,
y el ambiente de 2.4 GHz cambia solo (WiFi de los vecinos, gente moviendose,
microondas). Comparar la perdida cruda del dongle A del lunes contra la del
dongle B del martes mide tanto el ambiente como el dongle.

Este script evita eso usando el CONTROL INTERNO que trae cada corrida: en
modo split, 3 sensores van al dongle y 3 al Bluetooth integrado de la Pi, al
mismo tiempo, en el mismo ambiente. El integrado es el mismo hardware en
todas las corridas, asi que sirve de referencia. La cifra que compara
dongles de verdad es la DIFERENCIA dentro de cada corrida:

    penalizacion = perdida(dongle) - perdida(integrado)      [en la misma corrida]

Un dongle mejor que otro baja esa penalizacion aunque el ambiente cambie.
Tambien se reporta la perdida absoluta, porque es la que se cita en el
informe, y la estabilidad del enlace (caidas y segundos sin datos), que es
lo que realmente importa con el sujeto en movimiento.

Uso:
    python3 analizar_dongles.py run_log.csv --outdir figuras
    python3 analizar_dongles.py run_log.csv --tags dongles_quieto,dongles_lejos
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
from analizar_bandwidth import load_log

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
# Orden categorico fijo del sistema de diseno (identidad por dongle).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]
GRIS = "#b9b7b2"


# Tandas que llevan etiqueta de dongle pero NO son parte de la comparacion:
# la prueba de humo del arranque y las corridas de diagnostico cuando algo
# no conectaba. Si se cuelan, aparecen como condiciones fantasma en la
# figura y ensucian el promedio de cada dongle.
TAGS_FUERA = {"smoke", "diag", "prueba", "rescate"}


def preparar(path, tags=None):
    df = load_log(path)
    df = df[df["acc_hz"] != "connect_failed"].copy()
    if len(df) == 0:
        return df
    for c in ["acc_hz", "gyro_hz", "acc_loss_pct", "gyro_loss_pct"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    if tags:
        df = df[df["tag"].isin(tags)]
    else:
        fuera = df["tag"].isin(TAGS_FUERA)
        if fuera.any():
            print("Se excluyen %d filas de tandas que no son de la comparacion "
                  "(%s). Para incluirlas, pasalas con --tags."
                  % (int(fuera.sum()),
                     ", ".join(sorted(set(df.loc[fuera, "tag"])))))
            df = df[~fuera]
    # La etiqueta del dongle solo se escribe en las filas del dongle. Para
    # poder agrupar la corrida entera hay que propagarla a las filas del
    # integrado de esa misma corrida.
    etiqueta = (df[df["dongle_label"] != ""]
                .groupby("run_stamp")["dongle_label"].first())
    df["dongle_corrida"] = df["run_stamp"].map(etiqueta).fillna("(sin etiqueta)")
    return df


def tabla_penalizacion(df):
    """Una fila por corrida: perdida del dongle, del integrado, y la diferencia."""
    filas = []
    for (stamp, dongle), g in df.groupby(["run_stamp", "dongle_corrida"]):
        d = g[g["adapter_role"] == "dongle"]
        o = g[g["adapter_role"] == "onboard"]
        if len(d) == 0:
            continue
        fila = {
            "run_stamp": stamp,
            "dongle": dongle,
            "tag": g["tag"].iloc[0],
            "nota": g["nota"].iloc[0],
            "odr_hz": g["odr_hz"].iloc[0],
            "sensores_dongle": len(d),
            "gyro_perdida_dongle": d["gyro_loss_pct"].mean(),
            "gyro_perdida_peor_sensor": d["gyro_loss_pct"].max(),
            "acc_perdida_dongle": d["acc_loss_pct"].mean(),
            "caidas_dongle": d["caidas"].sum(),
            "reconexiones_dongle": d["reconexiones"].sum(),
            "seg_sin_datos_dongle": d["seg_sin_datos"].mean(),
        }
        if len(o):
            fila["gyro_perdida_integrado"] = o["gyro_loss_pct"].mean()
            fila["penalizacion_pts"] = (fila["gyro_perdida_dongle"]
                                        - fila["gyro_perdida_integrado"])
            fila["caidas_integrado"] = o["caidas"].sum()
        else:
            fila["gyro_perdida_integrado"] = np.nan
            fila["penalizacion_pts"] = np.nan
            fila["caidas_integrado"] = np.nan
        filas.append(fila)
    return pd.DataFrame(filas).sort_values(["dongle", "tag", "run_stamp"])


def barras(ax, resumen, dongles, condiciones, columna, titulo, ylabel):
    """Barras agrupadas: una serie por dongle, una posicion por condicion."""
    n = len(dongles)
    ancho = 0.8 / max(n, 1)
    x = np.arange(len(condiciones))
    for i, d in enumerate(dongles):
        sub = resumen[resumen["dongle"] == d].set_index("condicion")
        alturas = [sub[columna].get(c, np.nan) for c in condiciones]
        errores = [sub.get(columna + "_sd", pd.Series(dtype=float)).get(c, 0.0)
                   for c in condiciones]
        pos = x - 0.4 + ancho * (i + 0.5)
        ax.bar(pos, alturas, ancho * 0.9, color=SERIES[i % len(SERIES)], label=d,
               yerr=errores, error_kw=dict(ecolor=INK2, lw=1, capsize=3))
        # La etiqueta va por encima de la barra Y de su barra de error, si no
        # se solapan y no se lee ninguna de las dos.
        for pos_x, h, e in zip(pos, alturas, errores):
            if not np.isnan(h) and h != 0:
                ax.annotate("%.1f" % h, (pos_x, h + (e or 0.0)),
                            textcoords="offset points", xytext=(0, 3),
                            ha="center", fontsize=8, color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels(condiciones, fontsize=9, color=INK2)
    ax.set_ylabel(ylabel, fontsize=10, color=INK2)
    ax.set_title(titulo, fontsize=11, color=INK, loc="left")
    ax.grid(axis="y", color="#e6e5e2", lw=1)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(length=0, colors=INK2, labelsize=9)


def main():
    p = argparse.ArgumentParser(description="Comparacion entre dongles BLE")
    p.add_argument("run_log")
    p.add_argument("--tags", default=None,
                   help="lista de tags a incluir, separados por coma "
                        "(default: todos los que tengan etiqueta de dongle)")
    p.add_argument("--outdir", default=".")
    p.add_argument("--por", default="nota", choices=["nota", "tag"],
                   help="que campo define la 'condicion' en el eje x. DEFAULT nota.")
    a = p.parse_args()

    tags = [t.strip() for t in a.tags.split(",")] if a.tags else None
    df = preparar(a.run_log, tags)
    if len(df) == 0:
        print("No hay corridas utilizables en ese run_log.")
        return 1

    det = tabla_penalizacion(df)
    if len(det) == 0:
        print("Ninguna corrida tiene sensores en el dongle.")
        return 1

    etiquetados = det[det["dongle"] != "(sin etiqueta)"]
    if len(etiquetados) == 0:
        print("Ninguna corrida trae --dongle. Sin eso no se pueden distinguir "
              "los dongles entre si: volve a correr las pruebas pasando "
              "--dongle <etiqueta>.")
        return 1
    det = etiquetados

    os.makedirs(a.outdir, exist_ok=True)
    det["condicion"] = det[a.por].replace("", "(sin nota)")

    pd.set_option("display.width", 200)
    print("=== Corrida por corrida ===")
    cols = ["run_stamp", "dongle", "tag", "condicion", "gyro_perdida_dongle",
            "gyro_perdida_integrado", "penalizacion_pts", "gyro_perdida_peor_sensor",
            "caidas_dongle", "reconexiones_dongle", "seg_sin_datos_dongle"]
    print(det[cols].to_string(index=False, float_format=lambda v: "%.2f" % v))

    resumen = det.groupby(["dongle", "condicion"]).agg(
        corridas=("run_stamp", "nunique"),
        gyro_perdida_dongle=("gyro_perdida_dongle", "mean"),
        gyro_perdida_dongle_sd=("gyro_perdida_dongle", "std"),
        gyro_perdida_integrado=("gyro_perdida_integrado", "mean"),
        penalizacion_pts=("penalizacion_pts", "mean"),
        penalizacion_pts_sd=("penalizacion_pts", "std"),
        peor_sensor=("gyro_perdida_peor_sensor", "max"),
        caidas=("caidas_dongle", "sum"),
        reconexiones=("reconexiones_dongle", "sum"),
        seg_sin_datos=("seg_sin_datos_dongle", "mean"),
    ).reset_index().fillna({"gyro_perdida_dongle_sd": 0.0, "penalizacion_pts_sd": 0.0})

    print("\n=== Resumen por dongle y condicion ===")
    print(resumen.to_string(index=False, float_format=lambda v: "%.2f" % v))
    salida_csv = os.path.join(a.outdir, "resumen_dongles.csv")
    resumen.to_csv(salida_csv, index=False)
    print("\nGuardado: %s" % salida_csv)

    global_ = det.groupby("dongle").agg(
        corridas=("run_stamp", "nunique"),
        penalizacion_pts=("penalizacion_pts", "mean"),
        gyro_perdida=("gyro_perdida_dongle", "mean"),
        caidas=("caidas_dongle", "sum"),
        seg_sin_datos=("seg_sin_datos_dongle", "mean"),
    ).reset_index().sort_values("penalizacion_pts")
    print("\n=== Ranking (menor penalizacion = mejor) ===")
    print(global_.to_string(index=False, float_format=lambda v: "%.2f" % v))

    dongles = list(global_["dongle"])
    condiciones = list(dict.fromkeys(det["condicion"]))

    fig, axes = plt.subplots(2, 1, figsize=(max(8, 2.2 * len(condiciones)), 8.5),
                             facecolor=SURFACE)
    for ax in axes:
        ax.set_facecolor(SURFACE)

    barras(axes[0], resumen, dongles, condiciones, "gyro_perdida_dongle",
           "Pérdida de giroscopio de los 3 sensores del dongle",
           "pérdida media (%)")
    # Referencia: el integrado, que es el mismo hardware en todas las corridas.
    ref = det.groupby("condicion")["gyro_perdida_integrado"].mean()
    for i, c in enumerate(condiciones):
        v = ref.get(c, np.nan)
        if not np.isnan(v):
            axes[0].hlines(v, i - 0.42, i + 0.42, color=INK2, lw=1.4, ls="--", zorder=5)
    axes[0].plot([], [], color=INK2, lw=1.4, ls="--",
                 label="Bluetooth integrado (control)")
    axes[0].legend(frameon=False, fontsize=9, ncol=len(dongles) + 1,
                   loc="upper left", bbox_to_anchor=(0, 1.0))

    barras(axes[1], resumen, dongles, condiciones, "seg_sin_datos",
           "Tiempo sin datos por sensor del dongle (estabilidad)",
           "segundos sin datos (media)")
    # Un panel vacio con el eje autoescalado a +-0.04 s se lee como si hubiera
    # datos diminutos. Si de verdad no se cayo ningun enlace, hay que decirlo.
    if float(resumen["seg_sin_datos"].fillna(0).abs().max()) == 0.0:
        axes[1].set_ylim(0, 1)
        axes[1].text(0.5, 0.5, "Ningún enlace se cayó en estas corridas",
                     transform=axes[1].transAxes, ha="center", va="center",
                     fontsize=11, color=INK2)

    fig.suptitle("Comparación de dongles BLE", fontsize=15, color=INK,
                 x=0.02, ha="left", y=0.995)
    fig.text(0.02, 0.958,
             "Cada corrida trae su propio control: 3 sensores al dongle y 3 al "
             "Bluetooth\nintegrado, al mismo tiempo. La línea punteada es ese control.",
             fontsize=10, color=INK2, ha="left", va="top", linespacing=1.4)
    fig.tight_layout(rect=[0, 0, 1, 0.90], h_pad=3.5)
    salida_png = os.path.join(a.outdir, "fig_dongles.png")
    fig.savefig(salida_png, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    print("Guardado: %s" % salida_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
