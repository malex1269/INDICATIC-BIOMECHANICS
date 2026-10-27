"""
graficas_resultados.py — figuras de resultados para el informe.

Genera dos figuras, cada una con UNA idea:

  fig1_posicion_vs_sensor.png
      Mapa de calor de perdida de giroscopio, sensor x corrida, separado en
      el bloque con el orden original y el bloque con el orden rotado.
      Responde: la perdida, ¿sigue al sensor o a la posicion en la cola?

  fig2_packed.png
      Porcentaje de muestras recibidas por sensor en cada configuracion de
      ODR, con y sin packed mode. Responde: ¿hasta que frecuencia se puede
      subir, y cuanto ayuda el empaquetado?

Color: rampa secuencial de un solo tono (mas oscuro = mas perdida), que es
lo correcto para magnitud. No se usan colores categoricos por sensor: el
sensor no es la identidad que importa aqui, lo es la magnitud de perdida.

Uso:
    python3 graficas_resultados.py ruta/al/run_log.csv --outdir ./figuras
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Rectangle

# Rampa secuencial azul (claro -> oscuro), del sistema de diseno.
AZUL = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
CMAP = LinearSegmentedColormap.from_list("azul", AZUL)

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
NARANJA = "#eb6834"   # slot categorico 2, para "con packed"
AZUL_CAT = "#2a78d6"  # slot categorico 1, para "sin packed"


def preparar(path):
    df = pd.read_csv(path)
    df = df[df["acc_hz"] != "connect_failed"].copy()
    for c in ["gyro_hz", "acc_hz", "gyro_loss_pct", "acc_loss_pct"]:
        df[c] = df[c].astype(float)
    return df


def orden_de_conexion(g):
    """El orden de las filas dentro de una corrida ES el orden de conexion."""
    return list(g["device_mac"])


# Tandas del 21-ago que alimentan cada figura. Son los defaults; se cambian
# desde la linea de comandos para analizar una sesion nueva sin tocar el codigo.
TAGS_POSICION = [("base", "Orden original"), ("orden", "Orden rotado")]
# En orden de preferencia: odr2 (21-ago tarde) incluye 400 Hz y se corrio con
# el codigo ya corregido; odr_test es la tanda anterior, contaminada por el
# efecto de posicion del ultimo sensor.
TAGS_PACKED = ["odr2", "odr_test"]


def fig_posicion(df, outdir, tags=None):
    bloques = []
    for tag, titulo in (tags or TAGS_POSICION):
        sub = df[df["tag"] == tag]
        if len(sub) == 0:
            continue
        piv = sub.pivot_table(index="run_stamp", columns="device_mac",
                              values="gyro_loss_pct")
        # Reordenar columnas segun el orden real de conexion de ese bloque.
        primera = sub[sub["run_stamp"] == sub["run_stamp"].iloc[0]]
        cols = [m for m in orden_de_conexion(primera) if m in piv.columns]
        piv = piv[cols]
        bloques.append((titulo, piv, cols[-1]))

    if not bloques:
        return None

    vmax = max(b[1].to_numpy().max() for b in bloques)
    anchos = [b[1].shape[0] for b in bloques]
    fig, axes = plt.subplots(
        1, len(bloques), figsize=(14, 5.4), facecolor=SURFACE,
        gridspec_kw={"width_ratios": anchos, "wspace": 0.55})
    fig.subplots_adjust(top=0.74)
    if len(bloques) == 1:
        axes = [axes]

    for ax, (titulo, piv, ultimo) in zip(axes, bloques):
        datos = piv.to_numpy().T  # sensores en filas
        im = ax.imshow(datos, cmap=CMAP, vmin=0, vmax=vmax, aspect="auto")
        ax.set_facecolor(SURFACE)

        etiquetas = []
        for m in piv.columns:
            etiquetas.append(f"{m[:8]}  ◀ ULTIMO" if m == ultimo else m[:8])
        ax.set_yticks(range(len(piv.columns)))
        ax.set_yticklabels(etiquetas, fontsize=9, color=INK)
        ax.set_xticks(range(piv.shape[0]))
        ax.set_xticklabels([f"C{i + 1}" for i in range(piv.shape[0])],
                           fontsize=9, color=INK2)
        ax.set_xlabel("corrida", fontsize=9, color=INK2)

        # Marcar la fila del ultimo sensor de la cola.
        fila = list(piv.columns).index(ultimo)
        ax.add_patch(Rectangle((-0.5, fila - 0.5), piv.shape[0], 1,
                               fill=False, edgecolor=NARANJA, lw=2.5, zorder=5))

        # Valor en cada celda; texto claro sobre celdas oscuras.
        for i in range(datos.shape[0]):
            for j in range(datos.shape[1]):
                v = datos[i, j]
                ax.text(j, i, f"{v:.0f}" if v >= 1 else "0",
                        ha="center", va="center", fontsize=8.5,
                        color="#ffffff" if v > vmax * 0.5 else INK)

        ax.set_title(titulo, fontsize=11, color=INK, pad=10, loc="left")
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(length=0)

    cb = fig.colorbar(im, ax=axes, fraction=0.02, pad=0.02)
    cb.set_label("pérdida de giroscopio (%)", fontsize=9, color=INK2)
    cb.ax.tick_params(labelsize=8, length=0, colors=INK2)
    cb.outline.set_visible(False)

    fig.suptitle("La pérdida sigue a la POSICIÓN en la cola, no al sensor",
                 fontsize=14, color=INK, x=0.055, ha="left", y=0.98)
    fig.text(0.055, 0.885,
             "Recuadro naranja = último sensor en conectarse (3.º del dongle). "
             "Al rotar el orden, el problema se mudó con la posición.",
             fontsize=10, color=INK2, ha="left")

    ruta = os.path.join(outdir, "fig1_posicion_vs_sensor.png")
    fig.savefig(ruta, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    return ruta


def fig_packed(df, outdir, tags=None):
    o = pd.DataFrame()
    for tag in (tags or TAGS_PACKED):
        o = df[df["tag"] == tag].copy()
        if len(o):
            break
    if len(o) == 0:
        return None
    o["recibido_pct"] = 100 - o["gyro_loss_pct"]
    o["config"] = o.apply(
        lambda r: f"{int(r['odr_hz'])} Hz\n{'con packed' if r['packed'] else 'sin packed'}",
        axis=1)

    orden = sorted(o["config"].unique(),
                   key=lambda c: (int(c.split(" ")[0]), "con" in c))
    fig, ax = plt.subplots(figsize=(10, 5.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    ancho = 0.13
    sensores = list(dict.fromkeys(o["device_mac"]))
    for k, mac in enumerate(sensores):
        vals = []
        for c in orden:
            f = o[(o["config"] == c) & (o["device_mac"] == mac)]["recibido_pct"]
            vals.append(f.iloc[0] if len(f) else np.nan)
        x = np.arange(len(orden)) + (k - len(sensores) / 2 + 0.5) * ancho
        colores = [NARANJA if "con packed" in c else AZUL_CAT for c in orden]
        ax.bar(x, vals, ancho * 0.88, color=colores, edgecolor=SURFACE, linewidth=1.2)

    ax.axhline(100, color=INK2, lw=1, ls="--", zorder=0)
    ax.text(-0.45, 102.5, "100% = sin pérdida", fontsize=9, color=INK2)

    ax.set_xticks(range(len(orden)))
    ax.set_xticklabels(orden, fontsize=10, color=INK)
    ax.set_ylabel("muestras de giroscopio recibidas (%)", fontsize=10, color=INK2)
    ax.set_ylim(0, 112)
    ax.grid(axis="y", color="#e6e5e2", lw=1)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(length=0, colors=INK2, labelsize=9)

    fig.suptitle("El techo del sistema: 100 Hz con packed",
                 fontsize=14, color=INK, x=0.02, ha="left", y=1.02)
    fig.text(0.02, 0.945,
             "Cada barra es uno de los 6 sensores. Con packed a 100 Hz llegan los 6 "
             "completos. A 200 Hz aguanta el Bluetooth integrado pero el dongle\n"
             "se satura; a 400 Hz falla todo. Sin packed (azul), 100 Hz ya es inviable.",
             fontsize=10, color=INK2, ha="left", va="top")

    ruta = os.path.join(outdir, "fig2_packed.png")
    fig.savefig(ruta, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    return ruta


def main():
    p = argparse.ArgumentParser(description="Figuras de resultados")
    p.add_argument("run_log")
    p.add_argument("--outdir", default=".")
    p.add_argument("--tags-posicion", default=None,
                   help="tandas para la figura de posicion en la cola, en formato "
                        "tag=titulo separadas por coma. DEFAULT: %s"
                        % ",".join("%s=%s" % t for t in TAGS_POSICION))
    p.add_argument("--tags-packed", default=None,
                   help="tandas para la figura de packed/ODR, separadas por coma y "
                        "en orden de preferencia (se usa la primera que exista). "
                        "DEFAULT: %s" % ",".join(TAGS_PACKED))
    a = p.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    posicion = None
    if a.tags_posicion:
        posicion = []
        for par in a.tags_posicion.split(","):
            tag, _, titulo = par.partition("=")
            posicion.append((tag.strip(), titulo.strip() or tag.strip()))
    packed = ([t.strip() for t in a.tags_packed.split(",")]
              if a.tags_packed else None)

    df = preparar(a.run_log)
    hechas = 0
    for r in [fig_posicion(df, a.outdir, posicion), fig_packed(df, a.outdir, packed)]:
        if r:
            print("Guardado:", r)
            hechas += 1
    if hechas == 0:
        # Antes esto salia en silencio con codigo 0 y parecia que habia
        # funcionado: las tandas estaban fijas en el codigo y un run_log de
        # otra sesion no tiene esos tags.
        print("No se genero ninguna figura: el run_log no tiene las tandas "
              "esperadas.")
        print("Tags disponibles: %s" % ", ".join(sorted(df["tag"].unique())))
        print("Indica cuales usar con --tags-posicion y --tags-packed "
              "(ver --help).")
        return 1


if __name__ == "__main__":
    sys.exit(main())
