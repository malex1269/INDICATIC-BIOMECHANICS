"""
analizar_sync_imu_imu.py — offset temporal entre dos IMUs a partir de un
evento fisico comun (golpe seco / palmada) visible en ambos como pico de
aceleracion. Primer paso de la Prueba 3 Nivel A (sincronizacion IMU-IMU).

El mismo metodo (deteccion de pico + correlacion cruzada) es la base para
el Nivel B (IMU-OpenCap): cuando se tenga el CSV/export real de OpenCap
con su propio timestamp, se puede adaptar norm_series() para leer esa
fuente en vez de otro IMU. Con el material disponible hoy no se conoce el
formato exacto que exporta OpenCap, asi que ese adaptador no esta escrito
todavia -- hace falta un export de ejemplo para no adivinar columnas.

Reporta dos estimaciones del offset:
  1. Diferencia simple entre el instante del pico maximo en cada sensor.
  2. Correlacion cruzada de las dos senales remuestreadas a una fs comun
     (mas robusta si el evento no es un pico unico y limpio).
Si ambas coinciden, el offset es confiable. Si difieren mucho, el evento
de calibracion probablemente no fue lo bastante claro/unico.

Uso:
    python3 analizar_sync_imu_imu.py sesion.csv --mac_ref AA:BB:CC --mac_target DD:EE:FF
"""
import argparse
import sys
import numpy as np
import pandas as pd
from scipy.signal import correlate, correlation_lags
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
from common import load_session


def norm_series(df, mac):
    sub = df[(df["mac"] == mac) & (df["type"] == "acc")].sort_values("timestamp_host")
    if len(sub) < 2:
        raise ValueError(f"No hay suficientes muestras de acc para {mac}")
    t0 = sub["timestamp_host"].iloc[0]
    t = (sub["timestamp_host"] - t0).dt.total_seconds().to_numpy()
    norm = np.sqrt(sub["x"].to_numpy() ** 2 + sub["y"].to_numpy() ** 2 + sub["z"].to_numpy() ** 2)
    return t, norm


def main():
    parser = argparse.ArgumentParser(description="Offset temporal entre 2 IMUs via evento fisico comun")
    parser.add_argument("csv_path")
    parser.add_argument("--mac_ref", required=True)
    parser.add_argument("--mac_target", required=True)
    parser.add_argument("--fs", type=int, default=50, help="fs para remuestreo comun en la correlacion cruzada")
    parser.add_argument("--out", default="sync_offset.png")
    args = parser.parse_args()

    df = load_session(args.csv_path)
    t_ref, norm_ref = norm_series(df, args.mac_ref)
    t_tgt, norm_tgt = norm_series(df, args.mac_target)

    # Metodo 1: pico simple (norma de aceleracion maxima = el golpe)
    peak_ref_s = t_ref[np.argmax(norm_ref)]
    peak_tgt_s = t_tgt[np.argmax(norm_tgt)]
    offset_peak = peak_tgt_s - peak_ref_s

    # Metodo 2: correlacion cruzada sobre senal remuestreada a fs comun
    t_end = min(t_ref[-1], t_tgt[-1])
    t_common = np.arange(0, t_end, 1.0 / args.fs)
    ref_i = np.interp(t_common, t_ref, norm_ref) - np.mean(norm_ref)
    tgt_i = np.interp(t_common, t_tgt, norm_tgt) - np.mean(norm_tgt)
    corr = correlate(tgt_i, ref_i, mode="full")
    lags = correlation_lags(len(tgt_i), len(ref_i), mode="full")
    offset_xcorr = lags[np.argmax(corr)] / args.fs

    print(f"Sensor referencia:  {args.mac_ref}")
    print(f"Sensor a alinear:   {args.mac_target}")
    print(f"Offset por pico simple:      {offset_peak:+.3f} s")
    print(f"Offset por correlacion cruzada: {offset_xcorr:+.3f} s")
    diff = abs(offset_peak - offset_xcorr)
    if diff > 0.05:
        print(f"ATENCION: las dos estimaciones difieren {diff:.3f} s -- revisar si el "
              f"evento de calibracion (golpe/palmada) fue claro y unico en la ventana analizada.")
    else:
        print("Las dos estimaciones coinciden -- offset confiable.")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=False)
    axes[0].plot(t_ref, norm_ref, label=args.mac_ref)
    axes[0].plot(t_tgt, norm_tgt, label=args.mac_target)
    axes[0].axvline(peak_ref_s, color="C0", linestyle="--")
    axes[0].axvline(peak_tgt_s, color="C1", linestyle="--")
    axes[0].set_title("Norma de aceleracion (sin alinear)")
    axes[0].set_xlabel("Tiempo (s), reloj propio de cada sensor")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_common, ref_i, label=args.mac_ref)
    axes[1].plot(t_common - offset_xcorr, tgt_i, label=f"{args.mac_target} (alineado)")
    axes[1].set_title("Despues de aplicar el offset por correlacion cruzada")
    axes[1].set_xlabel("Tiempo (s)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"\nGuardado: {args.out}")


if __name__ == "__main__":
    sys.exit(main())
