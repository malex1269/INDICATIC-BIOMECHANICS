"""
analizar_nyquist.py — densidad espectral de potencia (Welch) por eje, para
decidir la fs minima necesaria (Prueba 1 del plan: Nyquist).

Calcula, para cada eje x/y/z de un sensor y un canal (acc o gyro):
  - fs efectiva real de la sesion (a partir de los timestamps, no del ODR
    nominal configurado -- ya sabemos que hay perdida de muestras)
  - la frecuencia donde se concentra el 95% y el 99% de la energia
  - la fs recomendada aplicando Nyquist con margen de 5x y 10x sobre f99

Importante: esto solo tiene sentido corriendo la sesion sobre datos que
contengan el evento de interes (frenado/arranque/sacudida), no sobre
movimiento cualquiera -- si se corre sobre una sesion generica el resultado
solo describe esa sesion, no el peor caso del proyecto.

Uso:
    python3 analizar_nyquist.py datos_recolectados/data4.csv --mac ED:9C:3A:4F:BC:CC --type acc
"""
import argparse
import sys
import numpy as np
import pandas as pd
from scipy.signal import welch
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
from common import load_session


def effective_fs(timestamps):
    """
    fs = (n-1) / duracion_total en vez de 1/mediana(dt): con timestamps de
    resolucion de segundo (formato viejo de data_collector.py) muchas
    diferencias consecutivas dan 0 y la mediana se rompe. Con el rango
    total dividido entre el numero de muestras el resultado es robusto
    aunque la resolucion del reloj del host sea gruesa.
    """
    span = (timestamps.iloc[-1] - timestamps.iloc[0]).total_seconds()
    if span <= 0:
        raise ValueError("Rango de tiempo invalido (span <= 0); revisar el CSV de entrada.")
    fs = (len(timestamps) - 1) / span
    if fs < 5:
        print(f"ADVERTENCIA: fs efectiva calculada ({fs:.3f} Hz) es demasiado baja para "
              f"este sensor. El archivo probablemente mezcla varias sesiones separadas en "
              f"el tiempo (ej. data4.csv, que se abre en modo append entre corridas). "
              f"Usa un CSV de una sola sesion continua (ej. los que genera acquire.py, "
              f"uno por corrida) para que este calculo sea valido.")
    return fs


def aviso_si_no_hay_movimiento(freqs, psd, axis):
    """
    Avisa si el espectro parece ruido y no movimiento humano.

    Trampa facil de caer: si se corre esta PSD sobre una sesion donde el
    sensor estaba quieto, el resultado no describe el fenomeno sino el
    ruido del sensor -- y como el ruido es plano, f99 sale altisimo y
    sugiere que hace falta una fs enorme. El movimiento humano concentra
    su energia por debajo de ~10 Hz, asi que si esa banda tiene una
    fraccion muy pequena del total, casi seguro no hay movimiento real.
    """
    banda_baja = psd[freqs <= 10].sum()
    fraccion = banda_baja / psd.sum() if psd.sum() > 0 else 0
    if fraccion < 0.5:
        print(f"  AVISO eje {axis}: solo {fraccion * 100:.0f}% de la energia esta por "
              f"debajo de 10 Hz. Esta sesion parece ruido, no movimiento -- el resultado "
              f"NO sirve para elegir la frecuencia de muestreo. Usa una grabacion que "
              f"contenga el movimiento real (caminata, frenado, sacudida).")
    return fraccion


def energy_frequencies(freqs, psd, thresholds=(0.95, 0.99)):
    cum = np.cumsum(psd) / np.sum(psd)
    out = {}
    for t in thresholds:
        idx = np.searchsorted(cum, t)
        idx = min(idx, len(freqs) - 1)
        out[t] = freqs[idx]
    return out


def main():
    parser = argparse.ArgumentParser(description="PSD / analisis de Nyquist por eje")
    parser.add_argument("csv_path")
    parser.add_argument("--mac", default=None, help="MAC a analizar (default: la primera que aparezca)")
    parser.add_argument("--type", default="acc", choices=["acc", "gyro"])
    parser.add_argument("--out", default="nyquist_psd.png")
    args = parser.parse_args()

    df = load_session(args.csv_path)
    df = df[df["type"] == args.type]
    if args.mac is None:
        args.mac = df["mac"].iloc[0]
        print(f"(no se paso --mac, usando la primera encontrada: {args.mac})")
    df = df[df["mac"] == args.mac].sort_values("timestamp_host")

    if len(df) < 20:
        print(f"Muy pocas muestras ({len(df)}) para {args.mac}/{args.type}. Abortando.")
        return 1

    fs = effective_fs(df["timestamp_host"])
    print(f"Sensor: {args.mac}  canal: {args.type}  muestras: {len(df)}  "
          f"fs efectiva: {fs:.2f} Hz (nominal esperado: verificar contra --odr usado en acquire.py)")

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    rows = []
    for axis, ax_plot in zip(["x", "y", "z"], axes):
        signal = df[axis].to_numpy(dtype=float)
        nperseg = min(256, len(signal))
        freqs, psd = welch(signal, fs=fs, nperseg=nperseg)
        aviso_si_no_hay_movimiento(freqs, psd, axis)
        energies = energy_frequencies(freqs, psd)
        f95, f99 = energies[0.95], energies[0.99]
        rows.append({
            "eje": axis, "fs_efectiva_hz": round(fs, 2),
            "f95_hz": round(f95, 2), "f99_hz": round(f99, 2),
            "fs_recomendada_5x_hz": round(5 * f99, 2),
            "fs_recomendada_10x_hz": round(10 * f99, 2),
            "nyquist_actual_ok": bool(fs / 2 >= f99),
        })
        ax_plot.semilogy(freqs, psd)
        ax_plot.axvline(f99, color="red", linestyle="--", label=f"f99={f99:.1f} Hz")
        ax_plot.axvline(fs / 2, color="green", linestyle=":", label=f"Nyquist actual={fs/2:.1f} Hz")
        ax_plot.set_ylabel(f"PSD {axis}")
        ax_plot.legend(fontsize=8)
        ax_plot.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Frecuencia (Hz)")
    fig.suptitle(f"{args.mac} — {args.type} — PSD (Welch)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)

    summary = pd.DataFrame(rows)
    pd.set_option("display.width", 160)
    print(summary.to_string(index=False))
    print(f"\nGuardado: {args.out}")
    if not summary["nyquist_actual_ok"].all():
        print("ATENCION: al menos un eje tiene f99 por encima de la mitad de la fs actual "
              "-> hay riesgo real de aliasing con la configuracion de hoy.")


if __name__ == "__main__":
    sys.exit(main())
