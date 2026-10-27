"""
analizar_drift.py — separa drift real de reloj vs. perdida de datos
disfrazada de drift (Prueba 5 + Hipotesis 3.3 del plan), y estima drift de
orientacion integrando el giroscopio.

Idea clave: el epoch que manda cada IMU es un reloj del propio dispositivo,
asi que su tasa de avance NO depende de cuantos paquetes BLE se perdieron
en el camino -- solo depende de las muestras que SI llegaron. Por eso
comparar "duracion segun epoch" contra "duracion real del host" aisla el
drift de reloj real, mientras que comparar "muestras recibidas" contra las
esperadas aisla la perdida de datos. Si el drift de reloj medido es
pequeno (decenas de ppm) pero la perdida de muestras varia mucho entre
sensores, confirma que lo que antes se llamaba "drift" era sobre todo
perdida de paquetes.

Uso (sesion de reposo rigido, todos los sensores juntos e inmoviles):
    python3 analizar_drift.py datos_recolectados/mi_sesion_reposo.csv --odr 50
"""
import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
from common import load_session


def main():
    parser = argparse.ArgumentParser(description="Drift de reloj vs perdida de datos, y drift de orientacion")
    parser.add_argument("csv_path")
    parser.add_argument("--odr", type=int, default=50, help="ODR nominal configurado en la sesion")
    parser.add_argument("--out", default="drift_orientacion.png")
    args = parser.parse_args()

    df = load_session(args.csv_path)
    host_span = (df["timestamp_host"].max() - df["timestamp_host"].min()).total_seconds()
    if host_span <= 0:
        print("Rango de tiempo invalido en el CSV.")
        return 1
    expected_samples = args.odr * host_span

    rows = []
    for mac, g in df.groupby("mac"):
        for kind in ["acc", "gyro"]:
            sub = g[g["type"] == kind]
            n = len(sub)
            if n == 0:
                continue
            loss_pct = max(0.0, 1 - n / expected_samples) * 100

            epoch_drift_ppm = None
            valid_epoch = sub["epoch"].dropna()
            if len(valid_epoch) >= 2:
                epoch_span_s = (valid_epoch.max() - valid_epoch.min()) / 1000.0
                epoch_drift_ppm = (epoch_span_s - host_span) / host_span * 1e6

            rows.append({
                "mac": mac, "canal": kind, "muestras": n,
                "perdida_pct": round(loss_pct, 2),
                "drift_reloj_ppm": round(epoch_drift_ppm, 1) if epoch_drift_ppm is not None else "sin epoch",
            })

    summary = pd.DataFrame(rows).sort_values(["canal", "mac"])
    pd.set_option("display.width", 160)
    print(f"Duracion de la sesion (host): {host_span:.1f} s   ODR nominal: {args.odr} Hz")
    print(summary.to_string(index=False))

    has_epoch = summary["drift_reloj_ppm"].apply(lambda v: v != "sin epoch")
    if has_epoch.any():
        loss_spread = summary["perdida_pct"].max() - summary["perdida_pct"].min()
        drift_vals = summary.loc[has_epoch, "drift_reloj_ppm"].astype(float)
        drift_spread = drift_vals.max() - drift_vals.min()
        print(f"\nDispersion entre sensores: perdida_pct spread={loss_spread:.2f} pts, "
              f"drift_reloj_ppm spread={drift_spread:.1f} ppm")
        if drift_spread < 100 and loss_spread > 2:
            print("-> El drift de reloj real es pequeno y parecido entre sensores, pero la "
                  "perdida de muestras varia bastante: consistente con la Hipotesis 3.3 "
                  "(el 'drift' reportado antes era sobre todo perdida de paquetes).")
    else:
        print("\nNinguna fila tiene epoch valido -- no se puede separar drift de reloj de "
              "perdida de datos con este archivo (formato viejo sin epoch para acc, o "
              "sesion muy corta).")

    # ---- Drift de orientacion ----------------------------------------
    # Se integra CADA EJE CON SU SIGNO, no la norma |gyro|. Integrar la
    # norma siempre suma positivo, asi que acumula tambien el ruido y
    # sobreestima el drift real (daba ~2x de mas en la sesion del 21-ago).
    # Con signo, el ruido de media cero se cancela y lo que queda es el
    # sesgo del giroscopio, que es el drift de verdad -- y que ademas se
    # puede calibrar restandolo.
    fig, ax = plt.subplots(figsize=(8, 5))
    orient_rows = []
    for mac, g in df[df["type"] == "gyro"].groupby("mac"):
        g = g.sort_values("timestamp_host")
        t = (g["timestamp_host"] - g["timestamp_host"].iloc[0]).dt.total_seconds().to_numpy()
        if len(t) < 2:
            continue
        duration_min = t[-1] / 60.0
        row = {"mac": mac}
        peor = 0.0
        for axis in ["x", "y", "z"]:
            v = g[axis].to_numpy(dtype=float)
            angle = np.concatenate([[0], np.cumsum(np.diff(t) * (v[:-1] + v[1:]) / 2)])
            deg_min = angle[-1] / duration_min if duration_min > 0 else float("nan")
            row[f"sesgo_{axis}_deg_s"] = round(float(np.mean(v)), 3)
            row[f"drift_{axis}_deg_min"] = round(deg_min, 1)
            peor = max(peor, abs(deg_min))
            if axis == "z":
                ax.plot(t, angle, label=f"{mac[:8]} z")
        row["peor_eje_deg_min"] = round(peor, 1)
        orient_rows.append(row)

    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Angulo integrado, eje z (deg)")
    ax.set_title("Drift de orientacion en reposo (integracion con signo, eje z)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)

    if orient_rows:
        print("\nDrift de orientacion (solo valido si la sesion es en reposo rigido).")
        print("El 'sesgo' es la lectura media del giroscopio quieto: deberia ser 0.")
        print("Como es aproximadamente constante, se puede restar en el "
              "postprocesado y el drift baja muchisimo.")
        od = pd.DataFrame(orient_rows).sort_values("peor_eje_deg_min")
        print(od.to_string(index=False))
    print(f"\nGuardado: {args.out}")


if __name__ == "__main__":
    sys.exit(main())
