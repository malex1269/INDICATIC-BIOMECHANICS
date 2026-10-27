"""
analizar_bandwidth.py — resume el log acumulado de pruebas de conectividad
(cuantos sensores por adaptador, packed vs raw, distintos ODR).

Sirve tanto para el bandwidth_results.csv viejo (data_collector_v2.py, sin
columnas tag/packed/odr_hz) como para el run_log.csv nuevo (acquire.py).

Uso:
    python3 analizar_bandwidth.py datos_recolectados/bandwidth_results.csv
    python3 analizar_bandwidth.py ../raspberry/data/run_log.csv
"""
import argparse
import sys
import pandas as pd
import matplotlib.pyplot as plt


def load_log(path):
    df = pd.read_csv(path)
    # Los run_log viejos no tienen todas las columnas: cada version del script
    # de adquisicion agrego las suyas. Se rellenan con el valor que estaba
    # fijo en el codigo de esa epoca, para poder mezclar tandas de distintos
    # dias en el mismo analisis sin que pandas rompa.
    for col, default in [("tag", "(sin tag)"), ("packed", False), ("odr_hz", 50),
                         ("conn_interval_ms", 7.5), ("dongle_label", ""),
                         ("adapter_role", ""),
                         # v3 (25-ago): antes estos tres estaban fijos en el codigo
                         ("modo", "stream"), ("supervision_ms", 6000),
                         ("tx_power_dbm", 0), ("reconexion_activa", False),
                         ("nota", ""), ("caidas", 0), ("reconexiones", 0),
                         ("seg_sin_datos", 0.0)]:
        if col not in df.columns:
            df[col] = default
    for col in ("dongle_label", "nota"):
        df[col] = df[col].fillna("")
    for col in ("caidas", "reconexiones", "seg_sin_datos"):
        df[col] = pd.to_numeric(df[col], errors="coerce").fillna(0)
    return df


def main():
    parser = argparse.ArgumentParser(description="Resumen de pruebas de conectividad/bandwidth")
    parser.add_argument("log_csv", help="run_log.csv o bandwidth_results.csv")
    parser.add_argument("--out", default="bandwidth_summary.png", help="archivo de salida del grafico")
    args = parser.parse_args()

    df = load_log(args.log_csv)
    failed = df[df["acc_hz"] == "connect_failed"]
    ok = df[df["acc_hz"] != "connect_failed"].copy()
    ok["acc_hz"] = ok["acc_hz"].astype(float)
    ok["gyro_hz"] = ok["gyro_hz"].astype(float)
    ok["acc_loss_pct"] = ok["acc_loss_pct"].astype(float)
    ok["gyro_loss_pct"] = ok["gyro_loss_pct"].astype(float)

    group_cols = ["tag", "modo", "adapter_mode", "packed", "odr_hz",
                  "conn_interval_ms", "supervision_ms", "tx_power_dbm",
                  "reconexion_activa", "sensors_requested"]
    summary = ok.groupby(group_cols).agg(
        sensores_conectados=("device_mac", "count"),
        acc_hz_medio=("acc_hz", "mean"),
        gyro_hz_medio=("gyro_hz", "mean"),
        acc_perdida_pct_media=("acc_loss_pct", "mean"),
        gyro_perdida_pct_media=("gyro_loss_pct", "mean"),
        caidas=("caidas", "sum"),
        reconexiones=("reconexiones", "sum"),
    ).reset_index()

    fail_counts = failed.groupby(group_cols).size().rename("fallos_conexion")
    summary = summary.merge(fail_counts, on=group_cols, how="left")
    summary["fallos_conexion"] = summary["fallos_conexion"].fillna(0).astype(int)
    summary = summary.sort_values(group_cols)

    # Desglose por adaptador: en modo split la pregunta util no es el promedio
    # de la corrida, sino como se porto cada adaptador por separado (y con que
    # dongle fisico), que es lo que permite comparar dongles entre si.
    if (ok["adapter_role"] != "").any():
        por_adaptador = ok.groupby(["tag", "adapter_role", "dongle_label"]).agg(
            sensores=("device_mac", "count"),
            acc_hz_medio=("acc_hz", "mean"),
            gyro_hz_medio=("gyro_hz", "mean"),
            acc_perdida_pct=("acc_loss_pct", "mean"),
            gyro_perdida_pct=("gyro_loss_pct", "mean"),
            gyro_perdida_max=("gyro_loss_pct", "max"),
            caidas=("caidas", "sum"),
            reconexiones=("reconexiones", "sum"),
            seg_sin_datos=("seg_sin_datos", "mean"),
        ).reset_index()
        pd.set_option("display.width", 160)
        print("=== Por adaptador (integrado vs dongle) ===")
        print(por_adaptador.to_string(index=False))
        por_adaptador.to_csv("resumen_por_adaptador.csv", index=False)
        print("Guardado: resumen_por_adaptador.csv\n")

    pd.set_option("display.width", 160)
    print("=== Por corrida ===")
    print(summary.to_string(index=False))
    summary.to_csv("bandwidth_summary.csv", index=False)
    print("\nGuardado: bandwidth_summary.csv")

    fig, ax = plt.subplots(figsize=(8, 5))
    for (adapter, packed, ci), sub in summary.groupby(
            ["adapter_mode", "packed", "conn_interval_ms"]):
        sub = sub.sort_values("sensors_requested")
        label = f"{adapter} @{ci:g}ms" + (" (packed)" if packed else "")
        ax.plot(sub["sensors_requested"], sub["gyro_perdida_pct_media"], marker="o", label=label)
    ax.set_xlabel("Sensores solicitados")
    ax.set_ylabel("Perdida de gyro promedio (%)")
    ax.set_title("Perdida de datos vs numero de sensores por adaptador")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Guardado: {args.out}")


if __name__ == "__main__":
    sys.exit(main())
