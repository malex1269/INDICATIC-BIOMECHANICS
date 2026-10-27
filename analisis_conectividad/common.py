"""
common.py — carga de sesiones IMU a un formato unico en memoria.

Los datos reales que ya existen en datos_recolectados/ (data4.csv,
test_hci*_*dev_*.csv) estan en el formato "pareado" viejo de
data_collector.py / data_collector_v2.py:
    timestamp, mac, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, gyro_epoch

Los datos nuevos que genera raspberry/acquire.py estan en formato "largo"
(una fila por muestra, con epoch tanto para acc como para gyro):
    timestamp_host, mac, type, x, y, z, epoch_device

load_session() detecta el formato y devuelve siempre el formato largo, para
que los demas scripts no tengan que preocuparse por cual de los dos es.
"""
import pandas as pd


def load_session(csv_path):
    df = pd.read_csv(csv_path)

    if "type" in df.columns:
        # Formato largo (ya nativo).
        out = df.rename(columns={"epoch_device": "epoch"}).copy()
        out["timestamp_host"] = pd.to_datetime(out["timestamp_host"])
        return out[["timestamp_host", "mac", "type", "x", "y", "z", "epoch"]]

    # Formato pareado viejo: una fila trae acc+gyro juntos -> se separa en 2.
    df["timestamp_host"] = pd.to_datetime(df["timestamp"])
    acc = df[["timestamp_host", "mac", "acc_x", "acc_y", "acc_z"]].copy()
    acc.columns = ["timestamp_host", "mac", "x", "y", "z"]
    acc["type"] = "acc"
    acc["epoch"] = pd.NA  # el formato viejo no guarda epoch para acc

    gyro = df[["timestamp_host", "mac", "gyro_x", "gyro_y", "gyro_z"]].copy()
    gyro.columns = ["timestamp_host", "mac", "x", "y", "z"]
    gyro["type"] = "gyro"
    gyro["epoch"] = df["gyro_epoch"] if "gyro_epoch" in df.columns else pd.NA

    out = pd.concat([acc, gyro], ignore_index=True)
    return out.sort_values("timestamp_host").reset_index(drop=True)
