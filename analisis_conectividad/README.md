# analisis_conectividad/ — análisis de las pruebas de la plataforma

Scripts que corren en cualquier PC sobre los CSV y bitácoras que produce
`../raspberry/acquire.py`. Los resultados y su lectura están en
`../docs/1_caracterizacion_plataforma.md`.

```bash
pip install -r requirements.txt
```

| Script | Para qué | Ejemplo |
|---|---|---|
| `analizar_bandwidth.py` | Resumen por configuración: Hz efectivo, % de pérdida por canal, caídas, fallos de conexión. | `python analizar_bandwidth.py ../datos/caracterizacion/run_log_20-21ago.csv` |
| `graficas_resultados.py` | Las figuras de la caracterización: pérdida por corrida, efecto de la posición en la cola, techo de frecuencia con/sin packed. | `python graficas_resultados.py ../datos/caracterizacion/run_log_20-21ago.csv --outdir salida` |
| `analizar_dongles.py` | Compara dongles usando el Bluetooth integrado de cada corrida como control del ambiente de 2.4 GHz. | `python analizar_dongles.py run_log.csv --outdir salida` |
| `analizar_cobertura.py` | Cuándo se cae cada enlace y si se recupera, segundo a segundo (con `--rssi`, la potencia recibida). | `python analizar_cobertura.py sesion.csv --odr 100` |
| `analizar_drift.py` | Separa la deriva real de reloj de la pérdida de datos, y mide el sesgo del giroscopio en reposo. | `python analizar_drift.py reposo.csv --odr 100` |
| `analizar_nyquist.py` | Espectro (Welch) por eje y frecuencia de muestreo recomendada. Solo vale sobre datos con el movimiento real. | `python analizar_nyquist.py sesion.csv --type acc` |
| `analizar_sync_imu_imu.py` | Desfase temporal entre dos sensores a partir de un golpe común. | `python analizar_sync_imu_imu.py sesion.csv --mac_ref AA:.. --mac_target BB:..` |
| `analizar_posicion.py` | Pérdida según la posición en la cola de conexión. | — |
| `common.py` | Lectura de los dos formatos de CSV (el de `acquire.py` y el del script original). | — |

Los CSV crudos de cada corrida (≈ 0.5 GB) no están en el repositorio; las
bitácoras `run_log` sí, en `../datos/caracterizacion/`, y alcanzan para
`analizar_bandwidth.py` y `graficas_resultados.py`.
