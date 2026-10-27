"""
fusionar.py — mete el CSV de un rescate dentro del CSV de la sesión.

El problema que resuelve: si un sensor no se pudo descargar al final de una
corrida, sus datos siguen en su flash y se rescatan con

    python3 acquire.py --outdir ~/datos_v3 --tag rescate --mode download \\
        --macs EE:9D:A0:09:C6:11

...pero eso escribe un **archivo nuevo**, no completa el hueco en el CSV de
la sesión. No hay forma de que lo complete: cuando el rescate corre, el CSV
de la sesión ya está cerrado.

Este script los junta:

    python3 fusionar.py sesion.csv rescate.csv

Escribe `sesion_completo.csv` al lado, con las filas de los dos ordenadas
por `epoch_device`. No toca ninguno de los dos originales.

Por qué se puede juntar sin más: cada fila trae su `mac` y su
`epoch_device`, que es el reloj del propio sensor. No hay ningún índice
compartido que respetar, así que las filas del rescate son válidas por sí
solas -- el orden en el archivo es una comodidad, no un dato.

Ojo con una cosa: `epoch_device` de dos sensores distintos NO es el mismo
instante. Cada uno corre a su ritmo (medido: 98.4 a 101.3 Hz con el mismo
ODR). Ordenar por epoch mezcla bien las filas para leer el archivo, pero
alinear los sensores entre sí sigue siendo trabajo del análisis.
"""
from __future__ import print_function
import csv
import os
import sys

COLUMNAS = ["timestamp_host", "mac", "type", "x", "y", "z", "epoch_device"]


def leer(ruta):
    """Filas del CSV, y las MAC que aparecen. Avisa si le faltan columnas."""
    with open(ruta, newline="") as f:
        r = csv.DictReader(f)
        faltan = [c for c in COLUMNAS if c not in (r.fieldnames or [])]
        if faltan:
            print("XX  %s no tiene las columnas %s" % (ruta, ", ".join(faltan)))
            print("    ¿Es un CSV de datos, y no el run_log?")
            return None, None
        filas = list(r)
    return filas, sorted({f["mac"] for f in filas})


def clave(fila):
    """Orden por epoch del sensor; una fila sin epoch va al final, no rompe."""
    try:
        return (0, int(fila["epoch_device"]))
    except (ValueError, KeyError, TypeError):
        return (1, 0)


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip())
        return 1
    sesion, rescate = sys.argv[1], sys.argv[2]
    for ruta in (sesion, rescate):
        if not os.path.isfile(ruta):
            print("No existe: %s" % ruta)
            return 1

    filas_s, macs_s = leer(sesion)
    if filas_s is None:
        return 1
    filas_r, macs_r = leer(rescate)
    if filas_r is None:
        return 1

    print("sesión:  %6d filas, %d sensores: %s"
          % (len(filas_s), len(macs_s), ", ".join(macs_s)))
    print("rescate: %6d filas, %d sensores: %s"
          % (len(filas_r), len(macs_r), ", ".join(macs_r)))

    repetidos = sorted(set(macs_s) & set(macs_r))
    if repetidos:
        print("")
        print("XX  %s ya está en la sesión." % ", ".join(repetidos))
        print("    Fusionar duplicaría sus muestras. Si el rescate es el bueno,")
        print("    sacá ese sensor del CSV de la sesión primero.")
        return 1

    salida = os.path.splitext(sesion)[0] + "_completo.csv"
    todas = sorted(filas_s + filas_r, key=clave)
    with open(salida, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNAS, extrasaction="ignore")
        w.writeheader()
        w.writerows(todas)

    print("")
    print("%d filas, %d sensores -> %s"
          % (len(todas), len(set(macs_s) | set(macs_r)), salida))
    print("Los dos archivos originales quedan como estaban.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
