"""
log.py — grabar en la memoria de los sensores, con los valores del 25-ago.

Un comando y nada más que recordar:

    python3 log.py                    # 60 s, tag "sesion"
    python3 log.py caminata01         # 60 s
    python3 log.py caminata01 180     # 180 s

Y para rescatar lo que quedó en un sensor que no se pudo descargar, sin
escribir la MAC entera (alcanza con que el trozo sea inconfundible):

    python3 log.py rescate EE         # EE:9D:A0:09:C6:11
    python3 log.py rescate EE F0      # varios de una
    python3 log.py rescate            # los seis

Es un envoltorio de `acquire.py --mode log`, que es el código que produjo
los dos únicos CSV buenos que hay: `cobertura_log_...20260825_135235.csv` y
`...140006.csv`, 0.0 % de pérdida en los cinco sensores que grabaron. No
reimplementa nada: le pasa a acquire.py exactamente los parámetros de esa
corrida y le deja hacer el trabajo.

    python3 acquire.py --outdir ~/datos_v3 --tag cobertura --nota B4_log \\
        --duration 60 --mode log --por-adaptador 2 --espera 10

Con dos diferencias respecto de aquella corrida, las dos pedidas y
medidas: 2+2+2 en vez de 3+3 (el 3+3 dejaba un dongle sin usar y
ponia tres conexiones en el integrado), y 10 s de cuenta regresiva.

El modo log graba y descarga en la MISMA corrida: arranca los sensores, te
podés alejar, y al final vuelve a conectar para bajar los datos. Hay que
estar cerca de la Pi cuando termine la cuenta.

SIN sudo, que es como se corrió el 25. Ver abajo por qué importa.

La caché .metawear
------------------
El SDK guarda un archivo por sensor en `.metawear/` (en la carpeta desde
donde se corre el script). Si esa carpeta quedó de una corrida con sudo,
pertenece a root, y una corrida sin sudo no puede escribirla: el SDK
levanta PermissionError adentro de una callback de C, la inicialización
del sensor nunca termina y **el script se cuelga para siempre en
"Descriptors found"**. Es exactamente lo que pasó el 27-ago.

Este script lo comprueba antes de arrancar y no te deja entrar en ese
estado: o lo arregla, o te dice el comando exacto.
"""
from __future__ import print_function
import os
import subprocess
import sys

# Los sensores, para poder escribir "EE" en vez de la MAC entera.
MACS = [
    "F8:D1:BE:CA:3E:1B",
    "D7:C0:2D:7B:6F:C9",
    "DB:A1:10:94:0E:FD",   # reemplazo de EE:9D:A0:09:C6:11, que no grababa
    "C7:30:05:86:B0:6F",
    "F0:F3:13:F7:E0:B4",
    "FB:34:B5:D8:F3:4E",
]

AQUI = os.path.dirname(os.path.abspath(__file__))
ACQUIRE = os.path.join(AQUI, "acquire.py")
CACHE = os.path.join(AQUI, ".metawear")

# Los valores de la corrida buena del 25-ago. Cambiarlos acá y en ningún
# otro lado: son los que se le pasan a acquire.py en cada corrida.
OUTDIR = "~/datos_v3"
DURACION = 60
NOTA = "log"
# 2 sensores por adaptador = 2+2+2 con los tres enchufados. El 3+3 que traia
# acquire.py dejaba el tercer dongle sin usar y ponia tres conexiones en el
# integrado, que es donde se cae el tercero de la cola.
POR_ADAPTADOR = 2
# Cuenta regresiva antes de arrancar a grabar, para ubicarse.
ESPERA = 10


def cache_utilizable():
    """
    True si se puede escribir en .metawear, o si no existe todavía.

    Si existe y no se puede escribir, se intenta borrarla (funciona si esta
    corrida tiene permisos suficientes). Si tampoco se puede, se devuelve
    False y el que llama imprime qué hacer: colgarse en "Descriptors found"
    sin ninguna explicación es mucho peor que no arrancar.
    """
    if not os.path.isdir(CACHE):
        return True
    if os.access(CACHE, os.W_OK | os.X_OK):
        return True
    print("La caché del SDK no es escribible: %s" % CACHE)
    print("Seguramente quedó de una corrida con sudo.")
    try:
        import shutil
        shutil.rmtree(CACHE)
        print("Borrada. Se regenera sola.")
        return True
    except OSError as e:
        print("No se pudo borrar (%s)." % e)
        return False


def resolver(trozo):
    """
    La MAC completa a partir de un trozo. Devuelve None si no es
    inconfundible, en vez de adivinar: una MAC mal resuelta descarga el
    sensor equivocado y no hay forma de darse cuenta después.
    """
    t = trozo.upper().replace("-", ":")
    iguales = [m for m in MACS if m == t]
    if iguales:
        return iguales[0]
    empiezan = [m for m in MACS if m.startswith(t)]
    if len(empiezan) == 1:
        return empiezan[0]
    if not empiezan:
        print("No hay ningún sensor que empiece con '%s'." % trozo)
    else:
        print("'%s' coincide con varios: %s" % (trozo, ", ".join(empiezan)))
    return None


def rescatar(trozos):
    """Baja lo que haya quedado en la flash de esos sensores."""
    if trozos:
        macs = []
        for t in trozos:
            m = resolver(t)
            if m is None:
                print("Sensores: %s" % ", ".join(MACS))
                return 1
            macs.append(m)
    else:
        macs = list(MACS)

    cmd = [sys.executable, ACQUIRE,
           "--outdir", OUTDIR,
           "--tag", "rescate",
           "--macs", ",".join(macs),
           "--mode", "download"]
    print("rescatando: %s" % ", ".join(macs))
    print(" ".join(cmd))
    print("")
    codigo = subprocess.call(cmd, cwd=AQUI)
    if codigo == 0:
        print("")
        print("Para meterlo en el CSV de la sesión:")
        print("  python3 fusionar.py <sesion>.csv <rescate>.csv")
    return codigo


def main():
    args = [a for a in sys.argv[1:]]
    if args and args[0].lower() in ("rescate", "rescatar"):
        if not os.path.isfile(ACQUIRE):
            print("Falta acquire.py al lado de este script: %s" % ACQUIRE)
            return 1
        if not cache_utilizable():
            print("")
            print("Corré esto y volvé a intentar:")
            print("  sudo rm -rf %s" % CACHE)
            return 1
        return rescatar(args[1:])
    tag = args[0] if args else "sesion"
    duracion = DURACION
    if len(args) > 1:
        try:
            duracion = int(args[1])
        except ValueError:
            print("La duración va en segundos: python3 log.py %s 180" % tag)
            return 1

    if not os.path.isfile(ACQUIRE):
        print("Falta acquire.py, tiene que estar al lado de este script:")
        print("  %s" % ACQUIRE)
        return 1

    if not cache_utilizable():
        print("")
        print("Corré esto y volvé a intentar:")
        print("  sudo rm -rf %s" % CACHE)
        return 1

    if os.geteuid() == 0:
        print("AVISO: estás corriendo con sudo.")
        print("El 25-ago esto se corrió SIN sudo, y el modo log no lo")
        print("necesita. Con sudo la caché queda de root y la próxima")
        print("corrida sin sudo se cuelga en 'Descriptors found'.")
        print("")

    cmd = [sys.executable, ACQUIRE,
           "--outdir", OUTDIR,
           "--tag", tag,
           "--nota", NOTA,
           "--duration", str(duracion),
           "--por-adaptador", str(POR_ADAPTADOR),
           "--espera", str(ESPERA),
           "--mode", "log"]
    print("%s  (%d s)" % (tag, duracion))
    print(" ".join(cmd))
    print("")
    return subprocess.call(cmd, cwd=AQUI)


if __name__ == "__main__":
    sys.exit(main())
