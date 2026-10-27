"""
rutas.py -- donde estan los datos y la biblioteca de Mythos MECH.

En el repositorio los datos viven en `datos/` (al lado de `pc/`), con la
carpeta `OPENCAP/` adentro. La raiz de datos se busca subiendo desde esta
carpeta hasta la primera que contenga `OPENCAP/` o `datos/OPENCAP/`, asi los
scripts corren igual desde el repositorio y desde la carpeta de trabajo
original del proyecto. Se puede fijar a mano:

    set INDICATIC_RAIZ=C:/ruta/a/los/datos          (Windows)
    export INDICATIC_RAIZ=/ruta/a/los/datos         (Linux / macOS)

Mythos MECH: si al lado de los scripts hay una carpeta `mythos_mech/` (la
copia empaquetada, solo `core/` y `kinematics/`) se usa esa; si no, la
carpeta `Mythos MECH/` de la raiz; y `MYTHOS_MECH` en el entorno manda sobre
las dos.
"""
import os

AQUI = os.path.dirname(os.path.abspath(__file__))


def _buscar_raiz():
    env = os.environ.get("INDICATIC_RAIZ")
    if env:
        return os.path.normpath(env)
    d = AQUI
    for _ in range(6):
        padre = os.path.dirname(d)
        if padre == d:
            break
        d = padre
        if os.path.isdir(os.path.join(d, "datos", "OPENCAP")):
            return os.path.join(d, "datos")
        if os.path.isdir(os.path.join(d, "OPENCAP")):
            return d
    return os.path.normpath(os.path.join(AQUI, "..", "datos"))


def _buscar_mythos(raiz):
    env = os.environ.get("MYTHOS_MECH")
    if env:
        return env
    local = os.path.join(AQUI, "mythos_mech")
    if os.path.isdir(os.path.join(local, "kinematics")):
        return local
    return os.path.join(raiz, "Mythos MECH")


# normpath: con rutas tipo `..\..` Windows pasa facil los 260 caracteres y
# deja de ver los archivos.
RAIZ = _buscar_raiz()
DIR_MYTHOS = _buscar_mythos(RAIZ)
# Donde escriben la app y los scripts cuando no se les da --salida.
DIR_SALIDAS = os.path.normpath(os.path.join(AQUI, "..", "resultados"))
