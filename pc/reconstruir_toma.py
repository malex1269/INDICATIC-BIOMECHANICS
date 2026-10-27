"""
reconstruir_toma.py -- de un CSV de IMU a un .mot listo para OpenSim.

    python reconstruir_toma.py testnorma2l9112026 --sin-excluir
    python reconstruir_toma.py testnorma2l9112026 --salida CARPETA
    python reconstruir_toma.py mi_toma.csv --variedad variedad.npz

Que hace, en una linea: reconstruye la cinematica articular completa a partir
de las DOS cantidades que el montaje S mide sin deriva (la cadena sagital
pie-sacro y la inclinacion de la pelvis), apoyandose en una "variedad de
marcha" aprendida del propio sujeto con OpenCap. Ver `variedad.py` para el
metodo y docs/4_integracion_opensim.md para el resumen.

QUE ES MEDIDO Y QUE ES GENERADO -- la separacion es el punto del metodo:

  MEDIDO (sale del IMU, no deriva)
    pelvis_tilt, pelvis_list        inclinacion del sacro vs gravedad
    la suma hip - knee + ankle      inclinacion del pie menos la del sacro
    cadencia, duracion de cada ciclo, fraccion de apoyo, asimetria I/D
  REPARTIDO (medido en total, el prior decide como se divide)
    hip_flexion, knee_angle         se reparten la cadena medida
  GENERADO POR EL PRIOR (no se mide: se toma de la marcha del sujeto)
    ankle_angle                     medido: el ajuste lo empeora, ver §4bis
    hip_adduction/rotation, subtalar, mtp, lumbar, brazos
    pelvis_rotation, pelvis_ty, pelvis_tz
  SINTETIZADO
    pelvis_tx                       progresion, del largo de zancada segun
                                    la cadencia medida

Por defecto la variedad EXCLUYE el trial de OpenCap emparejado con la toma
que se reconstruye (leave-one-take-out): si no, se estaria reconstruyendo con
datos que ya vieron la respuesta. Se apaga con --sin-excluir.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sparse6 as s6
import variedad as V
import mot_io
from mapa_sensores import mapa_de

from rutas import RAIZ, DIR_SALIDAS   # RAIZ = la carpeta de datos (con OPENCAP/)
DIR_OC = os.path.join(RAIZ, "OPENCAP")

# ---------------------------------------------------------------- sesiones
# Una entrada por sesion grabada con los sensores y OpenCap a la vez:
#   clave     subcadena que identifica la sesion en el tag de la toma (la
#             fecha tal como se escribio al grabar: "9112026" = 11/09/2026)
#   carpeta   la carpeta interna del export de OpenCap (MarkerData/,
#             OpenSimData/Kinematics/, OpenSimData/Model/)
#   trials    los trials de OpenCap con marcha normal, para la variedad
# Los CSV de los sensores de cada sesion van en
# datos/OPENCAP/<sesion>/Datos imu de esta sesion/ y se encuentran solos.
# Para agregar una sesion: una entrada aca, sus pares en PARES y su mapa de
# sensores en mapa_sensores.py (ver docs/MANUAL_PIPELINE.md, seccion 6).
SESIONES = [
    ("9112026", os.path.join(DIR_OC, "sesion_2026-09-11", "OpenCapData"),
     ["TEST1_1", "TEST2"]),
]

# Toma de los sensores -> trial de OpenCap de la misma pasada. Verificado
# con sincronizar.py: test1normal con TEST1_1 (r = 0.67, 7 golpes de talon,
# 27 ms) y testnorma2l con TEST2 (r = 0.47, 10 golpes, 26 ms). TEST1 fue la
# pasada fallida y no tiene toma de los sensores.
PARES = {"test1normal9112026": "TEST1_1",
         "testnorma2l9112026": "TEST2"}


def dir_oc_de(tag):
    """(carpeta de OpenCap, trials de la variedad) de la sesion del tag."""
    for clave, carpeta, trials in SESIONES:
        if clave in tag:
            return carpeta, trials
    raise SystemExit("la toma '%s' no es de ninguna sesion registrada en "
                     "reconstruir_toma.SESIONES" % tag)


def rutas_trial(carpeta, trial):
    """(trc, mot) de un trial, con la estructura del export de OpenCap."""
    return (os.path.join(carpeta, "MarkerData", trial + ".trc"),
            os.path.join(carpeta, "OpenSimData", "Kinematics", trial + ".mot"))


def osim_de(tag):
    """El .osim escalado de la sesion (para recortar al rango del modelo)."""
    carpeta, _t = dir_oc_de(tag)
    return os.path.join(carpeta, "OpenSimData", "Model", "LaiUhlrich2022_scaled.osim")


def tomas_con_fecha():
    """Las tomas de PARES que pertenecen a una sesion registrada."""
    return [t for t in PARES if any(s[0] in t for s in SESIONES)]


PIES = [("pie_der", "r"), ("pie_izq", "l")]


def carpetas_imu_en_opencap():
    """Toda carpeta `OPENCAP/*/Datos imu de esta sesion/`."""
    out = []
    if os.path.isdir(DIR_OC):
        for s in sorted(os.listdir(DIR_OC)):
            d = os.path.join(DIR_OC, s, "Datos imu de esta sesion")
            if os.path.isdir(d):
                out.append(d)
    return out


DIRS_IMU = carpetas_imu_en_opencap()


def buscar_csv(nombre):
    """Acepta un tag (`testnorma2l9112026`) o una ruta a un CSV."""
    if os.path.isfile(nombre):
        return nombre
    for carpeta in DIRS_IMU:
        if not os.path.isdir(carpeta):
            continue
        cand = [os.path.join(carpeta, f) for f in sorted(os.listdir(carpeta))
                if f.startswith(nombre) and f.endswith(".csv")
                and "run_log" not in f
                and os.path.getsize(os.path.join(carpeta, f)) > 100000]
        if cand:
            return cand[0]
    raise SystemExit("no encuentro la toma '%s'" % nombre)


def preparar_imu(csv):
    """Orientacion e inclinacion (pitch/roll vs gravedad) de cada sensor."""
    R, t = s6.cargar_toma(csv)
    inv = mapa_de(csv)
    D = {}
    for mac, seg in inv.items():
        if mac not in R:
            continue
        _, acc, gyro = R[mac]
        rep = s6.detectar_reposo(acc, gyro)
        Rseg, _ = s6.marco_segmento(acc, rep, ~rep)
        q, _, _ = s6.orientacion(acc, gyro, sesgo=s6.sesgo_gyro(gyro, rep))
        pit, rol = s6.inclinacion(q, Rseg)
        D[seg] = dict(pitch=pit, roll=rol, acc=acc, gyro=gyro, rep=rep)
    return D, t


def ciclos_de(t, ic, dur_min=0.7, dur_max=3.0):
    """Ciclos (i0, i1) entre contactos iniciales consecutivos, y si cada uno
    sigue inmediatamente al anterior (mismo tramo de marcha)."""
    cic, idx = [], []
    for p in range(len(ic) - 1):
        i0, i1 = ic[p], ic[p + 1]
        if dur_min < t[i1] - t[i0] < dur_max:
            cic.append((int(i0), int(i1)))
            idx.append(p)
    mismo = np.array([idx[k] == idx[k - 1] + 1 for k in range(1, len(idx))], bool)
    return cic, mismo


def _por_ciclo(x, cic):
    """Remuestrea x a 0-100 % de ciclo. [n_ciclos, 101]."""
    g = np.linspace(0, 1, V.N)
    return np.array([np.interp(g, np.linspace(0, 1, i1 - i0), x[i0:i1])
                     for i0, i1 in cic])


# ------------------------------------------------------------- relleno
def _puente(tt, v0, v1, neutro, tau):
    """Valores de un hueco: sale de v0, pasa por la pose neutra, entra a v1.

    Sin esto la animacion salta de golpe entre la ultima zancada y el parado.
    """
    y = np.full(len(tt), float(neutro))
    if len(tt) == 0:
        return y
    if v0 is not None:
        w = np.clip((tt - tt[0]) / tau, 0, 1)
        w = 0.5 - 0.5 * np.cos(np.pi * w)
        y = (1 - w) * v0 + w * y
    if v1 is not None:
        w = np.clip((tt[-1] - tt) / tau, 0, 1)
        s = 1.0 - (0.5 - 0.5 * np.cos(np.pi * w))
        y = (1 - s) * y + s * v1
    return y


def rellenar(x, t, neutro, tau=0.25):
    """Completa los tramos sin ciclo de marcha (parado) con la pose neutra."""
    x = np.asarray(x, float).copy()
    ok = ~np.isnan(x)
    if not ok.any():
        return np.full(len(x), float(neutro))
    n, i = len(x), 0
    while i < n:
        if ok[i]:
            i += 1
            continue
        j = i
        while j < n and not ok[j]:
            j += 1
        x[i:j] = _puente(t[i:j],
                         x[i - 1] if i > 0 else None,
                         x[j] if j < n else None,
                         neutro, tau)
        i = j
    return x


# --------------------------------------------------------- reconstruccion
# Canales de hombro/antebrazo que en el .mot de OpenCap salen degenerados: con
# el brazo pegado al cuerpo la rotacion axial del humero deja de ser observable
# por video markerless y se va a valores absurdos (arm_rot_l llega a -489 deg en
# test3, +318 en test7). No son saltos de 360: no es enrollamiento, es que la
# medicion no existe. La variedad los aprendio asi y la reconstruccion los
# reproduce, y es la causa de que el modelo se vea retorcido en OpenSim.
#
# Como ademas el montaje S NO MIDE los brazos -- salen enteros del prior, ver la
# cabecera de este archivo -- congelarlos en un valor fijo no pierde ninguna
# medicion: cambia basura por una pose quieta y honesta. El braceo (arm_flex,
# arm_add, elbow_flex) SI queda del prior, que ahi si es razonable.
BRAZOS_DEGENERADOS = ["arm_rot_r", "pro_sup_r", "arm_rot_l", "pro_sup_l"]


def congelar_brazos(out, canales=BRAZOS_DEGENERADOS):
    """Deja los canales degenerados en su mediana, en vez de dejarlos girar."""
    for c in canales:
        if c in out:
            x = np.asarray(out[c], float)
            out[c] = np.full(len(x), float(np.nanmedian(x)))
    return out


def reconstruir(csv, v, sigma=(9.2, 5.6), verbose=True, baseline=False,
                redist=False, tobillo_prior=False, brazos_prior=False):
    """Devuelve (t, datos) listo para `mot_io.escribir_mot`.

    Con `baseline=True` ignora lo que midio el IMU y devuelve el CICLO MEDIO
    de la variedad, estirado a la cadencia medida. Es la linea base honesta
    contra la que hay que comparar: cualquier metodo tiene que ganarle a
    "la caminata generica de este sujeto". Lo usa `validar_reconstruccion.py`.
    """
    D, t = preparar_imu(csv)
    if "espalda" not in D:
        raise SystemExit("falta el sensor de sacro (espalda): sin el no hay "
                         "cadena sagital ni pelvis_tilt")
    n = len(t)
    pel_p, pel_r = D["espalda"]["pitch"], D["espalda"]["roll"]

    # ciclos de marcha por pie, desde el ZUPT
    info = {}
    for seg, suf in PIES:
        if seg not in D:
            if verbose:
                print("  aviso: falta %s, esa pierna sale solo del prior" % seg)
            continue
        ic, fo, _ = s6.eventos_marcha(t, D[seg]["acc"], D[seg]["gyro"])
        cic, mismo = ciclos_de(t, ic)
        if len(cic) >= 2:
            info[seg] = dict(suf=suf, cic=cic, mismo=mismo)
        elif verbose:
            print("  aviso: %s tiene %d ciclos, no alcanza" % (seg, len(cic)))
    if not info:
        raise SystemExit("no se detecto ningun ciclo de marcha en esta toma")

    # El signo del eje AP sale de una PCA, que da la direccion pero no el
    # sentido. Sin pose estatica hay que resolverlo: se prueban las cuatro
    # combinaciones (pie, sacro) y gana la de menor residuo contra la
    # variedad. O sea que la propia variedad resuelve el signo, sin OpenCap.
    mejor = None
    for sp in (+1.0, -1.0):
        total, sub = 0.0, {}
        for seg, d in info.items():
            elegido = None
            for sf in (+1.0, -1.0):
                m_cad = _por_ciclo(sf * D[seg]["pitch"] - sp * pel_p, d["cic"])
                m_til = _por_ciclo(sp * pel_p, d["cic"])
                C, offs, res = V.ajustar(v, m_cad, m_til, d["mismo"], sigma,
                                         tobillo_del_prior=tobillo_prior)
                if elegido is None or res < elegido["res"]:
                    elegido = dict(res=res, sf=sf, C=C, offs=offs,
                                   m_cad=m_cad, m_til=m_til)
            sub[seg] = elegido
            total += elegido["res"]
        if mejor is None or total < mejor[0]:
            mejor = (total, sp, sub)
    _, sp, sub = mejor

    # sintesis + redistribucion del residuo (el pie queda donde el IMU dice)
    for seg, d in info.items():
        e = sub[seg]
        C = np.zeros_like(e["C"]) if baseline else e["C"]
        Y = V.sintetizar(v, C)
        if not baseline and redist:
            _, off_cad, _, _ = V._operador(v, tobillo_prior)
            Y = V.redistribuir(v, Y, e["m_cad"], off_cad + e["offs"][0])
        e["Y"] = Y

    if verbose:
        print("  signo sacro %+.0f" % sp)
        for seg, d in info.items():
            e = sub[seg]
            print("  %-8s %2d ciclos  signo %+.0f  offset cadena %+6.1f deg"
                  "  residuo %5.1f deg" % (seg, len(d["cic"]), e["sf"],
                                           e["offs"][0], np.sqrt(e["res"])))
        # Los dos pies deberian elegir el mismo signo: llevan el sensor igual
        # puesto. Que discrepen es la senal mas clara de que la resolucion de
        # signo fallo en uno de los dos, y esa pierna sale espejada.
        sfs = {e["sf"] for e in sub.values()}
        if len(sfs) > 1:
            print("  AVISO: los dos pies eligieron signos distintos. Revisa la"
                  " pierna del residuo mas alto: puede estar espejada.")

    # ------------------------------------------------- armar la serie de tiempo
    out = {c: np.full(n, np.nan) for c in mot_io.COLUMNAS}
    suma = {c: np.zeros(n) for c in V.CENTRAL_DET + V.CENTRAL_ABS[2:]}
    cuenta = {c: np.zeros(n) for c in suma}
    vel_s, vel_c = np.zeros(n), np.zeros(n)
    a_zanc, b_zanc = float(v["reg_zancada"][0]), float(v["reg_zancada"][1])

    for seg, d in info.items():
        suf, Y = d["suf"], sub[seg]["Y"]
        gc = np.linspace(0, 1, V.N)
        for k, (i0, i1) in enumerate(d["cic"]):
            g = np.linspace(0, 1, i1 - i0)
            for c in V.BILAT:                       # pierna y brazo de ESTE lado
                out[c + "_" + suf][i0:i1] = np.interp(
                    g, gc, Y[k, V.IDX[c + "_ipsi"]])
            for c in suma:                          # tronco y pelvis: promedio
                suma[c][i0:i1] += np.interp(g, gc, Y[k, V.IDX[c]])
                cuenta[c][i0:i1] += 1.0
            T = t[i1] - t[i0]                       # progresion segun cadencia
            vel_s[i0:i1] += (a_zanc * T + b_zanc) / T
            vel_c[i0:i1] += 1.0

    for c in suma:
        out[c] = np.where(cuenta[c] > 0,
                          suma[c] / np.maximum(cuenta[c], 1), np.nan)

    # pelvis_tilt y pelvis_list son MEDIDOS: van completos, sin prior.
    off_til = np.mean([sub[s]["offs"][1] for s in info])
    out["pelvis_tilt"] = sp * pel_p - off_til
    lst = sp * pel_r
    out["pelvis_list"] = lst - (lst.mean() - float(v["mu"][V.IDX["pelvis_list"]].mean()))

    # rellenar los tramos de parado con la pose neutra
    neutro = {c: 0.0 for c in mot_io.COLUMNAS}
    for c in mot_io.COLUMNAS:
        if c in ("pelvis_tilt", "pelvis_list", "pelvis_tx"):
            continue
        out[c] = rellenar(out[c], t, neutro[c])
    out["pelvis_ty"] = out["pelvis_ty"] + float(v["altura_pelvis"])

    # progresion: velocidad media entre los dos pies, integrada
    vel = np.where(vel_c > 0, vel_s / np.maximum(vel_c, 1), 0.0)
    vel = np.convolve(vel, np.ones(25) / 25.0, mode="same")
    out["pelvis_tx"] = np.concatenate([[0.0], np.cumsum(vel[1:] * np.diff(t))])

    # mascara de "esto es marcha reconstruida, no parado". No es una columna
    # del .mot (escribir_mot ignora las claves que no lo son); la usa la
    # validacion para no medirse a si misma sobre los tramos de parado.
    out["_marcha"] = vel_c > 0

    if not brazos_prior:
        congelar_brazos(out)
        if verbose:
            print("  hombro/antebrazo congelados (%s): el prior de OpenCap los"
                  " da degenerados. --brazos-prior los deja como estaban"
                  % ", ".join(BRAZOS_DEGENERADOS))

    frac = float(np.mean(vel_c > 0)) * 100
    if verbose:
        print("  %.0f %% de la toma es marcha reconstruida; el resto, parado"
              % frac)
        print("  avance total %.1f m" % out["pelvis_tx"][-1])
    return t, out


# ------------------------------------------------------------------ variedad
def _pares_de(carpeta, trials):
    return [rutas_trial(carpeta, x) for x in trials]


def obtener_variedad(ruta_npz, excluir=None, n_pc=5, verbose=True, tag=None):
    """Carga la variedad del .npz, o la construye si no existe.

    `tag` elige la sesion de OpenCap. Para el 2026-09-10 la variedad util es
    un solo trial (`Caminata1Normal6_1`), asi que si ademas se pide excluirlo
    -- leave-one-take-out sobre su propia toma pareada -- no queda nada. En
    ese caso se cae a los trials del 2026-09-01, que son del MISMO sujeto pero
    de otro dia: eso sigue siendo leave-one-take-out y de hecho es una prueba
    mas dura, porque mide si la variedad generaliza entre sesiones.
    """
    if ruta_npz and os.path.isfile(ruta_npz):
        if verbose:
            print("variedad: %s" % ruta_npz)
        return V.cargar(ruta_npz)

    carpeta, todos = dir_oc_de(tag or "")
    trials = [x for x in todos if x != excluir]
    if not trials:
        raise SystemExit("sin trials para la variedad: excluir %s deja vacia la "
                         "sesion (usar --sin-excluir)" % excluir)
    if verbose:
        print("construyendo la variedad con %d trials%s"
              % (len(trials), " (excluyendo %s)" % excluir if excluir else ""))
    pares = _pares_de(carpeta, trials)
    v = V.construir(pares, n_pc=n_pc, verbose=verbose)
    if ruta_npz:
        V.guardar(v, ruta_npz)
        if verbose:
            print("  guardada en %s" % ruta_npz)
    return v


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", help="tag (testnorma2l9112026) o ruta a un CSV de los sensores")
    p.add_argument("--salida", default=DIR_SALIDAS,
                   help="carpeta donde dejar el .mot")
    p.add_argument("--variedad", default=None,
                   help="archivo .npz de la variedad; si no existe, se crea")
    p.add_argument("--sin-excluir", action="store_true", dest="sin_excluir",
                   help="NO excluir el trial emparejado de la variedad "
                        "(deja de ser leave-one-take-out: solo para demo)")
    p.add_argument("--n-pc", type=int, default=5, dest="n_pc",
                   help="componentes de la variedad. DEFAULT 5, que es el "
                        "valor medido como mejor con el sensor real")
    p.add_argument("--redistribuir", action="store_true",
                   help="forzar que la cadena sintetizada iguale EXACTAMENTE "
                        "a la medida (el pie cae donde dice el IMU). Se paga "
                        "con ~1 grado mas de RMSE en rodilla: ver el manual")
    p.add_argument("--brazos-prior", action="store_true", dest="brazos_prior",
                   help="NO congelar arm_rot/pro_sup. Los deja como los da la "
                        "variedad, que es como se veian antes: el hombro gira "
                        "150-200 grados porque OpenCap no observa esa rotacion")
    p.add_argument("--baseline", action="store_true",
                   help="ignorar el IMU y devolver el ciclo medio del sujeto "
                        "a la cadencia medida. Sirve para comparar")
    a = p.parse_args()

    csv = buscar_csv(a.toma)
    # el tag es lo que va antes de `_log_` en el nombre que pone acquire.py
    # (p. ej. `testnorma2l9112026`); lleva la fecha para no pisarse entre
    # sesiones.
    base = os.path.basename(csv)
    tag_largo = base.split("_log_")[0]
    tag = tag_largo if tag_largo in PARES else base.split("_")[0]
    excluir = None if a.sin_excluir else PARES.get(tag)
    print("toma: %s" % os.path.basename(csv))

    v = obtener_variedad(a.variedad, excluir=excluir, n_pc=a.n_pc, tag=tag)
    t, datos = reconstruir(csv, v, redist=a.redistribuir, baseline=a.baseline,
                           brazos_prior=a.brazos_prior)

    if not os.path.isdir(a.salida):
        os.makedirs(a.salida)
    ruta = os.path.join(a.salida, tag + "_reconstruido.mot")
    osim = osim_de(tag)
    lim = mot_io.limites_osim(osim) if os.path.isfile(osim) else None
    ruta, faltan, recortado = mot_io.escribir_mot(ruta, t, datos, limites=lim)
    print("\nlisto: %s" % ruta)
    if faltan:
        print("  (columnas escritas en 0: %s)" % ", ".join(faltan))
    for c, pct in sorted(recortado.items(), key=lambda kv: -kv[1]):
        print("  recortado al rango del modelo: %-18s %.1f %% de las muestras"
              % (c, pct))
    print("\nAbrilo asi:  OpenSim GUI -> OPENCAP/OSIM/LaiUhlrich2022_scaled.osim")
    print("             File > Load Motion > %s" % os.path.basename(ruta))


if __name__ == "__main__":
    main()
