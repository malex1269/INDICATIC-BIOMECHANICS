"""
sparse6.py -- procesamiento del arreglo sparse-6 en modo log.

Todo lo que se puede sacar de los CSV de modo log SIN OpenCap: carga y
remuestreo en reloj de dispositivo, calibracion funcional (sin pose
estatica), sesgo de giroscopio por ZUPT, orientacion VQF 6D, eventos de
marcha, parametros espacio-temporales, trayectoria de pie por ZUPT y el
angulo de la cadena sagital pelvis->pie.

Unidades de entrada: acc en g, gyro en deg/s (formato de acquire.py).
"""
import numpy as np
from scipy import signal
import vqf

G = 9.80665
FS = 100.0


# --------------------------------------------------------------- carga
def cargar_toma(csv, fs=FS):
    """CSV largo -> dict mac -> (t, acc_g[N,3], gyro_dps[N,3]) en grilla comun.

    El tiempo es `epoch_device` absoluto: la grilla arranca en el ultimo
    arranque y termina en el primer fin, asi que los 6 sensores comparten
    exactamente las mismas muestras.
    """
    import pandas as pd
    df = pd.read_csv(csv).rename(columns={"epoch_device": "epoch"})

    # Un CSV de SALIDA (`_series.csv`, `_eventos.csv`) abierto por error daba
    # `KeyError: 'mac'` con un traceback de pandas, que no le dice nada a
    # nadie. Se chequea acá para que el mensaje diga que pasa y que hacer.
    faltan = [c for c in ("mac", "type", "x", "y", "z", "epoch") if c not in df.columns]
    if faltan:
        import os as _os
        raise ValueError(
            "'%s' no es un CSV de sensores: le faltan las columnas %s.\n"
            "Los CSV de sensores son los de la carpeta de la sesion y tienen\n"
            "  timestamp_host, mac, type, x, y, z, epoch_device\n"
            "Los `_series.csv` y `_eventos.csv` son SALIDAS del analisis, no\n"
            "entradas: no se abren con la app."
            % (_os.path.basename(csv), ", ".join(faltan)))

    crudo = {}
    for mac, g in df.groupby("mac"):
        d = {}
        for tipo in ("acc", "gyro"):
            s = g[g["type"] == tipo].sort_values("epoch")
            if len(s) < 100:
                d = None
                break
            d[tipo] = (s["epoch"].to_numpy(float) / 1000.0,
                       s[["x", "y", "z"]].to_numpy(float))
        if d:
            crudo[mac] = d
    if not crudo:
        return {}, np.array([])
    t0 = max(max(d["acc"][0][0], d["gyro"][0][0]) for d in crudo.values())
    t1 = min(min(d["acc"][0][-1], d["gyro"][0][-1]) for d in crudo.values())
    t = np.arange(0.0, t1 - t0, 1.0 / fs)
    out = {}
    for mac, d in crudo.items():
        r = []
        for tipo in ("acc", "gyro"):
            tt, xyz = d[tipo]
            r.append(np.column_stack([np.interp(t + t0, tt, xyz[:, k])
                                      for k in range(3)]))
        out[mac] = (t, r[0], r[1])
    return out, t


# ------------------------------------------------- reposo / apoyo / sesgo
def _rachas_min(q, n_min):
    out = np.zeros_like(q)
    i = 0
    while i < len(q):
        if q[i]:
            j = i
            while j < len(q) and q[j]:
                j += 1
            if j - i >= n_min:
                out[i:j] = True
            i = j
        else:
            i += 1
    return out


def detectar_reposo(acc_g, gyro_dps, fs=FS, th_w=25.0, th_a=0.12, min_ms=80):
    """True donde el sensor esta quieto: |w| bajo y |a| ~ 1 g, sostenido."""
    wm = np.linalg.norm(gyro_dps, axis=1)
    am = np.linalg.norm(acc_g, axis=1)
    b, a = signal.butter(2, 5.0 / (fs / 2), "low")
    wm_s = signal.filtfilt(b, a, wm)
    am_s = signal.filtfilt(b, a, am)
    q = (wm_s < th_w) & (np.abs(am_s - 1.0) < th_a)
    return _rachas_min(q, int(min_ms * fs / 1000))


def sesgo_gyro(gyro_dps, reposo):
    """Sesgo (deg/s) = media del gyro en las ventanas de reposo."""
    if reposo.sum() < 50:
        return np.zeros(3)
    return gyro_dps[reposo].mean(axis=0)


# --------------------------------------------- calibracion funcional
def eje_flexion(gyro_dps, mascara):
    """Eje principal de rotacion del segmento (eje medio-lateral, de flexion).

    PCA de la velocidad angular durante el movimiento: en marcha la flexo-
    extension domina, asi que la primera componente es el eje ML del
    segmento. Reemplaza a la pose estatica que no se grabo en estas tomas.
    Devuelve (eje unitario, fraccion de varianza explicada).
    """
    W = gyro_dps[mascara]
    W = W - W.mean(axis=0)
    C = W.T @ W / max(len(W) - 1, 1)
    val, vec = np.linalg.eigh(C)
    e = vec[:, -1]
    return e / np.linalg.norm(e), val[-1] / val.sum()


def eje_ap(acc_g, z, caminando):
    """Eje antero-posterior funcional: direccion horizontal de mayor varianza
    de la aceleracion durante la marcha (frenado/propulsion domina sobre el
    sway medio-lateral). El signo queda indeterminado.
    """
    A = acc_g[caminando]
    A = A - A.mean(axis=0)
    A = A - np.outer(A @ z, z)                   # quitar la componente vertical
    C = A.T @ A / max(len(A) - 1, 1)
    val, vec = np.linalg.eigh(C)
    e = vec[:, -1]
    return e / np.linalg.norm(e), val[-1] / max(val[1] + val[2], 1e-12)


def marco_segmento(acc_g, reposo, caminando, signo=1.0):
    """R_sensor->segmento por calibracion funcional (sin pose estatica).

    z_seg = vertical (gravedad medida en reposo)
    x_seg = adelante   (eje AP funcional, ver eje_ap)
    y_seg = z x x      (medio-lateral, eje de flexo-extension)

    Sin pose estatica y sin magnetometro esto fija la vertical y el plano
    sagital de cada sensor por separado, pero NO el signo de "adelante" ni el
    rumbo comun entre sensores. `signo` permite fijarlo desde afuera cuando
    hay una regla (p. ej. en los pies, pico de balanceo positivo).
    """
    if reposo.sum() < 50:
        z = acc_g.mean(axis=0)
    else:
        z = acc_g[reposo].mean(axis=0)
    z = z / np.linalg.norm(z)
    x, rel = eje_ap(acc_g, z, caminando)
    x = signo * x
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T, rel     # filas = ejes del segmento


# -------------------------------------------------------- orientacion
def orientacion(acc_g, gyro_dps, fs=FS, sesgo=None):
    """VQF 6D. Devuelve (q[N,4] sensor->mundo, sesgo_vqf_dps[N,3], reposo_vqf).

    VQF estima el sesgo del giroscopio online, asi que su estimacion sirve de
    control independiente del sesgo que se calcula por ZUPT.
    """
    gy = np.deg2rad(gyro_dps if sesgo is None else gyro_dps - sesgo)
    ac = acc_g * G
    r = vqf.offlineVQF(np.ascontiguousarray(gy), np.ascontiguousarray(ac),
                       None, 1.0 / fs)
    return r["quat6D"], np.degrees(r["bias"]), r["restDetected"].astype(bool)


def q2R(q):
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# ----------------------------------------------------- eventos de marcha
def eventos_marcha(t, acc_g, gyro_dps, fs=FS, min_stance_ms=150,
                   min_swing_ms=150):
    """Devuelve (ic, fo, reposo) con ic/fo en indices.

    ic = inicio de la fase de apoyo (contacto inicial), fo = fin (despegue).
    Se apoya en ZUPT, que con 100 Hz y 0 % de perdida es el metodo mas
    robusto disponible aca.
    """
    reposo = detectar_reposo(acc_g, gyro_dps, fs, min_ms=min_stance_ms)
    d = np.diff(reposo.astype(int))
    ic = np.flatnonzero(d == 1) + 1
    fo = np.flatnonzero(d == -1) + 1
    if len(ic) and len(fo) and fo[0] < ic[0]:
        fo = fo[1:]
    n = min(len(ic), len(fo))
    ic, fo = ic[:n], fo[:n]
    keep = np.ones(n, bool)
    for k in range(n - 1):
        if (ic[k + 1] - fo[k]) < min_swing_ms * fs / 1000:
            keep[k + 1] = False
    return ic[keep], fo[keep], reposo


def parametros_marcha(t, ic, fo, t_zancada_max=3.0):
    """Tiempos de zancada / apoyo / balanceo por ciclo, filtrando pausas."""
    if len(ic) < 3:
        return {}
    T = np.diff(t[ic])
    apoyo = (t[fo] - t[ic])[:-1]
    ok = (T < t_zancada_max) & (apoyo > 0)
    T, apoyo = T[ok], apoyo[ok]
    if not len(T):
        return {}
    return dict(n_ciclos=int(len(T)), t_zancada=T, t_apoyo=apoyo,
                pct_apoyo=100 * apoyo / T,
                cadencia_pasos_min=120.0 / np.median(T))


# ------------------------------------------------ trayectoria por ZUPT
def trayectoria_pie(acc_g, q, reposo, fs=FS):
    """Integracion strapdown con correccion ZUPT por paso.

    Devuelve posicion [N,3] en marco mundo (vertical valida, rumbo relativo).
    La velocidad se fuerza a 0 en cada apoyo y el error se reparte
    linealmente dentro de cada balanceo: correccion clasica de PDR.
    """
    R = q2R(q)
    a_mundo = np.einsum("nij,nj->ni", R, acc_g * G) - np.array([0.0, 0.0, G])
    dt = 1.0 / fs
    v = np.zeros_like(a_mundo)
    v[1:] = np.cumsum(a_mundo[:-1] * dt, axis=0)
    d = np.diff(reposo.astype(int))
    fin_apoyo = np.flatnonzero(d == -1) + 1
    ini_apoyo = np.flatnonzero(d == 1) + 1
    for a in fin_apoyo:
        b = ini_apoyo[ini_apoyo > a]
        if not len(b):
            break
        b = b[0]
        n = b - a
        if n < 3:
            continue
        v[a:b] -= np.outer(np.linspace(0.0, 1.0, n), v[b - 1])
    v[reposo] = 0.0
    p = np.zeros_like(v)
    p[1:] = np.cumsum(v[:-1] * dt, axis=0)
    # deriva vertical: cada apoyo esta a la misma altura (el suelo), asi que
    # la altura media de los apoyos define la deriva; se interpola y se resta.
    # Si la toma arranca con el pie ya en reposo, esa primera racha no tiene
    # un flanco 1->0 de "inicio" en el diff -- se descarta (es parcial, no
    # aporta un apoyo completo) en vez de emparejarla mal con el fin
    # equivocado (lo que daba a > b y un promedio de una ventana vacia).
    d = np.diff(reposo.astype(int))
    ini = np.flatnonzero(d == 1) + 1
    fin = np.flatnonzero(d == -1) + 1
    if len(ini) and len(fin) and fin[0] < ini[0]:
        fin = fin[1:]
    n = min(len(ini), len(fin))
    if n >= 2:
        centros = (ini[:n] + fin[:n]) / 2.0
        alturas = np.array([p[a:b, 2].mean() for a, b in zip(ini[:n], fin[:n])])
        p[:, 2] -= np.interp(np.arange(len(p)), centros, alturas)
    return p


def signo_pie_por_avance(acc_g, q, reposo, Rseg, fs=FS):
    """Signo del eje "adelante" del pie, desde su propio desplazamiento.

    El eje AP sale de una PCA (que no fija el sentido). El sentido se resuelve
    con la unica referencia que no necesita ni magnetometro ni pose estatica:
    el pie AVANZA. Se integra la trayectoria por ZUPT, se toma el
    desplazamiento horizontal neto de cada zancada y se proyecta sobre el eje
    AP candidato expresado en el marco del mundo del propio sensor. Si el
    producto es negativo, el eje apuntaba hacia atras.

    No usa OpenCap: fijar el signo contra la referencia con la que despues se
    valida seria circular.
    """
    p = trayectoria_pie(acc_g, q, reposo, fs)
    d = np.diff(reposo.astype(int))
    ini = np.flatnonzero(d == 1) + 1
    fin = np.flatnonzero(d == -1) + 1
    if len(ini) and len(fin) and fin[0] < ini[0]:
        fin = fin[1:]
    n = min(len(ini), len(fin))
    if n < 2:
        return 1.0
    # desplazamiento horizontal de apoyo a apoyo, en el mundo del sensor
    avance = np.zeros(3)
    for k in range(n - 1):
        avance += p[ini[k + 1]] - p[ini[k]]
    avance[2] = 0.0
    if np.linalg.norm(avance) < 1e-6:
        return 1.0
    # el eje AP del segmento, llevado al mundo, promediado sobre los apoyos
    Rw = q2R(q) @ Rseg.T                       # mundo <- segmento
    x_mundo = Rw[reposo][:, :, 0].mean(axis=0)
    x_mundo[2] = 0.0
    return 1.0 if x_mundo @ avance > 0 else -1.0


def parametros_espaciales(p, ic, fo):
    """Largo de zancada y clearance por ciclo, desde la trayectoria ZUPT."""
    largo, clear = [], []
    for k in range(len(ic) - 1):
        a, b = ic[k], ic[k + 1]
        largo.append(np.linalg.norm(p[b, :2] - p[a, :2]))
        if fo[k] < b:
            clear.append(p[fo[k]:b, 2].max() - p[a:fo[k] + 1, 2].mean()
                         if fo[k] > a else np.nan)
        else:
            clear.append(np.nan)
    return np.array(largo), np.array(clear)


# --------------------------------------------- cadena sagital pelvis->pie
def angulos_relativos(q_prox, q_dist, R_prox, R_dist):
    """Angulos del segmento distal respecto del proximal, en el marco proximal.

    Con los marcos funcionales de marco_segmento(), 'sagital' es la rotacion
    alrededor del eje ML del segmento proximal, o sea la SUMA de las flexiones
    de la cadena que hay entre los dos segmentos.
    """
    Rp = q2R(q_prox) @ R_prox.T          # segmento proximal -> mundo
    Rd = q2R(q_dist) @ R_dist.T
    Rrel = np.einsum("nij,njk->nik", np.transpose(Rp, (0, 2, 1)), Rd)
    sag = np.degrees(np.arctan2(Rrel[:, 0, 2], Rrel[:, 2, 2]))
    fro = np.degrees(np.arcsin(np.clip(-Rrel[:, 1, 2], -1.0, 1.0)))
    tra = np.degrees(np.arctan2(Rrel[:, 1, 0], Rrel[:, 1, 1]))
    return sag, fro, tra


def angulos_mundo(q, R_seg):
    """Inclinacion sagital y frontal del segmento respecto de la vertical."""
    Rm = q2R(q) @ R_seg.T
    sag = np.degrees(np.arctan2(Rm[:, 0, 2], Rm[:, 2, 2]))
    fro = np.degrees(np.arcsin(np.clip(-Rm[:, 1, 2], -1.0, 1.0)))
    return sag, fro


def inclinacion(q, R_seg):
    """Pitch y roll respecto de la gravedad, en el marco del propio segmento.

    LA cantidad que sirve sin magnetometro: el acelerometro ancla la vertical,
    asi que pitch y roll NO derivan, mientras que el rumbo (yaw) y cualquier
    angulo relativo entre dos sensores derivan ~1 deg/s (medido contra OpenCap
    en la sesion del 2026-09-01). Misma convencion que `opencap_io.inclinacion`.
    """
    Rw = q2R(q) @ R_seg.T                       # mundo <- segmento
    g = np.einsum("nji,j->ni", Rw, np.array([0.0, 0.0, -1.0]))
    return (np.degrees(np.arctan2(g[:, 0], -g[:, 2])),
            np.degrees(np.arctan2(g[:, 1], -g[:, 2])))


def por_ciclo(x, t, ic, n=101, t_max=2.5):
    """Normaliza x a 0-100 % de ciclo de marcha. Devuelve [n_ciclos, n]."""
    out = []
    for k in range(len(ic) - 1):
        a, b = ic[k], ic[k + 1]
        if t[b] - t[a] > t_max or b - a < 20:
            continue
        out.append(np.interp(np.linspace(0, 1, n),
                             np.linspace(0, 1, b - a), x[a:b]))
    return np.array(out)


# ----------------------------------------------------- export OpenSense
def exportar_sto(ruta, t, quats, nombres):
    """Escribe un archivo de orientaciones para OpenSense IMU IK.

    quats: dict nombre_frame -> [N,4] (w,x,y,z). nombres: orden de columnas.
    """
    with open(ruta, "w") as f:
        f.write("DataRate=%.6f\n" % (1.0 / (t[1] - t[0])))
        f.write("DataType=Quaternion\nversion=3\nOpenSimVersion=4.4\n")
        f.write("endheader\n")
        f.write("time\t" + "\t".join(nombres) + "\n")
        for i in range(len(t)):
            fila = ["%.6f" % t[i]]
            for nm in nombres:
                w, x, y, z = quats[nm][i]
                fila.append("%.8f,%.8f,%.8f,%.8f" % (w, x, y, z))
            f.write("\t".join(fila) + "\n")
    return ruta
