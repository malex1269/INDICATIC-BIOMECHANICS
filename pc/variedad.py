"""
variedad.py -- la "variedad de marcha" del sujeto, y el estimador que la ajusta.

QUE ES
------
El montaje S (cabeza, sacro, 2 manos, 2 pies) NO puede medir la rodilla: la
orientacion del pie da 3 ecuaciones para 6 incognitas de la cadena
pelvis->pie, y en marcha cadera/rodilla/tobillo giran sobre ejes casi
paralelos. Lo unico que queda medido es la SUMA `hip - knee + ankle`.

Entonces no se mide la rodilla: se ELIGE, entre las trayectorias de rodilla
que este sujeto efectivamente produjo (medidas por OpenCap), la que mejor
explica lo que midio el IMU. Eso es la "variedad": una PCA sobre los ciclos
de marcha del propio sujeto, que reduce las incognitas de 4x101 por ciclo a
~5 coeficientes. El problema pasa de subdeterminado a sobredeterminado.

El estimador es una regresion ridge en forma cerrada -- sin entrenamiento,
sin red neuronal, determinista:

    c = (A' W A + L^-1)^-1 A' W (m - off)

con W = 1/sigma^2 usando los sigma YA MEDIDOS contra OpenCap (9.2 deg en la
cadena, 5.6 deg en la pelvis) y L = autovalores de la PCA. No hay
hiperparametro que ajustar.

DECISIONES QUE YA SE MIDIERON (resumen en docs/4_integracion_opensim.md)
------------------------------------------------------------------------
- n_pc = 5. Con el sensor real, mas componentes solo amplifican el ruido.
- EL TOBILLO SALE DEL PRIOR, no del ajuste: con el error real del sensor la
  reconstruccion del tobillo sale PEOR que usar el ciclo medio (-11 %).
  Por eso no entra en el operador de observacion (`tobillo_del_prior`).
- Cadera y rodilla si mejoran: ~+27 % y ~+26 % sobre el ciclo medio.

LIMITE, dicho claro: la variedad solo contiene patrones que el sujeto hizo
delante de la camara. Si cojea de una forma que no esta en el entrenamiento,
sale una caminata normal, no la cojera.
"""
import os
import numpy as np

import opencap_io as oc

N = 101                      # muestras por ciclo normalizado (0-100 %)

# Canales absolutos: su nivel medio significa algo (inclinacion vs gravedad).
CENTRAL_ABS = ["pelvis_tilt", "pelvis_list",
               "lumbar_extension", "lumbar_bending", "lumbar_rotation"]
# Canales a los que hay que quitarles la media POR CICLO: su nivel absoluto
# es la posicion/rumbo del sujeto en el laboratorio, no fisiologia. Si no se
# quita, la variedad aprende "hacia donde camino ese dia" en vez del gesto.
CENTRAL_DET = ["pelvis_rotation", "pelvis_ty", "pelvis_tz"]
# Bilaterales, guardados como ipsi/contra respecto de la pierna del ciclo.
BILAT = ["hip_flexion", "hip_adduction", "hip_rotation", "knee_angle",
         "ankle_angle", "subtalar_angle", "mtp_angle",
         "arm_flex", "arm_add", "arm_rot", "elbow_flex", "pro_sup"]

CANALES = (CENTRAL_ABS + CENTRAL_DET
           + [c + "_ipsi" for c in BILAT] + [c + "_contra" for c in BILAT])
IDX = {c: i for i, c in enumerate(CANALES)}
NC = len(CANALES)


# ------------------------------------------------------------------ utilidad
def _suav(x, n=9):
    k = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(x, k, mode="same")
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, x)


def _picos(s, k, mask):
    """Maximos locales de s en una ventana +-k, dentro de mask."""
    out = []
    for i in range(k, len(s) - k):
        if mask[i] and s[i] == s[i - k:i + k + 1].max():
            if not out or i - out[-1] > k:
                out.append(i)
    return np.array(out, int)


# -------------------------------------------------- extraccion de ciclos
def extraer_ciclos(trc, mot, v_min=0.30):
    """Ciclos de marcha rectilinea de una toma de OpenCap.

    Devuelve (Y, T, L, H): Y [n_ciclos, NC, 101], T = duracion de cada ciclo
    (s), L = largo de zancada de cada ciclo (m), H = altura media de la pelvis
    en el ciclo (m) -- hace falta porque a `pelvis_ty` se le quita la media y
    despues hay que devolverle un nivel absoluto o el modelo flota.

    Detecta el contacto inicial SIN asumir la direccion de marcha: proyecta
    (talon_der - talon_izq) sobre la direccion instantanea de avance de la
    pelvis. Maximos = contacto derecho, minimos = izquierdo. Asi aguanta que
    el sujeto vaya y vuelva, que es lo que hace en estas tomas.
    """
    t, M = oc.leer_trc(trc)
    tm, df = oc.leer_mot(mot)
    fs = 1.0 / np.median(np.diff(t))

    pel = _suav(M["midHip"], 15)
    v = np.gradient(pel, axis=0) * fs
    vh = v[:, [0, 2]]                                 # plano horizontal, Y=arriba
    rap = _suav(np.linalg.norm(vh, axis=1), 9)
    vhn = vh / np.maximum(np.linalg.norm(vh, axis=1)[:, None], 1e-9)
    # Umbral de velocidad de pelvis para considerar 'caminando'.
    #
    # NO BAJARLO sin mirar la regresion de zancada. Se probo, porque las tomas
    # lentas del 2026-09-10 (`Caminata2Lento1` y `Caminata3Problema`, mediana
    # 0.19 y 0.20 m/s) dan CERO ciclos con 0.30. Bajando el umbral entran, pero
    # lo que entra no es marcha:
    #
    #   v_min   ciclos   varianza en 5 PC   zancada = a*T + b
    #   0.30      20           73 %          a = +0.52
    #   0.15      29           64 %          a = -0.54   <- imposible
    #   0.10      39           60 %          a = -0.16   <- imposible
    #
    # Una pendiente negativa dice que a mayor duracion de zancada, menor largo
    # de zancada. Son arrastres y giros colandose como ciclos. El parametro
    # queda expuesto para poder repetir la prueba, no para usarlo.
    mov = rap > v_min                                 # solo tramos caminando

    tramos, i = [], 0
    while i < len(mov):
        if mov[i]:
            j = i
            while j < len(mov) and mov[j]:
                j += 1
            if t[j - 1] - t[i] > 1.5:
                tramos.append((i, j))
            i = j
        else:
            i += 1

    s = _suav(np.einsum("ij,ij->i",
                        (M["r_calc_study"] - M["L_calc_study"])[:, [0, 2]],
                        vhn), 9)
    k = int(0.35 * fs)

    Y, T, L, H = [], [], [], []
    for lado, sg in (("r", +1.0), ("l", -1.0)):
        ica = _picos(sg * s, k, mov)
        ipsi, contra = lado, ("l" if lado == "r" else "r")
        for a, b in tramos:
            ic = ica[(ica >= a) & (ica < b)]
            for p in range(len(ic) - 1):
                i0, i1 = ic[p], ic[p + 1]
                dur = t[i1] - t[i0]
                if not (0.7 < dur < 2.0):
                    continue
                g = np.linspace(t[i0], t[i1], N)
                fila = []
                for c in CENTRAL_ABS:
                    fila.append(np.interp(g, tm, df[c].to_numpy(float)))
                for c in CENTRAL_DET:
                    x = np.interp(g, tm, df[c].to_numpy(float))
                    if c == "pelvis_ty":
                        H.append(float(x.mean()))
                    fila.append(x - x.mean())
                for suf in (ipsi, contra):
                    for c in BILAT:
                        fila.append(np.interp(g, tm,
                                              df[c + "_" + suf].to_numpy(float)))
                Y.append(np.array(fila))
                T.append(dur)
                L.append(float(np.linalg.norm(pel[i1, [0, 2]] - pel[i0, [0, 2]])))
    if not Y:
        return np.zeros((0, NC, N)), np.zeros(0), np.zeros(0), np.zeros(0)
    return np.array(Y), np.array(T), np.array(L), np.array(H)


# ----------------------------------------------------------- construccion
def construir(pares, n_pc=5, verbose=True, v_min=0.30):
    """Arma la variedad a partir de pares (ruta_trc, ruta_mot) de OpenCap.

    Devuelve un dict con todo lo que hace falta para reconstruir despues.
    """
    Ys, Ts, Ls, Hs = [], [], [], []
    for trc, mot in pares:
        Y, T, L, H = extraer_ciclos(trc, mot, v_min=v_min)
        if verbose:
            print("  %-28s %3d ciclos" % (os.path.basename(mot), len(Y)))
        if len(Y):
            Ys.append(Y)
            Ts.append(T)
            Ls.append(L)
            Hs.append(H)
    if not Ys:
        raise RuntimeError("no se encontro ningun ciclo de marcha")
    Y = np.concatenate(Ys)
    T = np.concatenate(Ts)
    L = np.concatenate(Ls)
    H = np.concatenate(Hs)

    mu = Y.mean(axis=0)                               # [NC, 101]
    # Estandarizar por canal ANTES de la PCA: si no, la rodilla (70 deg de
    # ROM) tapa a pelvis_list (5 deg) y la base no representa el tronco.
    esc = np.maximum(Y.std(axis=(0, 2)), 1e-6)        # [NC]
    Z = ((Y - mu) / esc[None, :, None]).reshape(len(Y), NC * N)
    _, S, Vt = np.linalg.svd(Z, full_matrices=False)
    var = S ** 2 / max(len(Y) - 1, 1)

    n_pc = min(n_pc, len(Vt))
    # Largo de zancada en funcion de la duracion del ciclo: asi la progresion
    # del modelo responde a la CADENCIA MEDIDA en vez de ser un valor fijo.
    # (Y evita la doble integracion strapdown, que dio clearance de 8 cm a
    # 1.06 m y es inusable.)
    reg = np.polyfit(T, L, 1) if len(T) > 2 else np.array([0.0, float(L.mean())])

    v = dict(mu=mu, esc=esc, B=Vt[:n_pc], lam=np.maximum(var[:n_pc], 1e-9),
             var_total=var, n_pc=n_pc, n_ciclos=len(Y),
             reg_zancada=reg, T=T, L=L, altura_pelvis=float(np.median(H)),
             canales=np.array(CANALES))
    if verbose:
        ac = np.cumsum(var) / var.sum() * 100
        print("  %d ciclos | %d componentes -> %.0f %% de varianza"
              % (len(Y), n_pc, ac[n_pc - 1]))
        print("  zancada = %.3f * T + %.3f  (m)" % (reg[0], reg[1]))
    return v


def guardar(v, ruta):
    np.savez(ruta, **v)
    return ruta


def cargar(ruta):
    d = np.load(ruta, allow_pickle=True)
    return {k: d[k] for k in d.files}


# ------------------------------------------------------------- estimador
def _operador(v, tobillo_del_prior=True):
    """Matrices A y offsets que ligan los coeficientes con lo que se mide.

    Observacion 1 -- cadena sagital = hip_flexion - knee_angle + ankle_angle.
    Observacion 2 -- pelvis_tilt.

    Con `tobillo_del_prior` el tobillo entra SOLO con su media (no depende de
    los coeficientes): es la decision que se midio al validar el metodo.
    """
    mu, esc, B = v["mu"], v["esc"], v["B"]
    n_pc = B.shape[0]
    Bc = B.reshape(n_pc, NC, N)

    partes = [("hip_flexion_ipsi", +1.0), ("knee_angle_ipsi", -1.0)]
    if not tobillo_del_prior:
        partes.append(("ankle_angle_ipsi", +1.0))

    A_cad = sum(sg * esc[IDX[c]] * Bc[:, IDX[c], :] for c, sg in partes).T
    off_cad = (mu[IDX["hip_flexion_ipsi"]] - mu[IDX["knee_angle_ipsi"]]
               + mu[IDX["ankle_angle_ipsi"]])
    A_til = (esc[IDX["pelvis_tilt"]] * Bc[:, IDX["pelvis_tilt"], :]).T
    off_til = mu[IDX["pelvis_tilt"]]
    return A_cad, off_cad, A_til, off_til


def ajustar(v, m_cadena, m_tilt, mismo_tramo=None, sigma=(9.2, 5.6),
            alpha=1.0, tobillo_del_prior=True):
    """Ajusta TODOS los ciclos de una toma de una vez.

    m_cadena, m_tilt: [n_ciclos, 101], ya remuestreados a 0-100 % de ciclo.
    mismo_tramo: [n_ciclos-1] bool, True si el ciclo k sigue inmediatamente
        al k-1 (mismo tramo de marcha). Solo entre esos se aplica el termino
        de continuidad, que es lo que evita que la animacion salte de zancada
        en zancada.

    Devuelve (C, offsets, residuo): C [n_ciclos, n_pc].

    Las incognitas son los coeficientes de cada ciclo MAS dos offsets
    constantes por toma (uno por observacion). Los offsets hacen falta porque
    sin pose estatica de calibracion el nivel absoluto de `inclinacion()` es
    desconocido; se estiman sin penalizar, y con ~10 ciclos quedan bien
    determinados. Con los 10 s de pose estatica del protocolo se MIDEN y esto
    sobra.
    """
    A_cad, off_cad, A_til, off_til = _operador(v, tobillo_del_prior)
    lam = v["lam"]
    n_pc = len(lam)
    K = len(m_cadena)
    if mismo_tramo is None:
        mismo_tramo = np.ones(max(K - 1, 0), bool)

    P = n_pc * K + 2
    M = np.zeros((P, P))
    b = np.zeros(P)
    uno = np.ones(N)
    inv_lam = np.diag(1.0 / lam)
    I = np.eye(n_pc)

    for k in range(K):
        sl = slice(k * n_pc, (k + 1) * n_pc)
        for A, off, m, sg, j in ((A_cad, off_cad, m_cadena[k], sigma[0], -2),
                                 (A_til, off_til, m_tilt[k], sigma[1], -1)):
            w = 1.0 / sg ** 2
            r = m - off
            M[sl, sl] += w * (A.T @ A)
            M[sl, j] += w * (A.T @ uno)
            M[j, sl] += w * (uno @ A)
            M[j, j] += w * N
            b[sl] += w * (A.T @ r)
            b[j] += w * r.sum()
        M[sl, sl] += inv_lam

    # continuidad entre ciclos consecutivos del mismo tramo
    for k in range(1, K):
        if not mismo_tramo[k - 1]:
            continue
        a = slice((k - 1) * n_pc, k * n_pc)
        c = slice(k * n_pc, (k + 1) * n_pc)
        G = np.diag(alpha / lam)
        M[a, a] += G
        M[c, c] += G
        M[a, c] -= G
        M[c, a] -= G

    X = np.linalg.solve(M, b)
    C = X[:n_pc * K].reshape(K, n_pc)
    offs = X[-2:]

    res = 0.0
    for k in range(K):
        res += np.mean((m_cadena[k] - off_cad - offs[0] - A_cad @ C[k]) ** 2)
    return C, offs, float(res / max(K, 1))


def sintetizar(v, C):
    """Coeficientes -> formas de onda. Devuelve [n_ciclos, NC, 101]."""
    mu, esc, B = v["mu"], v["esc"], v["B"]
    Y = (C @ B).reshape(len(C), NC, N) * esc[:, None] + mu
    return Y


def redistribuir(v, Y, m_cadena, off_cadena):
    """Hace que la cadena sintetizada iguale EXACTAMENTE a la medida.

    El ajuste es blando; despues queda un residuo. Se reparte entre cadera y
    rodilla (el tobillo no, viene del prior) proporcional al desvio del prior
    de cada una: el pie queda en la orientacion que midio el IMU -- que es lo
    que el ojo mira en la animacion -- y el reparto lo decide el prior.

    cadena = hip - knee + ankle, asi que d_hip - d_knee = r.
    """
    ih, ik, ia = IDX["hip_flexion_ipsi"], IDX["knee_angle_ipsi"], IDX["ankle_angle_ipsi"]
    sh, sk = float(v["esc"][ih]), float(v["esc"][ik])
    wh = sh ** 2 / (sh ** 2 + sk ** 2)
    wk = 1.0 - wh
    Y = Y.copy()
    for k in range(len(Y)):
        r = (m_cadena[k] - off_cadena) - (Y[k, ih] - Y[k, ik] + Y[k, ia])
        Y[k, ih] += r * wh
        Y[k, ik] -= r * wk
    return Y
