"""
opencap_io.py -- lectura de los export de OpenCap y marcos de segmento.

Formato confirmado con el export del 2026-09-02 (sesion del 2026-09-01):
  .mot  ->  cabecera 'Coordinates', inDegrees=yes, tiempo arranca en 0, 60 Hz,
            34 columnas: pelvis_tilt/list/rotation + pelvis_tx/ty/tz,
            hip_flexion/adduction/rotation_r|l, knee_angle_r|l, ankle_angle_r|l,
            subtalar_angle_r|l, mtp_angle_r|l, lumbar_extension/bending/rotation,
            arm_flex/add/rot_r|l, elbow_flex_r|l, pro_sup_r|l
  .trc  ->  60 Hz, 63 marcadores, metros, marco de OpenSim (Y = arriba)
  .osim ->  LaiUhlrich2022_scaled, sin cuerpo de cabeza (va dentro de torso)

No necesita el paquete `opensim`: los angulos articulares salen directo del
.mot y las orientaciones de segmento se arman de los marcadores del .trc.
"""
import numpy as np
import pandas as pd

FS_OPENCAP = 60.0


# --------------------------------------------------------------- lectura
def leer_mot(ruta):
    """Devuelve (t, DataFrame de coordenadas). Angulos en grados."""
    with open(ruta) as f:
        for i, linea in enumerate(f):
            if linea.strip() == "endheader":
                break
    df = pd.read_csv(ruta, sep="\t", skiprows=i + 1)
    df.columns = [c.strip() for c in df.columns]
    return df["time"].to_numpy(float), df


def leer_trc(ruta):
    """Devuelve (t, dict marcador -> [N,3] en metros, marco OpenSim Y=arriba)."""
    with open(ruta) as f:
        lineas = f.readlines()
    cab = [c for c in lineas[3].rstrip("\n").split("\t") if c.strip()]
    nombres = cab[2:]
    datos = np.genfromtxt(ruta, skip_header=6, delimiter="\t")
    t = datos[:, 1]
    return t, {nm: datos[:, 2 + 3 * k: 5 + 3 * k] for k, nm in enumerate(nombres)}


# ------------------------------------------------- marcos de segmento
def _norm(v):
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def _marco(adelante, izquierda):
    """Marco ortonormal [N,3,3], columnas = (adelante, izquierda, arriba).

    Es EXACTAMENTE la convencion de `sparse6.marco_segmento` del lado IMU
    (x = anterior, y = a la izquierda del sujeto, z = superior), para que los
    angulos de los dos sistemas se puedan restar sin conversiones.
    La lateralidad importa: el eje "izquierda" hay que darlo con el mismo
    sentido en los dos lados del cuerpo, o el pie izquierdo sale espejado
    respecto del derecho.
    """
    x = _norm(adelante)
    y = izquierda - np.sum(izquierda * x, axis=1, keepdims=True) * x
    y = _norm(y)
    z = np.cross(x, y)
    return np.stack([x, y, z], axis=2)


def marcos_segmento(M):
    """Marcos anatomicos de los segmentos instrumentados con IMU (montaje S)."""
    R = {}
    asis_r, asis_l = M["r.ASIS_study"], M["L.ASIS_study"]
    psis_r, psis_l = M["r.PSIS_study"], M["L.PSIS_study"]
    centro_pel = (asis_r + asis_l + psis_r + psis_l) / 4.0
    R["pelvis"] = _marco((asis_r + asis_l) / 2 - (psis_r + psis_l) / 2,
                         asis_l - asis_r)
    sh_r, sh_l = M["r_shoulder_study"], M["L_shoulder_study"]
    arriba = (sh_r + sh_l) / 2 - centro_pel
    R["torso"] = _marco(np.cross(arriba, sh_r - sh_l), sh_l - sh_r)
    for lado, pre, signo in (("r", "r_", +1.0), ("l", "L_", -1.0)):
        # 5meta esta en el borde LATERAL del pie: a la derecha en el pie
        # derecho y a la izquierda en el izquierdo -> hay que invertir el
        # signo para que "izquierda" apunte igual en los dos pies.
        R["calcn_" + lado] = _marco(
            M[pre + "toe_study"] - M[pre + "calc_study"],
            -signo * (M[pre + "5meta_study"] - M[pre + "calc_study"]))
        # Mano: adelante = del codo a la muneca; lateral = de medial a lateral
        R["hand_" + lado] = _marco(
            (M[pre + "lwrist_study"] + M[pre + "mwrist_study"]) / 2
            - M[pre + "lelbow_study"],
            -signo * (M[pre + "lwrist_study"] - M[pre + "mwrist_study"]))
    return R


def marcos_pierna(M):
    """Muslo y pierna: sin IMU en el montaje S. Son la referencia contra la
    que se mide cuanto se puede aproximar la cadena sin instrumentarlos."""
    R = {}
    for lado, pre, hjc, signo in (("r", "r_", "RHJC_study", +1.0),
                                  ("l", "L_", "LHJC_study", -1.0)):
        centro_rod = (M[pre + "knee_study"] + M[pre + "mknee_study"]) / 2
        centro_tob = (M[pre + "ankle_study"] + M[pre + "mankle_study"]) / 2
        izq_rod = -signo * (M[pre + "knee_study"] - M[pre + "mknee_study"])
        izq_tob = -signo * (M[pre + "ankle_study"] - M[pre + "mankle_study"])
        R["femur_" + lado] = _marco(np.cross(M[hjc] - centro_rod, izq_rod),
                                    izq_rod)
        R["tibia_" + lado] = _marco(np.cross(centro_rod - centro_tob, izq_tob),
                                    izq_tob)
    return R


def inclinacion(R):
    """Pitch y roll respecto de la gravedad, en el marco del propio segmento.

    Inmune al rumbo: la unica referencia es la vertical, asi que se puede
    comparar contra el IMU sin magnetometro (donde el yaw deriva ~1 deg/s).
    Gravedad de OpenSim = -Y.

    EL SIGNO -- corregido el 2026-09-10. Esta funcion devolvia el pitch y el
    roll con el signo OPUESTO al de las coordenadas del `.mot`, que es la
    convencion de OpenSim y la que usa `variedad.py`. Medido: ajustando
    `valor_del_.mot = a * valor_de_esta_funcion + b` sale

        cadena pelvis->pie   a = -1.18 -1.35 -1.18 -1.00 -1.03   (5 trials,
                                 dos sesiones)   r = -0.91 a -0.98
        pelvis_tilt          a = -0.97   pelvis_list  a = -1.19  (test5_60s)

    o sea un giro de signo puro, no un factor de escala. Antes de corregirlo
    la cadena salia con r NEGATIVO contra el `.mot` (-0.78 a -0.88) y
    POSITIVO contra los marcadores: era la misma cantidad en dos
    convenciones opuestas.

    No afecta a la reconstruccion: `variedad.py` arma la variedad enteramente
    desde el `.mot` (`hip_flexion - knee_angle + ankle_angle`) y nunca llama
    aca.
    """
    g = np.einsum("nji,j->ni", R, np.array([0.0, -1.0, 0.0]))
    return (-np.degrees(np.arctan2(g[:, 0], -g[:, 2])),
            -np.degrees(np.arctan2(g[:, 1], -g[:, 2])))


# ------------------------------------------------- velocidad angular
def velocidad_angular(R, fs=FS_OPENCAP):
    """omega [N,3] en rad/s (marco mundo), de una serie de matrices R [N,3,3].

    omega_x de dR/dt * R^T, que es antisimetrica. |omega| NO depende de la
    convencion del marco -- por eso sirve para sincronizar contra el IMU sin
    tener que acertarle a los ejes anatomicos.
    """
    dR = np.gradient(R, 1.0 / fs, axis=0)
    W = np.einsum("nij,nkj->nik", dR, R)
    return np.stack([W[:, 2, 1] - W[:, 1, 2],
                     W[:, 0, 2] - W[:, 2, 0],
                     W[:, 1, 0] - W[:, 0, 1]], axis=1) / 2.0


def angulos_relativos(R_prox, R_dist):
    """Angulos del segmento distal en el marco proximal (grados), misma
    descomposicion que `sparse6.angulos_relativos`."""
    Rrel = np.einsum("nij,njk->nik", np.transpose(R_prox, (0, 2, 1)), R_dist)
    sag = np.degrees(np.arctan2(Rrel[:, 0, 2], Rrel[:, 2, 2]))
    fro = np.degrees(np.arcsin(np.clip(-Rrel[:, 1, 2], -1.0, 1.0)))
    tra = np.degrees(np.arctan2(Rrel[:, 1, 0], Rrel[:, 1, 1]))
    return sag, fro, tra
