"""
pipeline.py -- corre el pipeline entero de una toma, en orden, con un comando.

    python pipeline.py testnorma2l9112026 --salida ../resultados/sesion_2026-09-11
    python pipeline.py test1normal9112026 --salida ../resultados/sesion_2026-09-11
    python pipeline.py <toma> --salida CARPETA --solo app          (solo la app)
    python pipeline.py <toma> --salida CARPETA --solo mot          (solo los .mot)

Deja en CARPETA tres subcarpetas:

    1_para_la_app/   el CSV recortado al tramo de marcha      (recortar_toma)
    2_para_opensim/  los tres .mot + el .sto                  (reconstruir_toma,
                                                              fusionar_mot x2,
                                                              analizar_toma)
    3_analisis/      figura y parametros de la app, la comparacion
                     de los tres .mot y la figura de sincronizacion
                                                             (parametros_marcha,
                                                              comparar_mot,
                                                              sincronizar)

Etapas, en orden (cada una es un script que tambien se puede correr solo):

  1. recortar_toma     CSV crudo -> CSV del tramo de marcha continua mas largo
  2. parametros_marcha lo que muestra la app: parametros MEDIDOS + figura
  3. analizar_toma     control de calidad por sensor + <toma>_orientations.sto
  4. reconstruir_toma  .mot SOLO IMU (cadena medida + variedad de marcha)
  5. fusionar_mot      .mot OpenCap + IMU (pierna y brazos)          [necesita par OpenCap]
  6. fusionar_mot      .mot OpenCap + IMU + pelvis del sacro         [necesita par OpenCap]
  7. comparar_mot      OpenCap vs +IMU vs +IMU+pelvis por ciclo      [necesita par OpenCap]
  8. sincronizar       figura del arranque por golpes de talon       [necesita par OpenCap]

La toma tiene que estar registrada (mapa de sensores en `mapa_sensores.py`;
sesion y par de OpenCap en `reconstruir_toma.py`): ver docs/MANUAL_PIPELINE.md §6.
"""
import argparse
import os
import subprocess
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import reconstruir_toma as R
import fusionar_mot as F
import validar_reconstruccion as VR


def correr(*args):
    cmd = [sys.executable, os.path.join(AQUI, args[0])] + list(args[1:])
    print("\n$ python " + " ".join(os.path.basename(a) if i == 0 else a
                                    for i, a in enumerate(cmd[1:])))
    subprocess.run(cmd, check=True, cwd=AQUI)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("toma", help="tag de la toma (test1normal9112026) o ruta al CSV crudo")
    p.add_argument("--salida", required=True,
                   help="carpeta raiz de salida; se crean 1_para_la_app, "
                        "2_para_opensim y 3_analisis adentro")
    p.add_argument("--solo", choices=["app", "mot", "comparacion"], default=None,
                   help="correr solo una parte")
    p.add_argument("--sin-recortar", action="store_true", dest="sin_recortar",
                   help="usar el CSV tal cual, sin quedarse con el tramo de marcha")
    a = p.parse_args()

    A = os.path.join(a.salida, "1_para_la_app")
    O = os.path.join(a.salida, "2_para_opensim")
    N = os.path.join(a.salida, "3_analisis")
    for d in (A, O, N):
        os.makedirs(d, exist_ok=True)

    # el CSV crudo: por tag (en las carpetas conocidas) o por ruta
    if os.path.isfile(a.toma):
        crudo = a.toma
    else:
        try:
            crudo = F.csv_crudo(a.toma)
        except SystemExit:
            crudo = R.buscar_csv(a.toma)
    tag = os.path.basename(crudo).split("_log_")[0]
    print("toma: %s\ncrudo: %s" % (tag, crudo))

    # 1. recorte
    if a.sin_recortar:
        csv_app = crudo
    else:
        correr("recortar_toma.py", crudo, "--tramo", "--salida", A)
        cand = [f for f in os.listdir(A) if f.startswith(tag) and f.endswith(".csv")]
        if not cand:
            raise SystemExit("recortar_toma no dejo ningun CSV en %s" % A)
        csv_app = os.path.join(A, sorted(cand)[0])

    todo = a.solo is None
    # 2-3. la app y el QC
    if todo or a.solo == "app":
        correr("parametros_marcha.py", csv_app, "--salida", N, "--figuras")
        correr("analizar_toma.py", csv_app, "--fig", O)

    # 4. solo IMU
    if todo or a.solo == "mot":
        correr("reconstruir_toma.py", csv_app, "--salida", O, "--sin-excluir")

    trial = R.PARES.get(tag)
    if not trial:
        print("\n%s no tiene par de OpenCap registrado en reconstruir_toma.PARES: "
              "sin fusion ni comparacion (ver docs/MANUAL_PIPELINE.md §6)" % tag)
        return

    # 5-6. OpenCap + IMU, con y sin pelvis (siempre desde el CSV CRUDO)
    if todo or a.solo == "mot":
        correr("fusionar_mot.py", tag, "--salida", O)
        correr("fusionar_mot.py", tag, "--salida", O,
               "--canales", "cadena,brazos,pelvis", "--sufijo", "_con_pelvis")
        # el .mot de OpenCap puro al lado de los otros dos, para cargarlos juntos
        import shutil
        _trc, mot_oc = VR.rutas_oc(tag, trial)
        osim = R.osim_de(tag)
        for src in (mot_oc, osim):
            if os.path.isfile(src):
                dst = os.path.join(O, os.path.basename(src))
                if not os.path.isfile(dst):
                    shutil.copy2(src, dst)
                    print("  copiado %s" % os.path.basename(src))

    # 7-8. comparacion y sincronizacion
    if todo or a.solo == "comparacion":
        correr("comparar_mot.py", tag, "--mot", O, "--salida", N)
        trc, _mot = VR.rutas_oc(tag, trial)
        correr("sincronizar.py", crudo, trc,
               os.path.join(N, tag + "_sincronizacion.png"))

    print("\nlisto. Salidas en %s" % os.path.abspath(a.salida))
    print("OpenSim: File > Open Model > %s ; File > Load Motion > 2_para_opensim/*.mot"
          % os.path.basename(R.osim_de(tag)))


if __name__ == "__main__":
    main()
