"""
acquire.py  --  v3 (2026-08-25), version de entrega del repositorio

USO EN LAS SESIONES (lo unico que hace falta saber para grabar):
    python3 acquire.py --mode check     # diagnostico del hardware
    python3 acquire.py --mode battery   # bateria de cada sensor
    python3 log.py caminata01 90        # graba 90 s en la memoria de los
                                        # sensores (modo log, 2+2+2) y descarga
Configuracion con la que se grabaron todas las sesiones con OpenCap:
modo log, 100 Hz, acelerometro + giroscopio, intervalo de conexion 30 ms,
tres adaptadores (Bluetooth integrado + 2 dongles) con dos sensores cada uno.
Lo que sigue describe la herramienta completa, que tambien sirve para las
pruebas de caracterizacion (ver ../docs/1_caracterizacion_plataforma.md).

Script unico para correr EN LA RASPBERRY. Cubre en una sola herramienta:
  - Prueba de conectividad (1..N sensores, integrado / dongle / split 3+3)
  - Prueba de packed mode (empaquetado BLE, para subir la fs efectiva)
  - Grabacion normal de una sesion (Nyquist, drift, sincronizacion)
  - Comparacion de dongles (--dongle etiqueta cada corrida)
  - Grabacion SIN enlace BLE, en la memoria del propio sensor (--mode log)
  - Diagnostico del hardware antes de empezar (--mode check)

Por que un solo script y no varios: todas esas pruebas son la misma
operacion (conectar N sensores a un adaptador, medir T segundos, registrar
Hz/perdida/bytes) con distintos parametros.


NOVEDADES DE LA v3 -- todas apuntan al problema abierto de la v2: los
enlaces se caian en menos de 20 s cuando los sensores se alejaban de la Pi
(ver figura 3 del informe del 21-ago). Son cuatro cambios independientes,
para poder medir cuanto aporta cada uno por separado:

  1. POTENCIA DE TRANSMISION DEL SENSOR (--tx-power, default +4 dBm).
     El firmware MetaWear arranca en 0 dBm y nunca se habia tocado. El
     nRF52 acepta +4 dBm, que son 4 dB mas de presupuesto de enlace: en
     espacio libre eso es ~1.6x de alcance, y en la practica es justo el
     margen que se pierde al atravesar el cuerpo. Valores validos:
     4, 0, -4, -8, -12, -16, -20, -30 (verificado en settings.h del SDK).

  2. SUPERVISION TIMEOUT (--supervision, default 16000 ms).
     Es el 4o parametro de mbl_mw_settings_set_connection_parameters y
     estaba en 6000 ms desde el codigo original. Es el tiempo que el
     enlace aguanta SIN recibir un solo paquete antes de darse por muerto.
     Con 6 s, cualquier sombra del cuerpo de mas de 6 segundos mata el
     enlace para siempre (la v2 no reconectaba). Subirlo a 16 s deja que
     el enlace sobreviva al desvanecimiento y siga solo cuando el sujeto
     vuelve a girar. El SDK permite hasta 32000 ms.

  3. RECONEXION AUTOMATICA (activada por defecto, --no-reconnect la apaga).
     Un hilo vigilante revisa cada pocos segundos si algun sensor perdio
     el enlace; si paso, lo reconecta, lo reconfigura y reanuda el
     streaming, sin tocar la corrida. En la v2 un sensor que se caia no
     volvia nunca. Se registran las caidas, las reconexiones y los
     segundos totales sin enlace.

  4. MODO LOG (--mode log): el sensor graba en su propia memoria flash y
     los datos se descargan al terminar. Durante el movimiento NO hay
     enlace BLE que perder, asi que la perdida de paquetes por distancia
     desaparece por completo. Es la unica opcion que garantiza 0% de
     perdida en campo; el precio es que no hay datos en vivo y que la
     descarga tarda. El codigo sale de los ejemplos log_acc.py /
     log_download.py del propio SDK.

  Extra: --rssi guarda la potencia recibida (dBm) de cada enlace una vez
  por segundo, para convertir "se cayo" en una curva de atenuacion real.
  Necesita sudo (usa el comando HCI Read_RSSI via hcitool).


CONFIGURACION ESTANDAR DEL PROYECTO (v3), todo por defecto:
  - split: 3 sensores al Bluetooth integrado + 3 al dongle
  - 100 Hz con packed mode
  - intervalo de conexion 30 ms, supervision 16 s, TX +4 dBm
  - reconexion automatica activada
  - 60 s por corrida

Por que 100 Hz con packed: verificado con datos propios el 21-ago. Sin
packed, a 100 Hz el giroscopio pierde entre 35% y 89% de las muestras; con
packed, los 6 sensores entregan ~100 Hz con menos de 2.5% de perdida.

Por que 30 ms de intervalo: 7.5 ms es el minimo del estandar BLE y viene de
una recomendacion de MbientLab pensada para UN sensor. Con varios sensores
en un mismo adaptador el controlador tiene que agendar cada conexion dentro
de la ventana del intervalo, y a 7.5 ms solo alcanza a mantener ~3. De ahi
sale el "limite de 3 conexiones por adaptador" que el articulo reporta como
limitacion del hardware: en realidad es consecuencia del propio parametro.

Uso rapido:
    python3 acquire.py --mode check                 # diagnostico, no conecta nada
    python3 acquire.py --tag prueba                 # sesion estandar 60 s
    python3 acquire.py --tag dongles --dongle B     # etiquetando el dongle
    python3 acquire.py --tag campo --mode log --duration 120
    python3 acquire.py --mode download              # rescatar un log que quedo en el sensor
"""
from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp
from mbientlab.metawear.cbindings import *
from ctypes import byref, cast, POINTER, c_void_p
from time import sleep, time
from datetime import datetime
import argparse
import csv
import os
import re
import subprocess
import sys
import threading

VERSION = "3.0 (2026-08-25)"

# MACs de los 6 nodos en uso (2026-08-19). El ORDEN importa: los 3 primeros
# van al Bluetooth integrado y los 3 ultimos al dongle, y ademas se conectan
# en este orden. Para cambiarlo sin editar el archivo, usa --macs o --rotate.
DEFAULT_MACS = [
    "F8:D1:BE:CA:3E:1B",
    "D7:C0:2D:7B:6F:C9",
    "DB:A1:10:94:0E:FD",   # reemplazo de EE:9D:A0:09:C6:11, que no grababa
    "C7:30:05:86:B0:6F",
    "F0:F3:13:F7:E0:B4",
    "FB:34:B5:D8:F3:4E",
]

# ODR validos en el sensor BMI160 (verificado en cbindings.py del SDK local).
ACC_ODR = {
    25: AccBmi160Odr._25Hz, 50: AccBmi160Odr._50Hz, 100: AccBmi160Odr._100Hz,
    200: AccBmi160Odr._200Hz, 400: AccBmi160Odr._400Hz,
    800: AccBmi160Odr._800Hz, 1600: AccBmi160Odr._1600Hz,
}
GYRO_ODR = {
    25: GyroBoschOdr._25Hz, 50: GyroBoschOdr._50Hz, 100: GyroBoschOdr._100Hz,
    200: GyroBoschOdr._200Hz, 400: GyroBoschOdr._400Hz,
    800: GyroBoschOdr._800Hz, 1600: GyroBoschOdr._1600Hz,
}

# Potencias de transmision que acepta el firmware (settings.h del SDK).
TX_POWERS = [4, 0, -4, -8, -12, -16, -20, -30]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

csv_lock = threading.Lock()
print_lock = threading.Lock()


def log(msg):
    """print() seguro entre hilos (el vigilante de reconexion tambien imprime)."""
    with print_lock:
        print(msg)
        sys.stdout.flush()


def run_cmd(cmd):
    """Ejecuta un comando y devuelve su stdout, o '' si no existe/falla."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


# ---------------------------------------------------------------- adaptadores

def get_adapters():
    """
    Lee hciconfig -a y devuelve {hci_name: {"mac":..., "bus":..., "role":...}}.

    IMPORTANTE: el numero de hci (hci0/hci1) NO es estable -- Linux lo asigna
    segun el orden de enumeracion al arrancar, y cambia con un reinicio o con
    un 'modprobe -r btusb'. El 20-ago se verifico que se habian invertido
    entre dos sesiones del mismo dia. Por eso NUNCA hay que identificar un
    adaptador por su numero de hci.

    Lo que si es estable es el tipo de bus:
      Bus: UART -> Bluetooth integrado de la Raspberry (chip Cypress)
      Bus: USB  -> dongle externo
    De ahi sale "role", que es lo que usa --adapter (onboard / dongle).
    """
    out = run_cmd(["hciconfig", "-a"])
    adapters = {}
    pattern = r"(hci\d+):\s+Type:\s+\S+\s+Bus:\s+(\S+).*?BD Address:\s+([0-9A-F:]{17})"
    for m in re.finditer(pattern, out, re.S):
        name, bus, mac = m.group(1), m.group(2), m.group(3)
        role = "onboard" if bus.upper() == "UART" else "dongle"
        adapters[name] = {"mac": mac, "bus": bus, "role": role}
    return adapters


def resolve_adapter(adapters, wanted):
    """Traduce onboard | dongle | hciX al hci real de esta corrida."""
    if wanted in ("onboard", "dongle"):
        matches = [n for n, a in adapters.items() if a["role"] == wanted]
        return matches[0] if matches else None
    return wanted if wanted in adapters else None


def adapter_is_up(hci_name):
    """
    True si hciconfig reporta el adaptador como "UP RUNNING".
    Sirve para detectar de antemano un adaptador en mal estado (ej. recien
    movido a otro puerto USB) ANTES de intentar conectar: si se intenta
    conectar con el adaptador caido, la libreria C++ del SDK (BLEEP) puede
    lanzar una excepcion C++ no capturable desde Python (terminate/abort),
    que tira todo el proceso -- esto evita llegar a ese punto.
    """
    return "UP RUNNING" in run_cmd(["hciconfig", hci_name])


def read_byte_counters():
    """{hci_name: (rx_bytes, tx_bytes)} leido de hciconfig."""
    out = run_cmd(["hciconfig"])
    counters = {}
    for m in re.finditer(r"(hci\d+):.*?RX bytes:(\d+).*?TX bytes:(\d+)", out, re.S):
        counters[m.group(1)] = (int(m.group(2)), int(m.group(3)))
    return counters


# ---------------------------------------------------------------------- RSSI

def is_root():
    return getattr(os, "geteuid", lambda: 0)() == 0


def hci_connections(hci_name):
    """{mac: connection_handle} de las conexiones LE abiertas en ese adaptador."""
    out = run_cmd(["hcitool", "-i", hci_name, "con"])
    conns = {}
    for m in re.finditer(r"[<>]\s+LE\s+([0-9A-F:]{17})\s+handle\s+(\d+)", out):
        conns[m.group(1).upper()] = int(m.group(2))
    return conns


def read_rssi(hci_name, handle):
    """
    RSSI absoluto en dBm de un enlace LE, via el comando HCI Read_RSSI
    (ogf 0x05, ocf 0x0005). BlueZ no expone el RSSI de una conexion LE por
    D-Bus, asi que se pide directo al controlador. Necesita root.
    Devuelve None si no se pudo leer.
    """
    lo, hi = handle & 0xFF, (handle >> 8) & 0xFF
    out = run_cmd(["hcitool", "-i", hci_name, "cmd", "0x05", "0x0005",
                   "0x%02x" % lo, "0x%02x" % hi])
    m = re.search(r"HCI Event: 0x0e[^\n]*\n((?:\s+[0-9A-Fa-f]{2})+)", out)
    if not m:
        return None
    data = [int(b, 16) for b in m.group(1).split()]
    # num_pkts(1) opcode(2) status(1) handle(2) rssi(1)
    if len(data) < 7 or data[3] != 0:
        return None
    rssi = data[6]
    return rssi - 256 if rssi > 127 else rssi


class RssiSampler(threading.Thread):
    """Guarda el RSSI de cada enlace una vez por segundo en un CSV aparte."""

    def __init__(self, states, path, period, stop_event):
        threading.Thread.__init__(self)
        self.daemon = True
        self.states = states
        self.path = path
        self.period = period
        self.stop_event = stop_event
        self.samples = 0

    def run(self):
        with open(self.path, "w", newline="", buffering=1) as f:
            w = csv.writer(f)
            w.writerow(["timestamp_host", "mac", "hci", "rssi_dbm"])
            while not self.stop_event.wait(self.period):
                por_hci = {}
                for st in self.states:
                    if st.hci_name not in por_hci:
                        por_hci[st.hci_name] = hci_connections(st.hci_name)
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                for st in self.states:
                    handle = por_hci[st.hci_name].get(st.device.address.upper())
                    valor = read_rssi(st.hci_name, handle) if handle is not None else None
                    w.writerow([ts, st.device.address, st.hci_name,
                                "" if valor is None else valor])
                    if valor is not None:
                        self.samples += 1


# --------------------------------------------------------------------- estado

class State:
    """Estado de un sensor: callbacks acc/gyro, cada uno escribe su propia fila."""

    def __init__(self, device, writer, hci_name, role, chip="bmi160"):
        self.device = device
        self.writer = writer
        self.hci_name = hci_name
        self.role = role
        self.chip = chip
        self.acc_samples = 0
        self.gyro_samples = 0
        self.is_streaming = False
        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_callback = FnVoid_VoidP_DataP(self.gyro_data_handler)
        self.acc_signal = None
        self.gyro_signal = None
        self.logger_acc = None
        self.logger_gyro = None
        # Contabilidad del enlace. Sin esto, un sensor que pierde el enlace a
        # mitad de sesion es indistinguible de uno que simplemente dejo de
        # mandar: ambos aparecen como perdida alta.
        self.drops = 0            # cuantas veces se cayo el enlace
        self.reconnects = 0       # cuantas veces se recupero
        self.first_drop_at = None  # instante de la primera caida (para el informe)
        self.down_since = None     # instante de la caida actual, None si esta arriba
        self.downtime = 0.0        # segundos acumulados sin datos
        self.last_attempt = 0.0
        self.tearing_down = False  # True durante el cierre: las caidas no cuentan
        device.on_disconnect = self._on_disconnect

    def _on_disconnect(self, status):
        if self.tearing_down:
            return
        self.is_streaming = False
        # Si ya sabiamos que estaba caido, este evento es el disconnect()
        # que hace el propio vigilante antes de reintentar, no una caida
        # nueva. Contarlo inflaria el numero de caidas del informe.
        if self.down_since is not None:
            return
        now = time()
        self.drops += 1
        if self.first_drop_at is None:
            self.first_drop_at = now
        self.down_since = now

    def mark_up(self):
        if self.down_since is not None:
            self.downtime += time() - self.down_since
            self.down_since = None

    def adopt(self, device):
        """
        Reemplaza el objeto MetaWear por uno nuevo conservando los contadores.
        Se usa cuando reconectar sobre el objeto viejo falla y hay que
        empezar de cero: los callbacks del State siguen siendo los mismos,
        solo cambia a que board apuntan.
        """
        self.device = device
        device.on_disconnect = self._on_disconnect
        self.acc_signal = None
        self.gyro_signal = None

    def _write_row(self, kind, x, y, z, epoch):
        with csv_lock:
            self.writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                self.device.address, kind, x, y, z, epoch,
            ])

    def acc_data_handler(self, ctx, data):
        if self.is_streaming:
            v = parse_value(data)
            self.acc_samples += 1
            self._write_row("acc", v.x, v.y, v.z, data.contents.epoch)

    def gyro_data_handler(self, ctx, data):
        if self.is_streaming:
            v = parse_value(data)
            self.gyro_samples += 1
            self._write_row("gyro", v.x, v.y, v.z, data.contents.epoch)

    def start_streaming(self):
        self.is_streaming = True
        board, g = self.device.board, gyro_api(self.chip)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(board)
        libmetawear.mbl_mw_acc_start(board)
        g["enable_rotation_sampling"](board)
        g["start"](board)

    def stop_streaming(self):
        self.is_streaming = False
        board, g = self.device.board, gyro_api(self.chip)
        libmetawear.mbl_mw_acc_stop(board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(board)
        g["stop"](board)
        g["disable_rotation_sampling"](board)


# ------------------------------------------------------------- configuracion

def set_link_params(board, conn_interval, supervision, tx_power, settle=0.0):
    """
    Parametros de radio y de enlace. Se aplican ANTES que la configuracion de
    los sensores porque cambiar el intervalo de conexion dispara una
    renegociacion del enlace BLE, y escribir la configuracion mientras esa
    renegociacion sigue en curso hace que el SDK falle con "Error trying to
    issue command mid state" (una operacion GATT lanzada mientras otra sigue
    pendiente). Cuando eso pasa el sensor queda conectado pero SIN configurar:
    nunca transmite, y aparece como 100% de perdida.

    settle: segundos de espera despues de la renegociacion. Cuando se
    configuran varios sensores se pasa 0 aqui y se espera UNA sola vez al
    final, en vez de una vez por sensor (con 6 sensores eso son 18 s de
    arranque que en corridas de 30-60 s pesaban mas que la propia medicion).
    """
    if tx_power is not None and existe("mbl_mw_settings_set_tx_power"):
        libmetawear.mbl_mw_settings_set_tx_power(board, int(tx_power))
        sleep(0.2)
    libmetawear.mbl_mw_settings_set_connection_parameters(
        board, conn_interval, conn_interval, 0, int(supervision))
    if settle:
        sleep(settle)


# --------------------------------------------------------- chip del sensor

# Los MetaMotionRL de este proyecto llevan un BMI160. Otros modelos de la
# misma familia (MetaMotionC, MetaMotionS) llevan un BMI270, y la API del
# giroscopio cambia de nombre entera: mbl_mw_gyro_bmi160_* vs bmi270_*.
# Llamar a la del chip equivocado NO da error -- el sensor conecta, nunca
# transmite, y aparece como 100% de perdida. Ese sintoma es carisimo de
# diagnosticar, asi que el chip se detecta y se elige la API sola.
GYRO_FUNCS = ("set_range", "set_odr", "write_config",
              "enable_rotation_sampling", "disable_rotation_sampling",
              "start", "stop",
              "get_rotation_data_signal", "get_packed_rotation_data_signal")

_api_cache = {}
_avisados = set()   # avisos que solo tienen sentido una vez por corrida


def existe(nombre):
    """
    True si la libreria instalada tiene esa funcion.

    No todas las versiones del SDK traen lo mismo, y la que esta instalada
    en la Raspberry es mas vieja que la del repositorio. Preguntar antes de
    llamar evita el AttributeError a mitad de una corrida.
    """
    try:
        getattr(libmetawear, nombre)
        return True
    except AttributeError:
        return False


def gyro_api(chip):
    """{nombre: funcion} del giroscopio para ese chip."""
    if chip not in _api_cache:
        _api_cache[chip] = {
            n: getattr(libmetawear, "mbl_mw_gyro_%s_%s" % (chip, n))
            for n in GYRO_FUNCS
        }
    return _api_cache[chip]


def chip_soportado(chip):
    """True si la libreria instalada tiene toda la API de ese chip."""
    return (existe("mbl_mw_acc_%s_set_odr" % chip)
            and all(existe("mbl_mw_gyro_%s_%s" % (chip, n)) for n in GYRO_FUNCS))


def detectar_chip(board):
    """
    'bmi160' o 'bmi270'.

    Si no se puede averiguar devuelve 'bmi160', que es lo que llevan los
    MetaMotionRL de este proyecto y lo que usaba el codigo anterior sin
    preguntar: la deteccion es una ayuda para no romperse con otros modelos,
    NUNCA un motivo para que una corrida no arranque.

    Todo se consulta con getattr y valores por defecto porque las constantes
    y las funciones cambian entre versiones del SDK -- la del repositorio
    tiene MODULE_GYRO_TYPE_BMI160, la instalada en la Pi no.
    """
    try:
        tipo = libmetawear.mbl_mw_metawearboard_lookup_module(
            board, getattr(Module, "GYRO", 19))
        es_270 = tipo == getattr(Const, "MODULE_GYRO_TYPE_BMI270", 1)
    except Exception:
        return "bmi160"
    if es_270:
        if chip_soportado("bmi270"):
            return "bmi270"
        if "bmi270" not in _avisados:
            _avisados.add("bmi270")
            log("AVISO: este sensor parece llevar un BMI270, pero el SDK "
                "instalado no trae su API. Se lo trata como BMI160; si no "
                "transmite nada, la causa es esta.")
    return "bmi160"


def nombre_modelo(board):
    try:
        nombre = libmetawear.mbl_mw_metawearboard_get_model_name(board)
        return nombre.decode() if nombre else "?"
    except Exception:
        return "?"


def leer_bateria(board, timeout=3.0):
    """
    Carga de la bateria en % y voltaje en mV, o None si no contesta.
    Vale la pena leerla al conectar: un sensor con la bateria baja anuncia
    mas debil, conecta peor y se cae antes, y eso se confunde facil con un
    problema de alcance o de dongle.
    """
    try:
        signal = libmetawear.mbl_mw_settings_get_battery_state_data_signal(board)
    except Exception:
        return None
    if not signal:
        return None
    listo = threading.Event()
    resultado = {}

    def cb(ctx, ptr):
        try:
            v = parse_value(ptr)
            resultado["pct"] = int(v.charge)
            resultado["mv"] = int(v.voltage)
        except Exception:
            pass
        listo.set()

    fn = FnVoid_VoidP_DataP(cb)
    try:
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, fn)
        libmetawear.mbl_mw_datasignal_read(signal)
        ok = listo.wait(timeout)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
    except Exception:
        return None
    return resultado if ok and resultado else None


def set_sensor_config(board, odr, chip):
    """ODR y rangos. Los valores son los mismos desde la v1, no se tocan."""
    getattr(libmetawear, "mbl_mw_acc_%s_set_odr" % chip)(board, ACC_ODR[odr])
    libmetawear.mbl_mw_acc_bosch_set_range(board, AccBoschRange._4G)
    libmetawear.mbl_mw_acc_write_acceleration_config(board)
    g = gyro_api(chip)
    g["set_range"](board, GyroBoschRange._1000dps)
    g["set_odr"](board, GYRO_ODR[odr])
    g["write_config"](board)


def get_signals(board, packed, chip):
    g = gyro_api(chip)
    if packed:
        acc = libmetawear.mbl_mw_acc_get_packed_acceleration_data_signal(board)
        gyro = g["get_packed_rotation_data_signal"](board)
    else:
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(board)
        gyro = g["get_rotation_data_signal"](board)
    return acc, gyro


def subscribe_stream(state, packed):
    board = state.device.board
    acc, gyro = get_signals(board, packed, state.chip)
    state.acc_signal, state.gyro_signal = acc, gyro
    libmetawear.mbl_mw_datasignal_subscribe(acc, None, state.acc_callback)
    libmetawear.mbl_mw_datasignal_subscribe(gyro, None, state.gyro_callback)


def unsubscribe_stream(state):
    if state.acc_signal:
        libmetawear.mbl_mw_datasignal_unsubscribe(state.acc_signal)
    if state.gyro_signal:
        libmetawear.mbl_mw_datasignal_unsubscribe(state.gyro_signal)


def cerrar_enlace_colgado(hci_name, mac):
    """
    Cierra una conexion LE que quedo abierta hacia ese sensor.

    Modo de falla clasico: un intento de conexion muere a mitad de camino y
    del lado de BlueZ el enlace queda abierto. El sensor figura como
    conectado, no acepta una conexion nueva, y TODOS los reintentos fallan
    igual por mas que se insista -- que es exactamente lo que se ve cuando
    un sensor agota los 15 intentos con el mismo error. Cerrarlo a mano
    desde el controlador desatasca eso. Necesita root.
    """
    handle = hci_connections(hci_name).get(mac.upper())
    if handle is None:
        return False
    if not is_root():
        log("..  %s figura conectado en %s (handle %d) pero no se puede cerrar "
            "sin sudo. A mano: sudo hcitool -i %s ledc %d"
            % (mac, hci_name, handle, hci_name, handle))
        return False
    log("..  cerrando enlace colgado de %s en %s (handle %d)"
        % (mac, hci_name, handle))
    run_cmd(["hcitool", "-i", hci_name, "ledc", str(handle)])
    sleep(1.0)
    return True


def reiniciar_adaptador(hci_name):
    """
    Baja y sube el adaptador. Es lo que uno termina haciendo a mano cuando
    BlueZ se queda pegado y todos los intentos fallan con el mismo error;
    hacerlo automatico ahorra tener que abortar la corrida. Necesita root.
    """
    if not is_root():
        log("..  convendria reiniciar %s pero hace falta sudo. A mano: "
            "sudo hciconfig %s down && sudo hciconfig %s up"
            % (hci_name, hci_name, hci_name))
        return False
    log("..  reiniciando %s" % hci_name)
    run_cmd(["hciconfig", hci_name, "down"])
    sleep(1.5)
    run_cmd(["hciconfig", hci_name, "up"])
    sleep(2.5)
    return adapter_is_up(hci_name)


def connect_device(mac, hci_addr, retries, pause=5.0, quiet=False,
                   hci_name=None, reset_cada=5):
    """
    Conecta un sensor reintentando. Devuelve (MetaWear, intento) o (None, N).
    Los fallos son transitorios, asi que insistir funciona: el script
    original que SI lograba conectar los 6 (control3.py) usaba 20 intentos.

    Insistir a secas no alcanza cuando lo que fallo es el estado del
    adaptador o un enlace que quedo colgado: ahi los 15 intentos dan el
    mismo error 15 veces. Por eso antes de cada reintento se cierra el
    enlace colgado si lo hay, y cada `reset_cada` intentos se reinicia el
    adaptador. Ambas cosas necesitan root; sin root se avisa que comando
    correr a mano y se sigue reintentando igual.
    """
    for attempt in range(1, retries + 1):
        if hci_name and attempt > 1:
            cerrar_enlace_colgado(hci_name, mac)
        device = None
        try:
            device = MetaWear(mac, hci_mac=hci_addr)
            device.connect()
            return device, attempt
        except Exception as e:
            if not quiet:
                log("--  %s: %s (intento %d/%d)" % (mac, e, attempt, retries))
            # Un intento fallido puede dejar el enlace a medio abrir (la firma
            # tipica es el "status = 2" del siguiente intento). Cerrarlo
            # explicitamente antes de reintentar evita arrastrar ese estado.
            if device is not None:
                try:
                    device.disconnect()
                except Exception:
                    pass
            if attempt >= retries:
                break
            if hci_name and reset_cada and attempt % reset_cada == 0:
                reiniciar_adaptador(hci_name)
            sleep(pause)
    return None, retries


def reabrir_enlace(state, hci_addr):
    """
    Vuelve a levantar el enlace de un sensor que se cayo. Devuelve True/False.

    Primero reintenta sobre el mismo objeto MetaWear, que es lo barato:
    device.connect() re-inicializa la board sobre el mismo puntero y los
    callbacks del State siguen sirviendo. Si eso falla -- porque warble dejo
    el Gatt anterior en un estado del que no se recupera y entonces falla
    siempre igual -- se crea un objeto nuevo. El estado del sensor se lee de
    la cache local (.metawear), asi que arrancar de cero no cuesta mucho mas.
    """
    try:
        state.device.disconnect()
    except Exception:
        pass
    try:
        state.device.connect()
        return True
    except Exception as primera:
        motivo = primera
    try:
        nuevo = MetaWear(state.device.address, hci_mac=hci_addr)
        nuevo.connect()
    except Exception:
        log("..  reconexion fallida %s: %s" % (state.device.address, motivo))
        return False
    state.adopt(nuevo)
    return True


# ------------------------------------------------------------- reconexion

class Reconnector(threading.Thread):
    """
    Vigila los enlaces durante la corrida y reconecta los que se caen.

    Por que un hilo aparte y no reconectar dentro del callback de
    desconexion: ese callback lo dispara la libreria C++ desde su propio
    hilo interno, y lanzar ahi una reconexion (que a su vez llama al mismo
    stack de warble/BlueZ) es la receta para un deadlock o un segfault. El
    callback solo marca la caida; este hilo hace el trabajo.

    La reconexion en si la hace reabrir_enlace(): primero sobre el mismo
    objeto MetaWear (lo barato) y, si eso no sale, con uno nuevo. Despues
    hay que volver a pedir las senales y suscribirse, porque la board se
    re-inicializa y los punteros viejos ya no valen.
    """

    def __init__(self, states, adapters, packed, conn_interval, supervision,
                 tx_power, odr, period, stop_event):
        threading.Thread.__init__(self)
        self.daemon = True
        self.states = states
        self.adapters = adapters
        self.packed = packed
        self.conn_interval = conn_interval
        self.supervision = supervision
        self.tx_power = tx_power
        self.odr = odr
        self.period = period
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.wait(self.period):
            if self.stop_event.is_set():
                return
            caidos = [st for st in self.states
                      if not st.tearing_down and not self._caido_hace_rato(st)]
            if caidos:
                self._recuperar(caidos)

    def _caido_hace_rato(self, st):
        """True si el sensor esta bien o si todavia no toca reintentarlo."""
        try:
            if st.device.is_connected:
                return True
        except Exception:
            return True
        return time() - st.last_attempt < self.period

    def _recuperar(self, caidos):
        """
        Recupera VARIOS sensores a la vez, en dos pasadas.

        Por que en dos pasadas y no uno por uno: cambiar el intervalo de
        conexion obliga a esperar ~3 s a que la renegociacion del enlace
        asiente antes de poder escribir la configuracion del sensor (si no,
        el SDK falla con "command mid state" y el sensor queda conectado
        pero mudo). Haciendo un sensor completo por vez, recuperar los 6
        despues de un desvanecimiento tardaria mas de 20 s. Reconectando
        todos primero y esperando UNA sola vez, tarda poco mas de 3.
        """
        ahora = time()
        for st in caidos:
            st.last_attempt = ahora
            st.is_streaming = False

        abiertos = []
        for st in caidos:
            if self.stop_event.is_set():
                return
            if reabrir_enlace(st, self.adapters[st.hci_name]["mac"]):
                abiertos.append(st)
        if not abiertos:
            return

        vivos = []
        for st in abiertos:
            try:
                set_link_params(st.device.board, self.conn_interval,
                                self.supervision, self.tx_power, settle=0.0)
                vivos.append(st)
            except Exception as e:
                log("..  %s reconecto pero fallo al fijar el enlace: %s"
                    % (st.device.address, e))
        if not vivos or self.stop_event.wait(3.5):
            return

        for st in vivos:
            if st.tearing_down or self.stop_event.is_set():
                return
            try:
                set_sensor_config(st.device.board, self.odr, st.chip)
                subscribe_stream(st, self.packed)
                st.start_streaming()
            except Exception as e:
                log("..  %s reconecto pero fallo al reconfigurar: %s"
                    % (st.device.address, e))
                continue
            sin_datos = time() - st.down_since if st.down_since else 0.0
            st.reconnects += 1
            st.mark_up()
            log("++  %s reconectado (%.1f s sin datos, reconexion #%d)"
                % (st.device.address, sin_datos, st.reconnects))


# ------------------------------------------------------------- modo check

def modo_check():
    """
    Diagnostico del hardware antes de una sesion. No conecta ningun sensor.
    Revisa las tres cosas que en las sesiones anteriores rompieron corridas
    enteras: el dongle en un puerto USB 3.0, un adaptador caido, y el WiFi
    de 2.4 GHz robandole aire al Bluetooth integrado.
    """
    print("acquire.py v%s -- diagnostico\n" % VERSION)
    problemas = []

    adapters = get_adapters()
    print("== Adaptadores Bluetooth ==")
    if not adapters:
        print("  ninguno (hciconfig vacio)")
        problemas.append("No se detecto ningun adaptador Bluetooth.")
    for name, a in sorted(adapters.items()):
        estado = "UP RUNNING" if adapter_is_up(name) else "CAIDO"
        print("  %s: %s  bus=%-5s -> %-7s  %s"
              % (name, a["mac"], a["bus"], a["role"], estado))
        if estado != "UP RUNNING":
            problemas.append("%s no esta UP RUNNING. Prueba: sudo hciconfig %s up"
                             % (name, name))
    roles = {a["role"] for a in adapters.values()}
    if "dongle" not in roles:
        problemas.append("No hay dongle USB conectado: el modo split no va a correr.")
    if "onboard" not in roles:
        problemas.append("No se ve el Bluetooth integrado (bus UART) de la Raspberry.")

    print("\n== Puerto USB del dongle ==")
    salida = run_cmd(["lsusb", "-t"])
    if not salida:
        print("  lsusb no disponible; revisalo a mano.")
    else:
        velocidad_bus = {}
        bus_actual = None
        for linea in salida.splitlines():
            m = re.match(r"/:\s+Bus\s+(\d+)", linea)
            if m:
                bus_actual = m.group(1)
                vm = re.search(r"(\d+)M\s*$", linea.strip())
                velocidad_bus[bus_actual] = int(vm.group(1)) if vm else None
            if "btusb" in linea:
                vm = re.search(r"(\d+)M\s*$", linea.strip())
                vel_disp = int(vm.group(1)) if vm else None
                vel_raiz = velocidad_bus.get(bus_actual)
                print("  dongle en bus %s (raiz %s Mbps, dispositivo %s Mbps)"
                      % (bus_actual, vel_raiz, vel_disp))
                if vel_raiz and vel_raiz >= 5000:
                    problemas.append(
                        "El dongle esta en un puerto USB 3.0 (los azules). En la "
                        "Raspberry Pi 4B el controlador USB 3.0 irradia justo en "
                        "2.4 GHz y degrada el BLE. Pasalo a un puerto USB 2.0 (negro).")
                else:
                    print("  OK: puerto USB 2.0")

    print("\n== WiFi ==")
    freq = None
    for iface in ("wlan0", "wlan1"):
        enlace = run_cmd(["iw", "dev", iface, "link"])
        if "Connected" in enlace:
            m = re.search(r"freq:\s*(\d+)", enlace)
            freq = int(m.group(1)) if m else None
            print("  %s conectado, freq %s MHz" % (iface, freq))
            if freq and freq < 3000:
                problemas.append(
                    "El WiFi esta en 2.4 GHz. En la Raspberry Pi 4B el WiFi y el "
                    "Bluetooth INTEGRADO comparten el mismo chip y la misma antena, "
                    "asi que el WiFi de 2.4 GHz le roba aire justo a los 3 sensores "
                    "del adaptador integrado.\n"
                    "     Mejor: cable Ethernet, o pasar el WiFi a 5 GHz.\n"
                    "     Como ultimo recurso, bajarlo -- pero como es el mismo chip, "
                    "bajar wlan0 puede dejar tocado el hci integrado y entonces esos "
                    "3 sensores no conectan. Si lo bajas, recicla el integrado "
                    "despues:\n"
                    "       sudo ifconfig wlan0 down\n"
                    "       sudo hciconfig hci0 down && sudo hciconfig hci0 up")
            elif freq:
                print("  OK: 5 GHz, no interfiere con el Bluetooth integrado")
    if freq is None:
        print("  sin conexion WiFi activa (ideal para las pruebas)")

    # Las funciones que no todas las versiones del SDK traen. El script sabe
    # arreglarselas sin ellas, pero conviene saber cuales faltan antes de
    # preguntarse por que no aparece el modelo o la bateria de un sensor.
    print("\n== SDK instalado ==")
    opcionales = [
        ("mbl_mw_settings_set_tx_power", "subir la potencia del sensor a +4 dBm"),
        ("mbl_mw_metawearboard_lookup_module", "detectar el chip (BMI160/BMI270)"),
        ("mbl_mw_metawearboard_get_model_name", "mostrar el modelo del sensor"),
        ("mbl_mw_settings_get_battery_state_data_signal", "leer la bateria"),
        ("mbl_mw_metawearboard_tear_down", "limpiar loggers viejos en --mode log"),
    ]
    faltan_opc = False
    for nombre, para_que in opcionales:
        hay = existe(nombre)
        print("  %-3s %s" % ("si" if hay else "NO", para_que))
        faltan_opc = faltan_opc or not hay
    if faltan_opc:
        print("  (lo que falta se saltea solo, no rompe la corrida)")

    print("\n== Resultado ==")
    if not problemas:
        print("  Todo en orden. Podes correr las pruebas.")
        return 0
    for i, p in enumerate(problemas, 1):
        print("  %d. %s" % (i, p))
    return 1


def escanear(hci_name, segundos):
    """
    Escucha los anuncios BLE con un adaptador. Devuelve {mac: nombre}.

    Usa 'hcitool lescan', que corre hasta que lo matan: se lanza y se le
    manda terminate a los N segundos. Necesita root.
    """
    vistos = {}
    try:
        proc = subprocess.Popen(["hcitool", "-i", hci_name, "lescan",
                                 "--duplicates"],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True)
    except OSError:
        return None
    try:
        proc.wait(timeout=segundos)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    salida, err = proc.communicate()
    if not salida and err and ("Set scan parameters failed" in err
                               or "Operation not permitted" in err):
        return None
    for linea in (salida or "").splitlines():
        partes = linea.strip().split(" ", 1)
        if len(partes) == 2 and re.fullmatch(r"([0-9A-F]{2}:){5}[0-9A-F]{2}",
                                             partes[0].upper()):
            mac = partes[0].upper()
            nombre = partes[1].strip()
            if mac not in vistos or vistos[mac] in ("(unknown)", ""):
                vistos[mac] = nombre
    return vistos


def modo_scan(macs, segundos):
    """
    Responde una sola pregunta: la Pi ve los sensores?

    Cuando un sensor no conecta hay tres culpables posibles -- el sensor
    (apagado, sin bateria, fuera de rango), el adaptador (pegado, caido) o
    el enlace. Escanear los separa sin conectar nada: si el sensor aparece
    en el escaneo, esta prendido y en rango y el problema es de conexion;
    si no aparece con NINGUN adaptador, el problema es el sensor; si
    aparece con uno y no con el otro, el problema es ese adaptador.
    """
    print("acquire.py v%s -- escaneo\n" % VERSION)
    if not is_root():
        print("El escaneo necesita root. Corre:  sudo python3 acquire.py --mode scan")
        return 1

    adapters = get_adapters()
    if not adapters:
        print("No se detectaron adaptadores Bluetooth. Abortando.")
        return 1

    nombres = sorted(adapters.keys(),
                     key=lambda n: (adapters[n]["role"] != "onboard", n))
    resultados = {}
    for name in nombres:
        rol = adapters[name]["role"]
        print("Escaneando %s s con %s (%s)..." % (segundos, name, rol))
        vistos = escanear(name, segundos)
        if vistos is None:
            print("  no se pudo escanear con %s (hcitool fallo o no esta)" % name)
            vistos = {}
        resultados[name] = vistos
        print("  %d dispositivos BLE a la vista" % len(vistos))

    print("\nLos 6 sensores del proyecto:")
    print("%-20s %s" % ("sensor", "  ".join("%-14s" % n for n in nombres)))
    perdidos = []
    for mac in macs:
        celdas = []
        visto_en_alguno = False
        for name in nombres:
            if mac in resultados[name]:
                celdas.append("%-14s" % "SI")
                visto_en_alguno = True
            else:
                celdas.append("%-14s" % "-")
        print("%-20s %s" % (mac, "  ".join(celdas)))
        if not visto_en_alguno:
            perdidos.append(mac)

    print()
    if not perdidos:
        print("Todos los sensores estan prendidos y a la vista. Si igual no")
        print("conectan, el problema es del enlace o del adaptador, no del")
        print("sensor: reinicia el adaptador y volve a intentar.")
        return 0

    print("NO se ven con ningun adaptador: %s" % ", ".join(perdidos))
    print("Eso apunta al sensor, no a la Pi: revisa que esten prendidos (LED),")
    print("que tengan bateria, y que no hayan quedado conectados a otro equipo")
    print("(un celular con la app de MbientLab abierta se los queda).")
    for name in nombres:
        if not resultados[name]:
            print("Ademas, %s no vio NADA: ese adaptador puede estar pegado ->"
                  % name)
            print("  sudo hciconfig %s down && sudo hciconfig %s up" % (name, name))
    return 1


def modo_bateria(macs, adapters, adapter_for, retries):
    """
    Conecta a cada sensor, lee bateria y modelo, y se desconecta.

    Existe porque "no conecta" y "tiene la bateria en el 8%" se parecen
    mucho desde afuera, y distinguirlos a ojo cuesta media sesion. Tambien
    sirve como chequeo de rutina antes de empezar: seis sensores cargados
    es una condicion de largada, no un detalle.
    """
    print("acquire.py v%s -- bateria\n" % VERSION)
    print("%-20s %-16s %-9s %-8s %s" % ("sensor", "modelo", "chip", "bateria", "mV"))
    flojos, mudos = [], []
    for i, mac in enumerate(macs):
        hci_name, hci_addr = adapter_for(i)
        device, _ = connect_device(mac, hci_addr, retries, quiet=True,
                                   hci_name=hci_name)
        if device is None:
            print("%-20s %s" % (mac, "NO CONECTO"))
            mudos.append(mac)
            continue
        chip = detectar_chip(device.board) or "?"
        modelo = nombre_modelo(device.board)
        bat = leer_bateria(device.board)
        if bat is None:
            print("%-20s %-16s %-9s %s" % (mac, modelo, chip, "no contesto"))
        else:
            marca = "  <-- BAJA" if bat["pct"] <= 20 else ""
            print("%-20s %-16s %-9s %-8s %d%s"
                  % (mac, modelo, chip, "%d%%" % bat["pct"], bat["mv"], marca))
            if bat["pct"] <= 20:
                flojos.append(mac)
        try:
            libmetawear.mbl_mw_debug_disconnect(device.board)
        except Exception:
            pass
        sleep(1.0)

    print()
    if flojos:
        print("Con la bateria baja: %s" % ", ".join(flojos))
        print("Un sensor asi anuncia mas debil, conecta peor y se cae antes.")
        print("Cargalos antes de medir o los datos no van a ser comparables.")
    if mudos:
        print("No conectaron: %s" % ", ".join(mudos))
        print("Proba primero:  sudo python3 acquire.py --mode scan")
    if not flojos and not mudos:
        print("Los %d sensores respondieron y ninguno esta con la bateria baja."
              % len(macs))
    return 1 if (flojos or mudos) else 0


# ------------------------------------------------------------- modo log

class LogDownload:
    """Recibe las muestras descargadas de la memoria del sensor."""

    def __init__(self, mac, writer):
        self.mac = mac
        self.writer = writer
        self.acc = 0
        self.gyro = 0
        self.done = threading.Event()
        self.handlers = []

    def make_handler(self, kind):
        def cb(ctx, ptr):
            v = parse_value(ptr)
            epoch = ptr.contents.epoch
            # timestamp_host se sintetiza a partir del epoch del propio sensor
            # para que los scripts de analisis (que ordenan por esa columna)
            # sigan funcionando igual con datos descargados.
            ts = datetime.fromtimestamp(epoch / 1000.0).strftime("%Y-%m-%d %H:%M:%S.%f")
            with csv_lock:
                self.writer.writerow([ts, self.mac, kind, v.x, v.y, v.z, epoch])
            if kind == "acc":
                self.acc += 1
            else:
                self.gyro += 1
        wrapped = FnVoid_VoidP_DataP(cb)
        self.handlers.append(wrapped)  # evita que el GC se lo lleve
        return wrapped


def parar_de_grabar(board, chip):
    """
    Corta la grabacion del sensor. Cada paso va por separado a proposito:
    si uno falla ("Error trying to issue command mid state" es habitual
    cuando el enlace viene golpeado), los demas se mandan igual. Dejar un
    sensor grabando es peor que cualquiera de esos errores.
    """
    for accion in (
            lambda: libmetawear.mbl_mw_logging_stop(board),
            lambda: libmetawear.mbl_mw_acc_stop(board),
            lambda: libmetawear.mbl_mw_acc_disable_acceleration_sampling(board),
            lambda: gyro_api(chip)["stop"](board),
            lambda: gyro_api(chip)["disable_rotation_sampling"](board),
            lambda: libmetawear.mbl_mw_logging_flush_page(board)
            if existe("mbl_mw_logging_flush_page") else None):
        try:
            accion()
            sleep(0.15)
        except Exception:
            pass


def descargar_log(device, writer, mac, n_notifies=10):
    """
    Descarga lo que el sensor tenga guardado, usando "anonymous datasignals".

    Por que anonymous y no los punteros MblMwDataLogger creados al inicio:
    si el enlace se cayo durante el paseo (que es justo lo que este modo
    esta pensado para tolerar), al reconectar la board se re-inicializa y
    esos punteros ya no sirven. Los anonymous datasignals se descubren
    leyendo la configuracion que quedo grabada en el propio sensor, asi que
    funcionan igual si el proceso se reinicio o si el sensor se reconecto.
    Es el mismo camino de log_download.py del SDK.
    """
    board = device.board
    encontrados = {}

    ev = threading.Event()
    resultado = {}

    def handler(ctx, brd, signals, length):
        resultado["length"] = length
        resultado["signals"] = cast(signals, POINTER(c_void_p * length)) if signals else None
        ev.set()

    handler_fn = FnVoid_VoidP_VoidP_VoidP_UInt(handler)

    # Intervalo minimo durante la descarga: aqui si conviene, es una
    # transferencia masiva de un solo sensor a la vez.
    libmetawear.mbl_mw_settings_set_connection_parameters(board, 7.5, 7.5, 0, 6000)
    sleep(1.0)

    libmetawear.mbl_mw_metawearboard_create_anonymous_datasignals(board, None, handler_fn)
    if not ev.wait(30):
        log("XX  %s: no respondio al buscar los loggers" % mac)
        log("    (ya se le mando el stop; sus datos siguen en la flash)")
        return None

    if not resultado.get("signals"):
        if resultado.get("length"):
            log("XX  %s: error creando anonymous signals, status=%s"
                % (mac, resultado["length"]))
        else:
            log("XX  %s: no hay ningun logger activo (no grabo nada)" % mac)
        return None

    libmetawear.mbl_mw_logging_stop(board)
    sleep(0.5)

    dl = LogDownload(mac, writer)
    for i in range(resultado["length"]):
        sig = resultado["signals"].contents[i]
        ident = libmetawear.mbl_mw_anonymous_datasignal_get_identifier(sig).decode()
        if "acceleration" in ident:
            kind = "acc"
        elif "angular-velocity" in ident:
            kind = "gyro"
        else:
            log("..  %s: logger desconocido '%s', se ignora" % (mac, ident))
            continue
        encontrados[kind] = ident
        libmetawear.mbl_mw_anonymous_datasignal_subscribe(sig, None, dl.make_handler(kind))

    if not encontrados:
        log("XX  %s: los loggers que hay no son acc ni gyro" % mac)
        return None

    def progreso(ctx, left, total):
        if left == 0:
            dl.done.set()

    def desconocido(ctx, id_, epoch, data, length):
        pass

    progreso_fn = FnVoid_VoidP_UInt_UInt(progreso)
    desconocido_fn = FnVoid_VoidP_UByte_Long_UByteP_UByte(desconocido)
    handler_dl = LogDownloadHandler(
        context=None,
        received_progress_update=progreso_fn,
        received_unknown_entry=desconocido_fn,
        received_unhandled_entry=cast(None, FnVoid_VoidP_DataP))

    log("..  %s: descargando..." % mac)
    t0 = time()
    libmetawear.mbl_mw_logging_download(board, n_notifies, byref(handler_dl))
    # Sin limite superior claro: una sesion larga puede tardar minutos. Se
    # corta si pasan 5 min sin terminar, para no colgar la sesion entera.
    if not dl.done.wait(300):
        log("XX  %s: la descarga no termino en 5 min (se guarda lo que llego)" % mac)
    sleep(1.0)
    log("OK  %s: %d acc + %d gyro en %.0f s"
        % (mac, dl.acc, dl.gyro, time() - t0))
    return dl


# ---------------------------------------------------------------------- main

def resolver_macs(args):
    """
    Aplica --macs y --rotate, y corta en --sensors. Aborta si hay una MAC mal
    escrita, que es un error facil de cometer y que si no se detecta aca
    aparece como "ese sensor no conecta" quince intentos despues.
    """
    if args.macs:
        lista = [m.strip().upper() for m in args.macs.split(",") if m.strip()]
        malas = [m for m in lista
                 if not re.fullmatch(r"([0-9A-F]{2}:){5}[0-9A-F]{2}", m)]
        if malas:
            print("MAC(s) con formato invalido: %s" % ", ".join(malas))
            print("Formato esperado: AA:BB:CC:DD:EE:FF, separadas por coma sin espacios.")
            sys.exit(1)
    else:
        lista = list(DEFAULT_MACS)

    if args.rotate:
        r = args.rotate % len(lista)
        lista = lista[r:] + lista[:r]

    return lista[:args.sensors]


def construir_parser():
    """
    El parser vive aparte de main() para que las pruebas puedan validar los
    comandos de COMANDOS.txt sin ejecutar nada: un flag mal escrito en la
    lista de comandos se descubriria si no en medio de la sesion, con los
    sensores puestos.
    """
    parser = argparse.ArgumentParser(
        description="Adquisicion / pruebas de la plataforma multi-IMU (v%s)" % VERSION)
    parser.add_argument("--mode", default="stream",
                        choices=["stream", "log", "download", "check", "scan",
                                 "battery"],
                        help="stream (DEFAULT: transmision BLE en vivo) | "
                             "log (el sensor graba en su memoria y se descarga al "
                             "final; unico modo sin perdida por distancia) | "
                             "download (rescata un log que quedo en los sensores) | "
                             "check (diagnostico del hardware, no conecta nada) | "
                             "scan (busca los sensores sin conectarse; responde si "
                             "el problema es el sensor o el adaptador. Necesita sudo) | "
                             "battery (conecta a cada uno, lee carga y modelo, y se va)")
    parser.add_argument("--por-adaptador", type=int, default=3,
                        dest="por_adaptador",
                        help="cuantos sensores por adaptador en modo split. "
                             "DEFAULT 3 (el 3+3 de siempre). Con 2 y tres "
                             "adaptadores queda 2+2+2, que en streaming "
                             "elimina el problema del ultimo puesto de la cola")
    parser.add_argument("--espera", type=int, default=10,
                        help="segundos de cuenta regresiva antes de arrancar a "
                             "grabar, para ubicarse. DEFAULT 10. Solo modo log")
    parser.add_argument("--adapter", default="split",
                        choices=["split", "onboard", "dongle", "hci0", "hci1"],
                        help="split (DEFAULT: 3 sensores al Bluetooth integrado + 3 al "
                             "dongle) | onboard | dongle. Nunca uses hci0/hci1: el "
                             "numero de hci cambia entre reinicios y ya se comprobo que "
                             "se invirtio entre dos sesiones del mismo dia.")
    parser.add_argument("--sensors", type=int, default=6,
                        help="numero de sensores a usar (primeros N de la lista)")
    parser.add_argument("--macs", default=None,
                        help="lista de MACs separadas por coma, para cambiar el orden o "
                             "usar otros sensores. El orden define que va a cada adaptador "
                             "(3 primeras -> integrado, 3 ultimas -> dongle).")
    parser.add_argument("--rotate", type=int, default=0,
                        help="rota la lista N posiciones a la izquierda. Atajo para "
                             "cambiar quien queda de ultimo en la cola sin escribir "
                             "todas las MACs.")
    parser.add_argument("--duration", type=int, default=60,
                        help="segundos de grabacion (DEFAULT 60)")
    parser.add_argument("--odr", type=int, default=100, choices=sorted(ACC_ODR.keys()),
                        help="frecuencia de muestreo acc/gyro en Hz. DEFAULT 100.")
    parser.add_argument("--packed", dest="packed", action="store_true", default=True,
                        help="usar packed mode, 3 muestras por paquete BLE (ACTIVO POR "
                             "DEFECTO). Sin el, a 100 Hz el giroscopio pierde hasta 89%%. "
                             "No aplica en --mode log.")
    parser.add_argument("--no-packed", dest="packed", action="store_false",
                        help="desactivar packed mode. Solo para comparaciones.")
    parser.add_argument("--conn-interval", type=float, default=30.0,
                        help="intervalo de conexion BLE en ms (rango 7.5-4000). "
                             "DEFAULT 30.")
    parser.add_argument("--supervision", type=int, default=16000,
                        help="supervision timeout en ms (rango 10-32000): cuanto aguanta "
                             "el enlace sin recibir NADA antes de darse por muerto. "
                             "DEFAULT 16000. El codigo original usaba 6000, y con eso "
                             "cualquier sombra del cuerpo de mas de 6 s mataba el enlace.")
    parser.add_argument("--tx-power", type=int, default=4, choices=TX_POWERS,
                        help="potencia de transmision del SENSOR en dBm. DEFAULT 4 (el "
                             "maximo). El firmware arranca en 0 y nunca se habia tocado; "
                             "+4 dBm son ~1.6x de alcance en espacio libre.")
    parser.add_argument("--reconnect", dest="reconnect", action="store_true", default=True,
                        help="reconectar automaticamente los enlaces que se caigan "
                             "(ACTIVO POR DEFECTO)")
    parser.add_argument("--no-reconnect", dest="reconnect", action="store_false",
                        help="no reconectar (comportamiento de la v2, para comparar)")
    parser.add_argument("--reconnect-period", type=float, default=3.0,
                        help="cada cuantos segundos revisa el vigilante. DEFAULT 3.")
    parser.add_argument("--rssi", action="store_true",
                        help="guardar el RSSI (dBm) de cada enlace 1 vez por segundo en "
                             "un CSV aparte. NECESITA sudo.")
    parser.add_argument("--rssi-period", type=float, default=1.0,
                        help="segundos entre lecturas de RSSI. DEFAULT 1.")
    parser.add_argument("--dongle", default="dongle_A",
                        help="etiqueta del dongle fisico conectado ahora (ej. tplink_ant, "
                             "tplink_mini, generico). Se guarda en el run_log para poder "
                             "comparar dongles despues. El script no puede saber cual "
                             "esta enchufado, asi que hay que decirselo.")
    parser.add_argument("--nota", default="",
                        help="texto libre con la condicion fisica de la corrida "
                             "(ej. '3m_espalda', 'caminando_pasillo'). Va al run_log.")
    parser.add_argument("--retries", type=int, default=15,
                        help="intentos de conexion por sensor antes de rendirse.")
    parser.add_argument("--scan-seconds", type=int, default=8,
                        help="segundos de escaneo por adaptador en --mode scan.")
    parser.add_argument("--warmup", type=int, default=5,
                        help="segundos de verificacion de arranque antes de medir.")
    parser.add_argument("--outdir", default=None,
                        help="carpeta de salida. DEFAULT: ./data junto al script. Usa una "
                             "carpeta nueva por tanda para no mezclar con lo viejo.")
    parser.add_argument("--tag", default="run", help="etiqueta para el nombre de archivo")
    return parser


def main():
    args = construir_parser().parse_args()

    if args.mode == "check":
        sys.exit(modo_check())

    if args.mode == "scan":
        sys.exit(modo_scan(resolver_macs(args), args.scan_seconds))

    data_dir = args.outdir or os.path.join(SCRIPT_DIR, "data")
    data_dir = os.path.abspath(os.path.expanduser(data_dir))
    os.makedirs(data_dir, exist_ok=True)
    results_csv = os.path.join(data_dir, "run_log.csv")

    print("acquire.py v%s\n" % VERSION)

    adapters = get_adapters()
    if not adapters:
        print("No se detectaron adaptadores Bluetooth (hciconfig vacio). Abortando.")
        sys.exit(1)
    print("Adaptadores detectados:")
    for name, a in sorted(adapters.items()):
        print("  %s: %s  bus=%-5s -> %s" % (name, a["mac"], a["bus"], a["role"]))

    if args.adapter == "split":
        target_hci = None
    else:
        target_hci = resolve_adapter(adapters, args.adapter)
        if target_hci is None:
            print("No se encontro un adaptador '%s'. Abortando." % args.adapter)
            sys.exit(1)
        a = adapters[target_hci]
        print("\n--adapter %s -> %s (%s, bus=%s)"
              % (args.adapter, target_hci, a["mac"], a["bus"]))

    # Para split: primero el integrado, luego el dongle, en orden estable por
    # rol y NO por numero de hci (que no es confiable entre reinicios).
    hci_names = sorted(adapters.keys(),
                       key=lambda n: (adapters[n]["role"] != "onboard", n))

    if args.adapter == "split":
        roles_presentes = {adapters[n]["role"] for n in hci_names}
        if roles_presentes != {"onboard", "dongle"}:
            print("\nEl modo split necesita el Bluetooth integrado Y un dongle, pero "
                  "solo se detecto: %s." % ", ".join(sorted(roles_presentes)))
            print("Revisa que el dongle este conectado (en un puerto USB 2.0) y que "
                  "aparezca en 'hciconfig -a'. Abortando.")
            sys.exit(1)
        print("\nsplit: %d sensor(es) por adaptador, empezando por el "
              "integrado ('%s')" % (args.por_adaptador, args.dongle))

    needed_hci = hci_names if args.adapter == "split" else [target_hci]
    down = [name for name in needed_hci if not adapter_is_up(name)]
    if down:
        print("\nAdaptador(es) %s no estan UP RUNNING segun hciconfig."
              % ", ".join(down))
        print("No se va a intentar conectar (podria colgar el proceso con un error de "
              "C++ no capturable). Revisa con 'hciconfig -a' y prueba:")
        for name in down:
            print("  sudo hciconfig %s up" % name)
        print("Si acabas de mover el dongle de puerto USB, revisa tambien "
              "'dmesg | tail -40'. Corre 'python3 acquire.py --mode check' para un "
              "diagnostico completo.")
        sys.exit(1)

    def adapter_for(index):
        if args.adapter == "split":
            por = max(1, args.por_adaptador)
            name = hci_names[min(index // por, len(hci_names) - 1)]
        else:
            name = target_hci
        return name, adapters[name]["mac"]

    macs = resolver_macs(args)
    print("\nOrden de conexion:")
    for i, m in enumerate(macs):
        destino = adapters[adapter_for(i)[0]]["role"]
        print("  %d. %s  -> %s" % (i + 1, m, destino))

    if args.rssi and not is_root():
        print("\nAVISO: --rssi necesita root (usa el comando HCI Read_RSSI). "
              "Corre con sudo o la columna va a quedar vacia.")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode_tag = "packed" if (args.packed and args.mode == "stream") else "raw"
    data_path = os.path.join(
        data_dir,
        "%s_%s_%s_%ddev_%s_%dhz_ci%g_%s.csv"
        % (args.tag, args.mode, args.adapter, len(macs), mode_tag, args.odr,
           args.conn_interval, stamp))

    if args.mode == "battery":
        sys.exit(modo_bateria(macs, adapters, adapter_for, args.retries))

    if args.mode == "download":
        sys.exit(modo_download(args, adapters, macs, adapter_for, data_path))

    # buffering=1 (linea por linea): si el proceso muere de golpe (crash,
    # segfault, corte de energia) sin llegar al flush/fsync final, cada fila
    # ya escrita hasta ese punto sobrevive en vez de perderse en el buffer.
    data_file = open(data_path, "w", newline="", buffering=1)
    writer = csv.writer(data_file)
    writer.writerow(["timestamp_host", "mac", "type", "x", "y", "z", "epoch_device"])

    states, failures = [], []
    for i, mac in enumerate(macs):
        hci_name, hci_addr = adapter_for(i)
        role = adapters[hci_name]["role"]
        device, attempt = connect_device(mac, hci_addr, args.retries,
                                          hci_name=hci_name)
        if device is None:
            print("XX  %s NO conecto tras %d intentos -> se continua sin ese sensor"
                  % (mac, args.retries))
            failures.append((mac, hci_name))
            continue
        chip = detectar_chip(device.board)
        modelo = nombre_modelo(device.board)
        bat = leer_bateria(device.board)
        # La bateria se imprime al conectar y no despues: un sensor flojo
        # conecta peor y se cae antes, y eso se confunde con un problema de
        # alcance o de dongle si no se sabe como estaba de carga.
        if bat is None:
            txt_bat = ""
        elif bat["pct"] <= 20:
            txt_bat = "  bateria %d%% (BAJA)" % bat["pct"]
        else:
            txt_bat = "  bateria %d%%" % bat["pct"]
        print("OK  %s via %s (%s) [intento %d]  %s / %s%s"
              % (mac, role, hci_name, attempt, modelo, chip, txt_bat))
        states.append(State(device, writer, hci_name, role, chip))

    if not states:
        print("\nNingun sensor conectado. Abortando.")
        print("\nQue mirar, en este orden:")
        print("  1. Estan los sensores prendidos y en rango? Comprobalo sin")
        print("     conectar nada:   sudo python3 acquire.py --mode scan")
        print("  2. Quedo el adaptador pegado? Reinicialo:")
        for name in needed_hci:
            print("     sudo hciconfig %s down && sudo hciconfig %s up" % (name, name))
        print("  3. Aisla el adaptador: probá uno solo, con un solo sensor.")
        print("     python3 acquire.py --adapter onboard --sensors 1 --duration 10")
        print("     python3 acquire.py --adapter dongle  --sensors 1 --duration 10")
        print("  4. Si bajaste el WiFi con 'ifconfig wlan0 down', volvé a subirlo:")
        print("     en la Raspberry Pi 4B el WiFi y el Bluetooth integrado son el")
        print("     mismo chip, y bajar wlan0 puede dejar tocado el hci integrado.")
        print("     sudo ifconfig wlan0 up")
        data_file.close()
        sys.exit(1)

    chips = {st.chip for st in states}
    if len(chips) > 1:
        print("\nAVISO: hay sensores con chips distintos en la misma corrida "
              "(%s)." % ", ".join(sorted(chips)))
        print("El script los maneja bien, pero el BMI160 y el BMI270 no son el "
              "mismo sensor: no mezcles sus datos en una comparacion sin decirlo.")

    if args.mode == "log":
        codigo = correr_log(args, states, failures, macs, adapters, stamp,
                            data_file, writer, data_path, results_csv)
        sys.exit(codigo)

    # ------------------------------------------------------------- streaming
    print("\nConfigurando %d sensor(es) a %d Hz (%s), intervalo %g ms, "
          "supervision %d ms, TX %+d dBm..."
          % (len(states), args.odr, "packed" if args.packed else "sin empaquetar",
             args.conn_interval, args.supervision, args.tx_power))

    # Dos pasadas: primero los parametros de enlace en TODOS los sensores, una
    # sola espera para que asienten las renegociaciones, y despues la
    # configuracion de los sensores. Hacerlo por sensor costaba 3 s x 6 = 18 s
    # de arranque, que en corridas de 30-60 s pesa mas que la propia medicion.
    for st in states:
        set_link_params(st.device.board, args.conn_interval, args.supervision,
                        args.tx_power, settle=0.0)
    sleep(3.5)
    for st in states:
        set_sensor_config(st.device.board, args.odr, st.chip)
        subscribe_stream(st, args.packed)

    # --- Verificacion de arranque -------------------------------------
    # Comprobar que cada sensor realmente esta transmitiendo ANTES de gastar la
    # medicion completa. Un sensor cuya configuracion fallo conecta pero no
    # manda nada, y antes eso solo se descubria al final como 100% de perdida.
    print("\nVerificando arranque (%d s)..." % args.warmup)
    for st in states:
        st.start_streaming()
    sleep(args.warmup)
    mudos = [s for s in states if s.acc_samples == 0 and s.gyro_samples == 0]

    if mudos:
        print("%d sensor(es) conectados pero sin transmitir -> reconfigurando: %s"
              % (len(mudos), ", ".join(s.device.address for s in mudos)))
        for st in mudos:
            st.stop_streaming()
            unsubscribe_stream(st)
            set_link_params(st.device.board, args.conn_interval, args.supervision,
                            args.tx_power, settle=3.0)
            set_sensor_config(st.device.board, args.odr, st.chip)
            subscribe_stream(st, args.packed)
            st.start_streaming()
        sleep(args.warmup)
        siguen = [s for s in mudos if s.acc_samples == 0 and s.gyro_samples == 0]
        if siguen:
            print("SIGUEN SIN TRANSMITIR tras reconfigurar: "
                  + ", ".join(s.device.address for s in siguen))
            print("Se continua igual; van a aparecer como 100% de perdida.")
    else:
        print("Todos los sensores conectados estan transmitiendo.")

    # Reiniciar contadores: la medicion real empieza limpia, sin el warmup.
    for st in states:
        st.acc_samples = 0
        st.gyro_samples = 0
        st.drops = 0
        st.reconnects = 0
        st.first_drop_at = None
        st.down_since = None
        st.downtime = 0.0

    counters_before = read_byte_counters()
    stop_event = threading.Event()

    vigilante = None
    if args.reconnect:
        vigilante = Reconnector(states, adapters, args.packed, args.conn_interval,
                                args.supervision, args.tx_power, args.odr,
                                args.reconnect_period, stop_event)
        vigilante.start()

    muestreador = None
    rssi_path = data_path.replace(".csv", "_rssi.csv")
    if args.rssi:
        muestreador = RssiSampler(states, rssi_path, args.rssi_period, stop_event)
        muestreador.start()

    print("\nTransmitiendo %d s%s (Ctrl+C corta antes, igual imprime resumen)..."
          % (args.duration, " con reconexion automatica" if args.reconnect else ""))
    t_start = time()
    try:
        sleep(args.duration)
    except KeyboardInterrupt:
        print("\nDetenido manualmente.")
    stop_event.set()
    for st in states:
        st.tearing_down = True
        st.mark_up()
    for st in states:
        try:
            st.stop_streaming()
        except Exception:
            pass
    elapsed = time() - t_start

    counters_after = read_byte_counters()

    for st in states:
        try:
            unsubscribe_stream(st)
            libmetawear.mbl_mw_debug_disconnect(st.device.board)
        except Exception:
            pass
        sleep(0.3)  # dar tiempo a que cada desconexion asiente antes de la siguiente
    data_file.flush()
    os.fsync(data_file.fileno())
    data_file.close()

    escribir_resultados(args, states, failures, macs, adapters, stamp, elapsed,
                        counters_before, counters_after, results_csv,
                        data_path, args.odr * elapsed, t_start)

    if args.rssi:
        print("RSSI: %s (%d lecturas validas)"
              % (rssi_path, muestreador.samples if muestreador else 0))

    # Los datos y el log ya estan escritos y con fsync en este punto. El cierre
    # normal del interprete (destructores, hilos nativos del SDK todavia
    # corriendo en segundo plano) produce un Segmentation fault intermitente al
    # salir -- salir de una vez evita que ese crash deje el adaptador BLE en mal
    # estado para la siguiente corrida.
    sys.stdout.flush()
    os._exit(0)


# ------------------------------------------------------- modo log / download

def led(board, color=None):
    """Verde fijo = grabando. Rojo parpadeando = quedo afuera. None = apagar."""
    try:
        libmetawear.mbl_mw_led_stop_and_clear(board)
        if color is None:
            return
        patron = LedPattern(repeat_count=Const.LED_REPEAT_INDEFINITELY)
        preset = LedPreset.SOLID if color == LedColor.GREEN else LedPreset.BLINK
        libmetawear.mbl_mw_led_load_preset_pattern(byref(patron), preset)
        libmetawear.mbl_mw_led_write_pattern(board, byref(patron), color)
        libmetawear.mbl_mw_led_play(board)
    except Exception:
        pass        # el LED es informativo; nunca puede tumbar una corrida


def cuenta_regresiva(segundos, mensaje):
    """Cuenta regresiva sobreescribiendo la misma linea."""
    fin = time() + segundos
    while True:
        falta = fin - time()
        if falta <= 0:
            break
        sys.stdout.write("\r%s %4d s   " % (mensaje, int(falta + 0.999)))
        sys.stdout.flush()
        sleep(min(1.0, falta))
    sys.stdout.write("\r" + " " * 72 + "\r")
    sys.stdout.flush()


def largo_del_log(board, timeout=5.0):
    """Cuantas entradas tiene la flash. None si el sensor no contesta."""
    try:
        signal = libmetawear.mbl_mw_logging_get_length_data_signal(board)
    except Exception:
        return None
    if not signal:
        return None
    listo, caja = threading.Event(), {}

    def cb(ctx, ptr):
        caja["v"] = parse_value(ptr)
        listo.set()

    fn = FnVoid_VoidP_DataP(cb)
    libmetawear.mbl_mw_datasignal_subscribe(signal, None, fn)
    libmetawear.mbl_mw_datasignal_read(signal)
    ok = listo.wait(timeout)
    libmetawear.mbl_mw_datasignal_unsubscribe(signal)
    return caja.get("v") if ok else None


def vaciar_memoria_verificada(board, mac, espera=8.0, reintentos=4):
    """
    Borra la flash y NO sigue hasta confirmar que quedo en cero.

    El bug que dejaba a EE:9D:A0:09:C6:11 sin grabar las tres corridas del
    27-ago: el codigo viejo esperaba 0.5 s fijos despues de
    logging_clear_entries. Borrar 8 MB tarda segundos, y logging_start(b, 0)
    -sin sobreescritura- sobre una memoria todavia ocupada para de inmediato
    y no graba nada, sin devolver ningun error: el sensor decia "grabando" y
    al descargar traia 0 muestras.
    """
    libmetawear.mbl_mw_logging_stop(board)
    sleep(0.5)
    if existe("mbl_mw_logging_flush_page"):
        libmetawear.mbl_mw_logging_flush_page(board)
        sleep(0.5)
    libmetawear.mbl_mw_logging_clear_entries(board)
    sleep(espera)
    for intento in range(1, reintentos + 1):
        largo = largo_del_log(board)
        if largo == 0:
            return True
        if largo is None:
            print("..  %s: la memoria no contesto (intento %d/%d)"
                  % (mac, intento, reintentos))
        else:
            print("..  %s: %d entradas todavia (intento %d/%d)"
                  % (mac, largo, intento, reintentos))
        if intento < reintentos:
            sleep(3.0)
    print("XX  %s: la flash no quedo confirmada en cero" % mac)
    return False


def correr_log(args, states, failures, macs, adapters, stamp, data_file, writer,
               data_path, results_csv):
    """
    Graba en la memoria flash de cada sensor y descarga al final.

    Durante la ventana de grabacion no importa si el enlace BLE se cae: el
    sensor sigue guardando en su flash. Por eso este modo es la unica forma
    de garantizar 0% de perdida con el sujeto en movimiento y lejos de la Pi.
    El costo es que no hay datos en vivo y que la descarga tarda (del orden
    de 1 minuto por sensor por cada minuto grabado a 100 Hz).
    """
    print("\nConfigurando %d sensor(es) para GRABAR EN MEMORIA a %d Hz, "
          "supervision %d ms, TX %+d dBm..."
          % (len(states), args.odr, args.supervision, args.tx_power))

    for st in states:
        set_link_params(st.device.board, args.conn_interval, args.supervision,
                        args.tx_power, settle=0.0)
    sleep(3.5)

    # --- FASE 1: preparar a todos, sin arrancar a ninguno --------------
    # Preparar es lento (borrar la flash son ~8 s por sensor). Si se
    # arrancara aca mismo, el primero empezaria a grabar casi un minuto
    # antes que el ultimo: medido en los CSV del 27-ago, 5.6 s de desfase
    # con el borrado viejo de 0.5 s, y mucho peor con el verificado.
    listos = []
    for st in states:
        board = st.device.board
        try:
            set_sensor_config(board, args.odr, st.chip)
            # Pizarra limpia. tear_down borra los loggers que hayan quedado de
            # una corrida anterior (por ejemplo si esa corrida murio antes de
            # descargar): si no, el sensor termina con loggers duplicados y la
            # descarga trae mezclado lo viejo con lo nuevo. clear_entries borra
            # ademas los datos ya guardados en la flash.
            if existe("mbl_mw_metawearboard_tear_down"):
                libmetawear.mbl_mw_metawearboard_tear_down(board)
                sleep(0.5)
            if not vaciar_memoria_verificada(board, st.device.address):
                led(board, LedColor.RED)
                continue
            acc, gyro = get_signals(board, packed=False, chip=st.chip)
            st.logger_acc = create_voidp(
                lambda fn, s=acc: libmetawear.mbl_mw_datasignal_log(s, None, fn),
                resource="acc_logger")
            st.logger_gyro = create_voidp(
                lambda fn, s=gyro: libmetawear.mbl_mw_datasignal_log(s, None, fn),
                resource="gyro_logger")
            listos.append(st)
            print("preparado  %s" % st.device.address)
        except Exception as e:
            print("XX  %s no se pudo preparar: %s" % (st.device.address, e))
            led(board, LedColor.RED)

    if not listos:
        print("Ningun sensor quedo listo. Abortando.")
        data_file.close()
        return 1

    if args.espera:
        print("")
        cuenta_regresiva(args.espera, "Ubicate. La grabacion empieza en")

    # --- FASE 2: arrancar a todos seguidos, sin nada en el medio -------
    # Este bucle tiene que ser lo mas corto posible: cada instruccion que
    # se le agregue se paga como desfase entre el primero y el ultimo.
    arrancados = []
    for st in listos:
        board = st.device.board
        try:
            libmetawear.mbl_mw_logging_start(board, 0)  # 0 = no sobreescribir
            # Encender acc y gyro sin marcar is_streaming: en este modo no hay
            # suscripcion, los datos van a la flash del sensor, no a callbacks.
            g = gyro_api(st.chip)
            libmetawear.mbl_mw_acc_enable_acceleration_sampling(board)
            libmetawear.mbl_mw_acc_start(board)
            g["enable_rotation_sampling"](board)
            g["start"](board)
            arrancados.append(st)
        except Exception as e:
            print("XX  %s no arranco: %s" % (st.device.address, e))
            led(board, LedColor.RED)
    if not arrancados:
        print("Ningun sensor quedo grabando. Abortando.")
        data_file.close()
        return 1
    listos = arrancados
    t_start = time()

    # El LED va despues del arranque, no en el medio: es una escritura BLE
    # mas y no vale la pena pagarla en desfase.
    for st in listos:
        led(st.device.board, LedColor.GREEN)

    # Comprobar que la flash de verdad esta creciendo. Un sensor que dice
    # "grabando" pero no graba se veia recien en la descarga, con 0 muestras.
    sleep(3.0)
    for st in listos:
        largo = largo_del_log(st.device.board)
        if largo is None:
            print("??  %s: no se pudo comprobar, se sigue igual" % st.device.address)
        elif largo > 0:
            print("GRABANDO  %s (%s entradas a los 3 s)" % (st.device.address, largo))
        else:
            print("XX  %s NO ESTA GRABANDO: la memoria no crece" % st.device.address)
            led(st.device.board, LedColor.RED)

    print("\n" + "=" * 72)
    print("GRABANDO EN MEMORIA %d s. Ya podes alejarte de la Raspberry:" % args.duration)
    print("no hace falta que el enlace BLE sobreviva, el sensor guarda solo.")
    print("Volve cerca de la Pi ANTES de que termine la cuenta, para la descarga.")
    print("=" * 72)
    try:
        cuenta_regresiva(args.duration, "Grabando, faltan")
    except KeyboardInterrupt:
        print("\nDetenido manualmente.")
    elapsed = time() - t_start

    # Parar de grabar ANTES de descargar, y a todos. La descarga es la parte
    # que falla; si el stop dependiera de que la descarga salga bien, un
    # sensor con la descarga rota queda grabando para siempre. Es lo que le
    # paso a FB el 27-ago: 108.4 s de datos en una corrida de 69.9 s, y el
    # LED verde encendido despues de terminar.
    for st in listos:
        parar_de_grabar(st.device.board, st.chip)
        led(st.device.board, None)
    print("Grabacion detenida y LED apagados en los %d sensores." % len(listos))

    print("\nCerrando la grabacion y descargando. Manten los sensores cerca de la Pi.")
    for st in listos:
        st.tearing_down = True
        # Si el enlace se cayo durante el paseo hay que volver a levantarlo para
        # poder mandar el "stop" y descargar. Es justo el caso que este modo
        # espera que pase.
        if not st.device.is_connected:
            print("..  %s perdio el enlace durante la grabacion, reconectando..."
                  % st.device.address)
            hci_addr = adapters[st.hci_name]["mac"]
            reconectado = False
            for intento in range(1, args.retries + 1):
                if reabrir_enlace(st, hci_addr):
                    reconectado = True
                    break
                sleep(3)
            if not reconectado:
                print("XX  %s no volvio: sus datos quedan EN EL SENSOR (la flash "
                      "no se borra sola). Recuperalos con: python3 acquire.py "
                      "--mode download --macs %s"
                      % (st.device.address, st.device.address))
                continue
        try:
            g = gyro_api(st.chip)
            libmetawear.mbl_mw_acc_stop(st.device.board)
            libmetawear.mbl_mw_acc_disable_acceleration_sampling(st.device.board)
            g["stop"](st.device.board)
            g["disable_rotation_sampling"](st.device.board)
            libmetawear.mbl_mw_logging_stop(st.device.board)
            libmetawear.mbl_mw_logging_flush_page(st.device.board)
            sleep(1.0)
        except Exception as e:
            print("..  %s: error al detener la grabacion: %s" % (st.device.address, e))

    descargas = {}
    for st in listos:
        try:
            if not st.device.is_connected:
                continue
            dl = descargar_log(st.device, writer, st.device.address)
            if dl is not None:
                descargas[st.device.address] = dl
                st.acc_samples = dl.acc
                st.gyro_samples = dl.gyro
                # Descarga completa -> se puede limpiar la flash para la
                # siguiente corrida. Solo despues de tener los datos.
                libmetawear.mbl_mw_logging_clear_entries(st.device.board)
                sleep(0.5)
                libmetawear.mbl_mw_debug_reset(st.device.board)
                sleep(1.0)
        except Exception as e:
            print("XX  %s: fallo la descarga: %s" % (st.device.address, e))

    data_file.flush()
    os.fsync(data_file.fileno())
    data_file.close()

    escribir_resultados(args, listos, failures, macs, adapters, stamp, elapsed,
                        {}, {}, results_csv, data_path, args.odr * elapsed, t_start)
    sys.stdout.flush()
    os._exit(0)


def modo_download(args, adapters, macs, adapter_for, data_path):
    """Rescata datos que quedaron grabados en los sensores de una corrida previa."""
    print("\nModo rescate: se conecta a cada sensor y descarga lo que tenga guardado.")
    f = open(data_path, "w", newline="", buffering=1)
    writer = csv.writer(f)
    writer.writerow(["timestamp_host", "mac", "type", "x", "y", "z", "epoch_device"])
    total = 0
    for i, mac in enumerate(macs):
        hci_name, hci_addr = adapter_for(i)
        device, _ = connect_device(mac, hci_addr, args.retries, hci_name=hci_name)
        if device is None:
            print("XX  %s no conecto" % mac)
            continue
        try:
            dl = descargar_log(device, writer, mac)
            if dl is not None:
                total += dl.acc + dl.gyro
                libmetawear.mbl_mw_logging_clear_entries(device.board)
                sleep(0.5)
                libmetawear.mbl_mw_debug_reset(device.board)
                sleep(1.0)
            else:
                libmetawear.mbl_mw_debug_disconnect(device.board)
        except Exception as e:
            print("XX  %s: %s" % (mac, e))
        sleep(1.0)
    f.flush()
    os.fsync(f.fileno())
    f.close()
    print("\n%d muestras rescatadas -> %s" % (total, data_path))
    sys.stdout.flush()
    os._exit(0)


# ------------------------------------------------------------------ resultados

HEADER = ["run_stamp", "tag", "modo", "adapter_mode", "packed", "odr_hz",
          "conn_interval_ms", "supervision_ms", "tx_power_dbm", "reconexion_activa",
          "dongle_label", "nota", "sensors_requested", "sensors_connected",
          "device_mac", "adapter_role", "hci", "elapsed_s", "acc_hz", "gyro_hz",
          "acc_loss_pct", "gyro_loss_pct", "caidas", "reconexiones",
          "seg_sin_datos", "desconexion_s", "rx_bytes_adapter", "tx_bytes_adapter"]


def escribir_resultados(args, states, failures, macs, adapters, stamp, elapsed,
                        counters_before, counters_after, results_csv, data_path,
                        expected, t_start):
    print("\n" + "=" * 78)
    print("RESUMEN - tag=%s modo=%s adapter=%s odr=%dHz packed=%s ci=%gms "
          "sup=%dms tx=%+ddBm reconexion=%s"
          % (args.tag, args.mode, args.adapter, args.odr,
             args.packed and args.mode == "stream", args.conn_interval,
             args.supervision, args.tx_power, args.reconnect))
    print("dongle=%s nota=%s pedidos=%d conectados=%d elapsed=%.1fs"
          % (args.dongle, args.nota or "-", len(macs), len(states), elapsed))
    print("=" * 78)
    print("%-20s %-8s %7s %8s %9s %10s %7s" %
          ("device", "adaptador", "acc Hz", "gyro Hz", "acc loss", "gyro loss", "caidas"))

    filas = []
    for st in states:
        acc_hz = st.acc_samples / elapsed
        gyro_hz = st.gyro_samples / elapsed
        acc_loss = max(0.0, 1 - st.acc_samples / expected) * 100 if expected else 0.0
        gyro_loss = max(0.0, 1 - st.gyro_samples / expected) * 100 if expected else 0.0
        if st.first_drop_at is None:
            corte, extra = "", ""
        else:
            corte = "%.1f" % (st.first_drop_at - t_start)
            extra = "  1a caida a los %ss, %.1fs sin datos, %d reconexiones" % (
                corte, st.downtime, st.reconnects)
        print("%-20s %-8s %7.1f %8.1f %8.1f%% %9.1f%% %7d%s"
              % (st.device.address, st.role, acc_hz, gyro_hz, acc_loss, gyro_loss,
                 st.drops, extra))
        filas.append([
            stamp, args.tag, args.mode, args.adapter,
            args.packed and args.mode == "stream", args.odr, args.conn_interval,
            args.supervision, args.tx_power, args.reconnect,
            args.dongle if st.role == "dongle" else "", args.nota,
            len(macs), len(states), st.device.address, st.role, st.hci_name,
            "%.1f" % elapsed, "%.1f" % acc_hz, "%.1f" % gyro_hz,
            "%.1f" % acc_loss, "%.1f" % gyro_loss,
            st.drops, st.reconnects, "%.1f" % st.downtime, corte,
        ])

    for mac, hci_name in failures:
        role = adapters[hci_name]["role"]
        print("%-20s %-8s %35s" % (mac, role, "NO CONECTO"))
        filas.append([
            stamp, args.tag, args.mode, args.adapter,
            args.packed and args.mode == "stream", args.odr, args.conn_interval,
            args.supervision, args.tx_power, args.reconnect,
            args.dongle if role == "dongle" else "", args.nota,
            len(macs), len(states), mac, role, hci_name, "%.1f" % elapsed,
            "connect_failed", "connect_failed", "", "", "", "", "", "",
        ])

    if counters_after:
        print("\nContadores de bytes por adaptador (delta durante la transmision):")
        rx_by_hci, tx_by_hci = {}, {}
        for name in sorted(counters_after.keys()):
            rx0, tx0 = counters_before.get(name, (0, 0))
            rx1, tx1 = counters_after.get(name, (0, 0))
            rx_by_hci[name], tx_by_hci[name] = rx1 - rx0, tx1 - tx0
            print("  %s: RX %9d bytes (%8.0f B/s)   TX %7d bytes (%6.0f B/s)"
                  % (name, rx1 - rx0, (rx1 - rx0) / elapsed,
                     tx1 - tx0, (tx1 - tx0) / elapsed))
        idx_hci = HEADER.index("hci")
        for fila in filas:
            fila.append(rx_by_hci.get(fila[idx_hci], ""))
            fila.append(tx_by_hci.get(fila[idx_hci], ""))
    else:
        for fila in filas:
            fila.extend(["", ""])

    # Si ya existe un run_log de una version anterior del script (con menos
    # columnas), seguir agregando ahi desalinearia el archivo en silencio.
    # En ese caso se archiva el viejo y se empieza uno nuevo.
    if os.path.exists(results_csv):
        with open(results_csv, newline="") as f:
            old_header = next(csv.reader(f), [])
        if old_header != HEADER:
            backup = results_csv.replace(".csv", "_v_anterior_%s.csv" % stamp)
            os.rename(results_csv, backup)
            print("\nAVISO: el run_log.csv anterior tenia otras columnas; se archivo "
                  "como %s y se empieza uno nuevo." % os.path.basename(backup))

    nuevo = not os.path.exists(results_csv)
    with open(results_csv, "a", newline="") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(HEADER)
        w.writerows(filas)
        # Si esta carpeta esta en una USB, un with normal deja los bytes en cache
        # del SO -- si sacan la unidad sin desmontarla el archivo puede quedar
        # vacio aunque el programa haya terminado sin error (paso el 20-ago).
        f.flush()
        os.fsync(f.fileno())

    print("\nDatos crudos: %s" % data_path)
    print("Log acumulado: %s" % results_csv)


if __name__ == "__main__":
    main()
