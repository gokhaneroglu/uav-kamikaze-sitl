# SÜREKLİ DALIŞ 50 m: Kararlı ilk uçuşun yanal kontrolü korunur.
# HOLD tamamlandıktan sonra uçak 50.0 m bağıl irtifaya kadar kesintisiz dalar.
# 50 m üstünde tek, sabit dalış pitch'i korunur; RECOVERY yalnızca
# DIVE_END_ALT eşiği bir kez görüldüğünde başlar. Harici GUIDED kontrolü
# sırasında 30 m görülürse acil emniyet katmanı attitude kontrolünü kesip RTL ister.



from __future__ import annotations

import math
import os
import sys
import time
from typing import Dict, List, Optional

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase


# ============================================================
# BAĞLANTI VE MAVLINK KİMLİĞİ
# ============================================================

# Jetson varsayılanı UART1 (/dev/ttyTHS0). Farklı bir Jetson taşıyıcı kartı,
# USB telemetri modülü veya SITL kullanılırsa kodu değiştirmeden ortam
# değişkenleriyle bağlantı seçilebilir.
CONNECTION = os.getenv("MAVLINK_CONNECTION", "/dev/ttyTHS0")
BAUD = int(os.getenv("MAVLINK_BAUD", "57600"))
SOURCE_SYSTEM = int(os.getenv("MAVLINK_SOURCE_SYSTEM", "246"))
SOURCE_COMPONENT = int(os.getenv("MAVLINK_SOURCE_COMPONENT", "190"))


# ============================================================
# GÖREV AYARLARI
# ============================================================

MIN_START_ALT = 30.0
CRUISE_ALT = 100.0

DIVE_WP_SEQ = 7
# GUIDED'a hedefe çok uzakken veya artık fazla yaklaşmışken girilmez.
# 120-200 m penceresi, yaklaşık 13-15 m/s hızda 6 s HOLD için yeterli alan bırakır.
GUIDED_ENTRY_DISTANCE_MAX_M = 200.0
GUIDED_ENTRY_DISTANCE_MIN_M = 120.0
DIVE_BEARING_FREEZE_DISTANCE = 35.0
DIVE_BEARING_FILTER_ALPHA = 0.15
DIVE_TRIGGER_ALT = 100.0
DIVE_TRIGGER_ALT_TOLERANCE = 3.0

# AUTO, dalış noktasına göre doğrultuya oturduğu ilk anda GUIDED'e geçer.
AUTO_ALIGN_HEADING_TOLERANCE_DEG = 5.0
AUTO_ALIGN_ROLL_TOLERANCE_DEG = 5.0
GUIDED_HOLD_DURATION_SEC = 6.0
GUIDED_HOLD_MAX_DURATION_SEC = 12.0
GUIDED_ALIGN_CONFIRM_SEC = 2.0
GUIDED_ALIGN_HEADING_TOLERANCE_DEG = 3.0
GUIDED_ALIGN_ROLL_TOLERANCE_DEG = 3.0
GUIDED_FORWARD_WP_MIN_DISTANCE_M = 120.0
GUIDED_HOLD_MIN_THRUST = 0.60
GUIDED_HOLD_ALT_KP = 0.30
GUIDED_HOLD_VZ_KD = 0.80
GUIDED_HOLD_MIN_PITCH_DEG = -1.0
GUIDED_HOLD_MAX_PITCH_DEG = 7.0
GUIDED_HOLD_ROLL_COMMAND_MAX_DEG = 4.0
GUIDED_HOLD_ROLL_RATE_LIMIT_DEG_PER_SEC = 10.0
GUIDED_HOLD_FINAL_LEVEL_SEC = 2.0

TARGET_FLIGHT_PATH_DOWN_DEG = 48.0

# İlk kayıttaki kararlı PID yapısı korunur, fakat dalış daha keskin başlatılır.
# En önemli sınır DIVE_MAX_PITCH_DEG=-12'dir: 50 m üstünde denetleyici
# burun kaldırma/level komutu veremez. Böylece g2'deki dalış-toparlanma döngüsü
# oluşmaz; uçuş-yolu açısı aşılırsa yalnızca dalış şiddeti azaltılır.
DIVE_BASE_PITCH_DEG = -38.0
DIVE_PATH_KP = 0.55
DIVE_PATH_KI = 0.002
DIVE_PATH_KD = 0.35
DIVE_PATH_D_FILTER_TAU = 0.30
DIVE_PATH_INTEGRAL_LIMIT = 8.0
DIVE_MIN_PITCH_DEG = -38.0
DIVE_MAX_PITCH_DEG = -38.0
DIVE_THRUST = 0.35  # Gerçek uçuşta kontrol otoritesini kaybetmemek için %20 yerine %35
DIVE_END_ALT = 50.0  # RECOVERY komutunun başlayacağı bağıl irtifa
EMERGENCY_RTL_ALT = 30.0  # GUIDED fazlarında mutlak acil RTL tabanı (bağıl irtifa)

# Önceki dinamik erken pull-out hesabının sabitleri. Bu 50 m doğrudan tetik
# sürümünde kontrol kararı için kullanılmaz; yalnızca karşılaştırma/telemetri için tutulur.
PULLOUT_RESPONSE_TIME_SEC = 3.20
PULLOUT_SAFETY_MARGIN_M = 5.0
PULLOUT_TRIGGER_MIN_ALT = 70.0
PULLOUT_TRIGGER_MAX_ALT = 90.0

# AUTO -> GUIDED geçişi mevcut pitch ve gazdan başlatılır.
DIVE_ENTRY_TRANSITION_SEC = 1.40
DIVE_ENTRY_LEVEL_SEC = 0.75      # HOLD sonunda kanatlar zaten yatay; kısa son doğrulama
DIVE_ENTRY_LEVEL_ROLL_MAX_DEG = 3.0
DIVE_ENTRY_LEVEL_THRUST = 0.55
DIVE_ENTRY_MIN_THRUST = 0.45
DIVE_MIN_SAFE_AIRSPEED = 14.0
DIVE_LOW_AIRSPEED_THRUST = 0.58
DIVE_LOW_AIRSPEED_ON = 13.5
DIVE_LOW_AIRSPEED_OFF = 15.0
DIVE_LOW_AIRSPEED_MAX_NOSE_DOWN_DEG = -28.0
DIVE_PITCH_NOSE_DOWN_RATE_DEG_PER_SEC = 22.0
DIVE_PITCH_NOSE_UP_RATE_DEG_PER_SEC = 22.0
DIVE_ROLL_RATE_LIMIT_DEG_PER_SEC = 12.0

# Kademeli yanal kontrol: yer izi PID'i küçük bir hedef roll üretir; iç roll
# PID'i ise gerçek rollü bu hedefe taşır. Yaw açısı doğrudan zorlanmaz.
COURSE_D_FILTER_TAU = 0.30
ROLL_D_FILTER_TAU = 0.12
COURSE_INTEGRAL_LIMIT = 35.0
ROLL_INTEGRAL_LIMIT = 12.0
LATERAL_LEVEL_PRIORITY_DEG = 4.0
# Bu eşik aşıldığında heading düzeltmesi tamamen kapatılmaz; yalnızca azaltılır.
LATERAL_HEADING_RETAIN_RATIO = 0.35

DIVE_COURSE_KP = 0.18
DIVE_COURSE_KI = 0.010
DIVE_COURSE_KD = 0.035
DIVE_DESIRED_ROLL_MAX_DEG = 3.0
DIVE_ROLL_COMMAND_MAX_DEG = 6.0

RECOVERY_COURSE_KP = 0.16
RECOVERY_COURSE_KI = 0.008
RECOVERY_COURSE_KD = 0.030
RECOVERY_DESIRED_ROLL_MAX_DEG = 2.5
RECOVERY_ROLL_COMMAND_MAX_DEG = 6.0

STRAIGHT_COURSE_KP = 0.14
STRAIGHT_COURSE_KI = 0.006
STRAIGHT_COURSE_KD = 0.025
STRAIGHT_DESIRED_ROLL_MAX_DEG = 5.0
STRAIGHT_ROLL_COMMAND_MAX_DEG = 18.0

ROLL_PID_KP = 1.60
ROLL_PID_KI = 0.20
ROLL_PID_KD = 0.12

# Eski yardımcı fonksiyonların uyumluluk sabitleri.
ROLL_LEVEL_KP = 0.90
ROLL_LEVEL_KD = 0.20
ROLL_LEVEL_MAX_CMD_DEG = 25.0
DIVE_HEADING_ROLL_KP = DIVE_COURSE_KP
DIVE_HEADING_ROLL_MAX_DEG = DIVE_DESIRED_ROLL_MAX_DEG
RECOVERY_HEADING_ROLL_KP = RECOVERY_COURSE_KP
RECOVERY_HEADING_ROLL_MAX_DEG = RECOVERY_DESIRED_ROLL_MAX_DEG
STRAIGHT_HEADING_ROLL_KP = STRAIGHT_COURSE_KP
STRAIGHT_HEADING_ROLL_MAX_DEG = STRAIGHT_DESIRED_ROLL_MAX_DEG

# Toparlanma sırasında daha güçlü kanat yataylama.
RECOVERY_ROLL_KP = 0.75
RECOVERY_ROLL_KD = 0.18
RECOVERY_ROLL_MAX_CMD_DEG = 25.0
RECOVERY_ROLL_PRIORITY_DEG = 10.0
RECOVERY_ROLL_CRITICAL_DEG = 20.0

# Toparlanma iki aşamalıdır. Pitch, uçuş-yolu açısına göre sınırlanır;
# gaz 1 saniyelik yumuşak geçişle %100'e çıkar.
RECOVERY_TARGET_PATH_DOWN_DEG = -12.0
RECOVERY_BASE_PITCH_DEG = 10.0
RECOVERY_PATH_KP = 0.22
RECOVERY_PATH_KI = 0.0
RECOVERY_PATH_KD = 0.06
RECOVERY_PATH_D_FILTER_TAU = 0.30
RECOVERY_PATH_INTEGRAL_LIMIT = 8.0
RECOVERY_MIN_PITCH_DEG = 3.0
RECOVERY_TARGET_PITCH_DEG = 14.0
RECOVERY_PITCH_NOSE_UP_RATE_DEG_PER_SEC = 45.0
RECOVERY_PITCH_NOSE_DOWN_RATE_DEG_PER_SEC = 18.0
RECOVERY_MIN_THRUST = 0.68
RECOVERY_NOMINAL_THRUST = 1.00
RECOVERY_LOW_SPEED_THRUST = 1.00
RECOVERY_THRUST_RAMP_SEC = 1.00
RECOVERY_COMPLETE_ALT = 100.0
RECOVERY_LEVEL_PATH_TOLERANCE_DEG = 5.0
RECOVERY_HEADING_RAMP_SEC = 2.0
RECOVERY_EMERGENCY_ALT = 45.0
RECOVERY_PATH_LEVEL_THRESHOLD_DEG = 3.0
RECOVERY_CLIMB_PATH_TARGET_DEG = -12.0

# Roll koruması: kilitlenen GPS yer izi dış PID ile küçük hedef bank üretir;
# belirgin yatışta öncelik yeniden kanatları yataylamaktır.
BANK_PROTECTION_START_DEG = 5.0
BANK_PROTECTION_CRITICAL_DEG = 10.0

# Toparlanma boyunca dalıştan çıkış doğrultusu sabit tutulur.
RECOVERY_HEADING_KP = 0.70
RECOVERY_MAX_ROLL_DEG = 6.0

# Toparlanma tamamlandıktan sonra aynı doğrultuda düz uçuş.
# Geçiş yumuşak yapılır; irtifa ve dikey hız birlikte kullanılır.
STRAIGHT_HOLD_ALT = RECOVERY_COMPLETE_ALT
STRAIGHT_BASE_PITCH_DEG = 4.0
STRAIGHT_ALT_KP = 0.35
STRAIGHT_VZ_KD = 0.85
STRAIGHT_MIN_PITCH_DEG = -2.0
STRAIGHT_MAX_PITCH_DEG = 12.0
STRAIGHT_BASE_THRUST = 0.78
STRAIGHT_TRANSITION_SEC = 4.0
STRAIGHT_HEADING_KP = 0.65
STRAIGHT_MAX_ROLL_DEG = 12.0

# Eski sürümlerle isim uyumluluğu için tutulan irtifa sabitleri.
FULL_RECOVERY_ALT = 100.0
HARD_RECOVERY_ALT = 90.0

CONTROL_PERIOD = 0.05          # 20 Hz SET_ATTITUDE_TARGET
TELEMETRY_PRINT_PERIOD = 1.00  # Terminali sade tutmak için saniyede bir satır
POSITION_STALE_TIMEOUT = 3.0
HEARTBEAT_STALE_TIMEOUT = 4.0
ATTITUDE_STALE_TIMEOUT = 1.0
VFR_HUD_STALE_TIMEOUT = 2.0
GUIDED_ENTRY_GRACE = 2.5       # GUIDED geçişinden sonraki ilk heartbeat için tolerans

# Pilot bu modlardan birine geçerse harici kod hiçbir mod komutu göndermez.
PILOT_OVERRIDE_MODES = {"MANUAL", "FBWA", "FBWB", "CRUISE", "STABILIZE"}

# GUIDED rayına geri oturtma katmanı. Bu eşikler mod değiştirmez; belirgin
# yatış veya doğrultu kaçışında normal yanal PID geçici olarak devreden çıkar.
# Önce kanatlar yataylanır, sonra kilitli GPS doğrultusu yeniden yakalanır.
GUIDED_EARLY_GUARD_SEC = 3.0
GUIDED_EARLY_RESCUE_ROLL_DEG = 7.0
GUIDED_EARLY_RESCUE_ROLL_DURATION_SEC = 0.20
HOLD_RESCUE_ROLL_DEG = 10.0
HOLD_RESCUE_ROLL_DURATION_SEC = 0.35
DIVE_RESCUE_ROLL_DEG = 12.0
DIVE_RESCUE_ROLL_DURATION_SEC = 0.30
GUIDED_RESCUE_ROLL_RATE_DEG_PER_SEC = 30.0
GUIDED_RESCUE_ROLL_RATE_DURATION_SEC = 0.15
GUIDED_RESCUE_COURSE_ERROR_DEG = 12.0
GUIDED_RESCUE_COURSE_ERROR_DURATION_SEC = 0.60

# Rayına oturtma iki aşamalıdır: LEVEL -> REACQUIRE.
RAIL_LEVEL_ROLL_KP = 1.15
RAIL_LEVEL_ROLL_KD = 0.30
RAIL_LEVEL_COMMAND_MAX_DEG = 22.0
RAIL_REACQUIRE_HEADING_KP = 0.55
RAIL_REACQUIRE_DESIRED_ROLL_MAX_DEG = 10.0
RAIL_REACQUIRE_ROLL_KP = 1.20
RAIL_REACQUIRE_ROLL_KD = 0.28
RAIL_REACQUIRE_COMMAND_MAX_DEG = 20.0
RAIL_COMMAND_RATE_LIMIT_DEG_PER_SEC = 35.0
RAIL_LEVEL_COMPLETE_ROLL_DEG = 3.0
RAIL_LEVEL_COMPLETE_ROLL_RATE_DEG_PER_SEC = 8.0
RAIL_HEADING_COMPLETE_ERROR_DEG = 3.0
RAIL_CONFIRM_SEC = 1.0
RAIL_DIVE_MAX_NOSE_DOWN_DEG = -8.0
RAIL_MIN_THRUST = 0.65


# ============================================================
# GÖREV KOORDİNATLARI — KULLANICININ GÖNDERDİĞİ WP SETİ AYNEN KORUNMUŞTUR
# ============================================================

HOME = (36.579175, 36.151376)
WP1 = (37.98205709, 41.84042580 )
WP2 = (36.57950354, 36.15010367)
WP3 = (36.57984786, 36.15178199)
WP4 = (36.57917411, 36.15438110)
WP5 = (36.57853561, 36.15356391)
VIRTUAL_WP = (36.57889042, 36.15265801)  # WP5'ten yaklaşık 90 m sonra
DIVE_TARGET = (36.57936350, 36.15145013)  # Sanal WP'den yaklaşık 120 m sonra


# ============================================================
# GENEL YARDIMCILAR
# ============================================================


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def wrap_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (
        math.sin(dp / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    )
    return 2.0 * radius * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def destination_point(lat: float, lon: float, bearing: float, distance_m: float) -> tuple[float, float]:
    """Verilen konumdan belirli doğrultu ve mesafedeki GPS noktasını hesaplar."""
    radius = 6_371_000.0
    angular_distance = distance_m / radius
    bearing_rad = math.radians(bearing)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


# ANSI terminal renkleri. Çıktı dosyaya yönlendirilirse otomatik kapanır.
USE_COLOR = sys.stdout.isatty()
ANSI_RESET = "\033[0m"
ANSI_BOLD = "\033[1m"
ANSI_GREEN = "\033[92m"
ANSI_YELLOW = "\033[93m"
ANSI_RED = "\033[91m"
ANSI_CYAN = "\033[96m"
ANSI_MAGENTA = "\033[95m"
ANSI_NEON_PINK = "\033[38;5;201m"  # 256 renkli terminalde neon pembe
ANSI_DIM = "\033[2m"


def paint(text: str, color: str, bold: bool = False) -> str:
    if not USE_COLOR:
        return text
    prefix = (ANSI_BOLD if bold else "") + color
    return f"{prefix}{text}{ANSI_RESET}"


def phase_label(phase: str) -> str:
    colors = {
        "AUTO": ANSI_GREEN,
        "HOLD": ANSI_CYAN,
        "DIVE": ANSI_RED,
        "RECOVERY": ANSI_YELLOW,
        "STRAIGHT": ANSI_CYAN,
    }
    return paint(f"[{phase}]", colors.get(phase, ANSI_MAGENTA), bold=True)


def mission_result_name(result: int) -> str:
    names = {
        mavutil.mavlink.MAV_MISSION_ACCEPTED: "ACCEPTED",
        mavutil.mavlink.MAV_MISSION_ERROR: "ERROR",
        mavutil.mavlink.MAV_MISSION_UNSUPPORTED_FRAME: "UNSUPPORTED_FRAME",
        mavutil.mavlink.MAV_MISSION_UNSUPPORTED: "UNSUPPORTED",
        mavutil.mavlink.MAV_MISSION_NO_SPACE: "NO_SPACE",
        mavutil.mavlink.MAV_MISSION_INVALID: "INVALID",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM1: "INVALID_PARAM1",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM2: "INVALID_PARAM2",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM3: "INVALID_PARAM3",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM4: "INVALID_PARAM4",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM5_X: "INVALID_PARAM5_X",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM6_Y: "INVALID_PARAM6_Y",
        mavutil.mavlink.MAV_MISSION_INVALID_PARAM7: "INVALID_PARAM7",
        mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE: "INVALID_SEQUENCE",
        mavutil.mavlink.MAV_MISSION_DENIED: "DENIED",
        mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED: "OPERATION_CANCELLED",
    }
    return names.get(result, f"UNKNOWN_{result}")


# ============================================================
# BAĞLANTI
# ============================================================

print(f"ArduPilot seri bağlantısı bekleniyor | Port: {CONNECTION} | Baud: {BAUD}")
master = mavutil.mavlink_connection(
    CONNECTION,
    baud=BAUD,
    autoreconnect=True,
    source_system=SOURCE_SYSTEM,
    source_component=SOURCE_COMPONENT,
)

heartbeat = master.wait_heartbeat(timeout=15)
if heartbeat is None:
    raise TimeoutError("15 saniye içinde HEARTBEAT alınamadı.")

print(
    f"Bağlantı kuruldu | System: {master.target_system} | "
    f"Component: {master.target_component}"
)

MODE_MAPPING = master.mode_mapping()
MODE_BY_ID = {mode_id: name for name, mode_id in MODE_MAPPING.items()}
SCRIPT_START_MONOTONIC = time.monotonic()


# ============================================================
# TELEMETRİ DURUMU
# ============================================================

state: Dict[str, object] = {
    "armed": False,
    "mode": "UNKNOWN",
    "system_status": None,
    "last_heartbeat": 0.0,
    "last_position": 0.0,
    "last_attitude": 0.0,
    "last_vfr_hud": 0.0,
    "lat": None,
    "lon": None,
    "rel_alt": None,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "groundspeed": 0.0,
    "airspeed": 0.0,
    "throttle_pct": 0.0,
    "roll": 0.0,
    "roll_rate": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "mission_seq": None,
    "failsafe": False,
    "failsafe_text": "",
}

# Son gönderilen GUIDED komutları terminalde gösterilir.
command_state: Dict[str, float] = {
    "roll": 0.0,
    "roll_rate": 0.0,
    "pitch": 0.0,
    "rudder": 0.0,
    "thrust": 0.0,
}


def request_message_interval(message_id: int, frequency_hz: float) -> None:
    interval_us = int(1_000_000 / frequency_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0, 0, 0, 0, 0,
    )


def request_telemetry() -> None:
    requests = [
        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 5.0),
    ]
    for message_id, rate in requests:
        request_message_interval(message_id, rate)
        time.sleep(0.03)


def decode_statustext(message) -> str:
    text = message.text
    if isinstance(text, bytes):
        text = text.decode("utf-8", errors="replace")
    return str(text).rstrip("\x00").strip()


def process_message(message) -> None:
    now = time.monotonic()
    message_type = message.get_type()

    if message_type == "HEARTBEAT":
        if message.get_srcSystem() != master.target_system:
            return
        state["armed"] = bool(
            message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        state["mode"] = MODE_BY_ID.get(message.custom_mode, f"MODE_{message.custom_mode}")
        state["system_status"] = message.system_status
        state["last_heartbeat"] = now

    elif message_type == "GLOBAL_POSITION_INT":
        state["lat"] = message.lat / 1e7
        state["lon"] = message.lon / 1e7
        state["rel_alt"] = message.relative_alt / 1000.0
        state["vx"] = message.vx / 100.0
        state["vy"] = message.vy / 100.0
        state["vz"] = message.vz / 100.0  # NED: pozitif değer aşağı yönlüdür
        state["groundspeed"] = math.hypot(state["vx"], state["vy"])
        state["last_position"] = now

    elif message_type == "ATTITUDE":
        state["roll"] = math.degrees(message.roll)
        state["roll_rate"] = math.degrees(message.rollspeed)
        state["pitch"] = math.degrees(message.pitch)
        state["yaw"] = (math.degrees(message.yaw) + 360.0) % 360.0
        state["last_attitude"] = now

    elif message_type == "VFR_HUD":
        state["airspeed"] = float(message.airspeed)
        state["groundspeed"] = float(message.groundspeed)
        state["throttle_pct"] = float(message.throttle)
        state["last_vfr_hud"] = now

    elif message_type == "MISSION_CURRENT":
        state["mission_seq"] = int(message.seq)

    elif message_type == "STATUSTEXT":
        text = decode_statustext(message)
        upper = text.upper()
        print("\n" + paint(f"[AP] {text}", ANSI_DIM))

        active_failsafe = (
            "FAILSAFE ON" in upper
            or "LONG FAILSAFE" in upper
            or "FAILSAFE: RTL" in upper
        )
        cleared_failsafe = "FAILSAFE CLEARED" in upper or "FAILSAFE OFF" in upper

        if active_failsafe and not cleared_failsafe:
            state["failsafe"] = True
            state["failsafe_text"] = text
        elif cleared_failsafe:
            state["failsafe"] = False
            state["failsafe_text"] = ""


def receive_available_messages(max_messages: int = 100) -> None:
    for _ in range(max_messages):
        message = master.recv_match(blocking=False)
        if message is None:
            break
        process_message(message)


def wait_for_initial_state(timeout: float = 12.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = master.recv_match(blocking=True, timeout=0.5)
        if message is not None:
            process_message(message)

        if (
            state["last_heartbeat"]
            and state["last_position"]
            and state["last_attitude"]
            and state["last_vfr_hud"]
            and state["lat"] is not None
        ):
            return

    raise TimeoutError("Başlangıç telemetrisi eksik: HEARTBEAT/GPS/ATTITUDE alınamadı.")


request_telemetry()
wait_for_initial_state()


# ============================================================
# BAŞLANGIÇ GÜVENLİK KONTROLLERİ
# ============================================================

start_mode = str(state["mode"])
start_alt = float(state["rel_alt"])

print(
    f"Başlangıç kontrolü | ARM: {state['armed']} | "
    f"Mod: {start_mode} | İrtifa: {start_alt:.1f} m"
)

if not bool(state["armed"]):
    raise RuntimeError("Araç ARM değil. Kod ARM komutu göndermez.")

if start_alt < MIN_START_ALT:
    raise RuntimeError(
        f"İrtifa yetersiz: {start_alt:.1f} m. En az {MIN_START_ALT:.0f} m gerekli."
    )

print(f"Başlangıç modu kabul edildi: {start_mode}. Görev yüklenince AUTO moduna geçilecek.")

if bool(state["failsafe"]):
    raise RuntimeError(f"Aktif failsafe var: {state['failsafe_text']}")


# ============================================================
# KADEMELİ İRTİFA PLANI
# ============================================================

start_alt = clamp(start_alt, MIN_START_ALT, CRUISE_ALT)
step = (CRUISE_ALT - start_alt) / 5.0

wp_altitudes = [
    start_alt + step,
    start_alt + 2.0 * step,
    start_alt + 3.0 * step,
    start_alt + 4.0 * step,
    CRUISE_ALT,
]

print("\nKademeli görev irtifaları:")
for index, altitude in enumerate(wp_altitudes, start=1):
    print(f"WP{index}: {altitude:.1f} m")
print(f"Sanal WP: {CRUISE_ALT:.1f} m")
print(f"Dalış WP: {CRUISE_ALT:.1f} m")


# ============================================================
# MİSYON MADDELERİ
# seq 0 HOME, seq 1-5 WP, seq 6 sanal WP, seq 7 dalış konumu, seq 8 LOITER_UNLIM
# ============================================================

MissionItem = Dict[str, object]


def nav_item(
    name: str,
    lat: float,
    lon: float,
    alt: float,
    command: int = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    param1: float = 0.0,
    param2: float = 30.0,
    param3: float = 0.0,
    param4: float = 0.0,
) -> MissionItem:
    return {
        "name": name,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "command": command,
        "param1": param1,
        "param2": param2,
        "param3": param3,
        "param4": param4,
    }


mission_items: List[MissionItem] = [
    nav_item("HOME", HOME[0], HOME[1], start_alt),
    nav_item("WP1", WP1[0], WP1[1], wp_altitudes[0]),
    nav_item("WP2", WP2[0], WP2[1], wp_altitudes[1]),
    nav_item("WP3", WP3[0], WP3[1], wp_altitudes[2]),
    nav_item("WP4", WP4[0], WP4[1], wp_altitudes[3]),
    nav_item("WP5", WP5[0], WP5[1], wp_altitudes[4]),
    nav_item("SANAL_WP", VIRTUAL_WP[0], VIRTUAL_WP[1], CRUISE_ALT),
    nav_item("DALIS_WP", DIVE_TARGET[0], DIVE_TARGET[1], CRUISE_ALT),
    nav_item(
        "EMNIYET_LOITER",
        DIVE_TARGET[0],
        DIVE_TARGET[1],
        CRUISE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        param3=120.0,
    ),
]


# ============================================================
# MİSYON PROTOKOLÜ
# ============================================================


def addressed_to_this_client(message) -> bool:
    target_system = getattr(message, "target_system", SOURCE_SYSTEM)
    target_component = getattr(message, "target_component", SOURCE_COMPONENT)
    return target_system in (0, SOURCE_SYSTEM) and target_component in (0, SOURCE_COMPONENT)


def drain_mission_messages(duration: float = 0.4) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        message = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=False,
        )
        if message is None:
            time.sleep(0.01)


def clear_mission(max_attempts: int = 5) -> None:
    print("\nÖnceki görev temizleniyor...")
    drain_mission_messages()

    for attempt in range(1, max_attempts + 1):
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        deadline = time.monotonic() + 1.8
        while time.monotonic() < deadline:
            message = master.recv_match(
                type=["MISSION_ACK", "STATUSTEXT"],
                blocking=True,
                timeout=0.3,
            )
            if message is None:
                continue
            if message.get_type() == "STATUSTEXT":
                process_message(message)
                continue
            if not addressed_to_this_client(message):
                continue

            result = int(message.type)
            if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Önceki görev temizlendi.")
                return

            raise RuntimeError(
                f"Görev temizleme reddedildi: {mission_result_name(result)}"
            )

        print(f"Görev temizleme ACK bekleniyor... deneme {attempt}/{max_attempts}")

    raise TimeoutError("MISSION_CLEAR_ALL için MISSION_ACK alınamadı.")


def send_mission_item(sequence: int) -> None:
    item = mission_items[sequence]

    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        sequence,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(item["command"]),
        1 if sequence == 0 else 0,
        1,
        float(item["param1"]),
        float(item["param2"]),
        float(item["param3"]),
        float(item["param4"]),
        int(round(float(item["lat"]) * 1e7)),
        int(round(float(item["lon"]) * 1e7)),
        float(item["alt"]),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )


def upload_mission(max_sessions: int = 3) -> None:
    print("Yeni görev yükleniyor...")

    for session in range(1, max_sessions + 1):
        drain_mission_messages()

        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            len(mission_items),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        session_deadline = time.monotonic() + 25.0
        last_protocol_activity = time.monotonic()
        timeout_retries = 0
        printed_sequences = set()

        while time.monotonic() < session_deadline:
            message = master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "STATUSTEXT"],
                blocking=True,
                timeout=0.35,
            )

            if message is None:
                if time.monotonic() - last_protocol_activity >= 1.5:
                    timeout_retries += 1
                    if timeout_retries > 5:
                        break
                    master.mav.mission_count_send(
                        master.target_system,
                        master.target_component,
                        len(mission_items),
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    last_protocol_activity = time.monotonic()
                continue

            if message.get_type() == "STATUSTEXT":
                process_message(message)
                continue

            if not addressed_to_this_client(message):
                continue

            message_type = message.get_type()
            last_protocol_activity = time.monotonic()
            timeout_retries = 0

            if message_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                sequence = int(message.seq)
                if not 0 <= sequence < len(mission_items):
                    raise RuntimeError(f"Geçersiz mission sequence istendi: {sequence}")

                # Güncel MAVLink protokolünde legacy MISSION_REQUEST alınsa bile
                # MISSION_ITEM_INT ile cevap vermek geçerlidir.
                send_mission_item(sequence)

                if sequence not in printed_sequences:
                    item = mission_items[sequence]
                    print(
                        f"SEQ {sequence}: {item['name']} gönderildi | "
                        f"İrtifa: {float(item['alt']):.1f} m"
                    )
                    printed_sequences.add(sequence)

            elif message_type == "MISSION_ACK":
                result = int(message.type)
                if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("Görev ArduPilot tarafından kabul edildi.")
                    return

                if result == mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    print("Mission işlemi iptal edildi; yükleme yeniden başlatılıyor.")
                    break

                raise RuntimeError(
                    f"Görev yükleme reddedildi: {mission_result_name(result)}"
                )

        print(f"Görev yükleme oturumu yenileniyor: {session}/{max_sessions}")
        time.sleep(0.5)

    raise TimeoutError("Görev yükleme tamamlanamadı; MISSION_ACK: ACCEPTED alınamadı.")


# ============================================================
# MOD VE GÖREV SIRASI KONTROLÜ
# ============================================================


def set_mode(mode_name: str, timeout: float = 8.0) -> None:
    mode_name = mode_name.upper()
    if mode_name not in MODE_MAPPING:
        raise ValueError(f"{mode_name} modu bulunamadı.")

    mode_id = MODE_MAPPING[mode_name]
    deadline = time.monotonic() + timeout
    next_send = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0, 0, 0, 0, 0,
            )
            next_send = now + 0.75

        message = master.recv_match(blocking=True, timeout=0.25)
        if message is not None:
            process_message(message)

        if str(state["mode"]) == mode_name:
            print(f"Mod değiştirildi: {mode_name}")
            return

    raise TimeoutError(f"{mode_name} moduna geçilemedi.")


def set_current_mission(sequence: int, timeout: float = 8.0) -> None:
    print(f"Aktif görev sırası SEQ {sequence} olarak ayarlanıyor...")
    deadline = time.monotonic() + timeout
    next_send = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            master.mav.mission_set_current_send(
                master.target_system,
                master.target_component,
                sequence,
            )
            next_send = now + 1.0

        message = master.recv_match(blocking=True, timeout=0.25)
        if message is not None:
            process_message(message)

        if state["mission_seq"] == sequence:
            print(f"Aktif görev sırası doğrulandı: SEQ {sequence}")
            return

    raise TimeoutError(f"MISSION_CURRENT SEQ {sequence} olarak doğrulanamadı.")


def safe_rtl(reason: str) -> None:
    print(f"\nRTL isteği: {reason}")
    try:
        if str(state["mode"]) != "RTL":
            set_mode("RTL", timeout=6.0)
    except Exception as rtl_error:
        print(f"RTL komutu doğrulanamadı: {rtl_error}")


def emergency_rtl_without_confirmation(reason: str) -> None:
    """Telemetri kesildiğinde RTL komutunu doğrulama beklemeden tekrarlar."""
    print(f"\nAcil RTL isteği: {reason}")
    rtl_mode_id = MODE_MAPPING.get("RTL")
    if rtl_mode_id is None:
        print("RTL modu mode mapping içinde bulunamadı.")
        return
    for _ in range(3):
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            rtl_mode_id,
        )
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            rtl_mode_id,
            0, 0, 0, 0, 0,
        )
        time.sleep(0.15)
    print("RTL komutu 3 kez gönderildi; telemetri olmadığı için mod doğrulanamadı.")


def pilot_took_control(mode: str, phase_name: str) -> bool:
    """Pilotun seçtiği uçuş moduna dokunmadan harici kontrolü bırakır."""
    if mode in PILOT_OVERRIDE_MODES:
        print(
            f"\nPilot müdahalesi algılandı: {mode} | Faz: {phase_name}. "
            "Harici GUIDED kontrolü sonlandırılıyor; pilot modu korunuyor."
        )
        return True
    return False


guided_safety_since: Dict[str, Optional[float]] = {
    "roll": None,
    "roll_rate": None,
    "course": None,
}


def reset_guided_lateral_safety() -> None:
    for key in guided_safety_since:
        guided_safety_since[key] = None


def guided_lateral_rescue_reason(
    phase_name: str,
    target_heading_deg: Optional[float],
    phase_elapsed_sec: float,
    now: float,
) -> Optional[str]:
    """Rayına oturtma katmanını başlatacak yanal sapmayı erkenden yakalar."""
    actual_roll = float(state["roll"])
    actual_roll_rate = float(state["roll_rate"])

    if phase_name == "HOLD" and phase_elapsed_sec <= GUIDED_EARLY_GUARD_SEC:
        roll_limit = GUIDED_EARLY_RESCUE_ROLL_DEG
        roll_duration = GUIDED_EARLY_RESCUE_ROLL_DURATION_SEC
    elif phase_name == "HOLD":
        roll_limit = HOLD_RESCUE_ROLL_DEG
        roll_duration = HOLD_RESCUE_ROLL_DURATION_SEC
    else:
        roll_limit = DIVE_RESCUE_ROLL_DEG
        roll_duration = DIVE_RESCUE_ROLL_DURATION_SEC

    checks = {
        "roll": (abs(actual_roll) > roll_limit, roll_duration),
        # Küçük açı çevresindeki sensör gürültüsünü ani yatış sayma.
        "roll_rate": (
            abs(actual_roll) > 2.0
            and abs(actual_roll_rate) > GUIDED_RESCUE_ROLL_RATE_DEG_PER_SEC,
            GUIDED_RESCUE_ROLL_RATE_DURATION_SEC,
        ),
    }

    live_course = ground_course_deg()
    course_error = None
    if target_heading_deg is not None and live_course is not None:
        course_error = wrap_180(target_heading_deg - live_course)
    checks["course"] = (
        course_error is not None
        and abs(course_error) > GUIDED_RESCUE_COURSE_ERROR_DEG,
        GUIDED_RESCUE_COURSE_ERROR_DURATION_SEC,
    )

    for key, (condition, duration) in checks.items():
        if condition:
            if guided_safety_since[key] is None:
                guided_safety_since[key] = now
            elif now - float(guided_safety_since[key]) >= duration:
                if key == "roll":
                    return (
                        f"{phase_name} roll {actual_roll:+.1f}°; "
                        f"sınır {roll_limit:.1f}°/{duration:.2f} s"
                    )
                if key == "roll_rate":
                    return (
                        f"{phase_name} ani yatış hızı {actual_roll_rate:+.1f}°/s; "
                        f"sınır {GUIDED_RESCUE_ROLL_RATE_DEG_PER_SEC:.0f}°/s"
                    )
                return (
                    f"{phase_name} doğrultu kaçışı {float(course_error):+.1f}°; "
                    f"sınır {GUIDED_RESCUE_COURSE_ERROR_DEG:.0f}°"
                )
        else:
            guided_safety_since[key] = None

    return None


guided_rail_state: Dict[str, object] = {
    "active": False,
    "stage": "IDLE",
    "trigger": "",
    "aligned_since": None,
}


def reset_guided_rail_recovery() -> None:
    guided_rail_state.update(
        {"active": False, "stage": "IDLE", "trigger": "", "aligned_since": None}
    )


def start_guided_rail_recovery(reason: str, phase_name: str) -> None:
    """GUIDED'da kalıp önce kanatları yataylar, sonra kilitli doğrultuyu yakalar."""
    if bool(guided_rail_state["active"]):
        return
    guided_rail_state.update(
        {
            "active": True,
            "stage": "LEVEL",
            "trigger": reason,
            "aligned_since": None,
        }
    )
    reset_guided_lateral_safety()
    print(
        f"\n[RAY KURTARMA] {reason} | Faz: {phase_name} | GUIDED korunuyor.\n"
        "[RAY KURTARMA] Aşama 1: kanatlar yataylanıyor; dalış geçici yumuşatılıyor."
    )


def guided_rail_recovery_command(
    target_heading_deg: Optional[float],
    now: float,
) -> tuple[float, bool, str]:
    """İki aşamalı rayına oturtma roll hedefini üretir.

    Dönüş: (ham roll komutu, bu çevrimde tamamlandı mı, aşama adı)
    """
    actual_roll = float(state["roll"])
    roll_rate = float(state["roll_rate"])
    live_course = ground_course_deg()
    if live_course is None:
        live_course = float(state["yaw"])
    heading_error = (
        0.0
        if target_heading_deg is None
        else wrap_180(float(target_heading_deg) - live_course)
    )

    stage = str(guided_rail_state["stage"])
    completed = False

    if stage == "LEVEL":
        command = clamp(
            -(RAIL_LEVEL_ROLL_KP * actual_roll + RAIL_LEVEL_ROLL_KD * roll_rate),
            -RAIL_LEVEL_COMMAND_MAX_DEG,
            RAIL_LEVEL_COMMAND_MAX_DEG,
        )
        if (
            abs(actual_roll) <= RAIL_LEVEL_COMPLETE_ROLL_DEG
            and abs(roll_rate) <= RAIL_LEVEL_COMPLETE_ROLL_RATE_DEG_PER_SEC
        ):
            guided_rail_state.update(
                {"stage": "REACQUIRE", "aligned_since": None}
            )
            stage = "REACQUIRE"
            print(
                "[RAY KURTARMA] Aşama 2: kanatlar yatay; kilitli dalış "
                "doğrultusu yeniden yakalanıyor."
            )
    else:
        desired_roll = clamp(
            RAIL_REACQUIRE_HEADING_KP * heading_error,
            -RAIL_REACQUIRE_DESIRED_ROLL_MAX_DEG,
            RAIL_REACQUIRE_DESIRED_ROLL_MAX_DEG,
        )
        roll_error = desired_roll - actual_roll
        command = clamp(
            desired_roll
            + RAIL_REACQUIRE_ROLL_KP * roll_error
            - RAIL_REACQUIRE_ROLL_KD * roll_rate,
            -RAIL_REACQUIRE_COMMAND_MAX_DEG,
            RAIL_REACQUIRE_COMMAND_MAX_DEG,
        )
        aligned = (
            abs(heading_error) <= RAIL_HEADING_COMPLETE_ERROR_DEG
            and abs(actual_roll) <= RAIL_LEVEL_COMPLETE_ROLL_DEG
            and abs(roll_rate) <= RAIL_LEVEL_COMPLETE_ROLL_RATE_DEG_PER_SEC
        )
        if aligned:
            if guided_rail_state["aligned_since"] is None:
                guided_rail_state["aligned_since"] = now
            elif now - float(guided_rail_state["aligned_since"]) >= RAIL_CONFIRM_SEC:
                completed = True
                guided_rail_state.update(
                    {"active": False, "stage": "IDLE", "aligned_since": None}
                )
                reset_guided_lateral_safety()
                reset_lateral_pid(target_heading_deg)
                print(
                    f"[RAY KURTARMA] Tamamlandı | HDG hata: {heading_error:+.1f}° | "
                    f"Roll: {actual_roll:+.1f}°. Normal GUIDED kontrolüne dönülüyor."
                )
        else:
            guided_rail_state["aligned_since"] = None

    return command, completed, stage


# ============================================================
# GUIDED ATTITUDE KONTROLÜ
# ============================================================


def set_attitude_target(
    roll_deg: float,
    pitch_deg: float,
    thrust: float,
) -> None:
    """ArduPlane'e roll, pitch, nötr dümen ve gaz hedefi gönderir.

    ArduPlane quaternion yaw açısını bir heading hedefi gibi kullanmaz;
    forced_rpy_cd.z üzerinden doğrudan dümen talebine dönüştürür. Bu nedenle
    gerçek yaw değeri quaternion'a kesinlikle yazılmaz. Yaw=0.0, dümeni nötr
    tutar; doğrultu düzeltmesi yalnızca roll/bank kontrolüyle yapılır.
    """
    command_state["roll"] = float(roll_deg)
    command_state["pitch"] = float(pitch_deg)
    command_state["rudder"] = 0.0
    command_state["thrust"] = clamp(float(thrust), 0.0, 1.0)

    quaternion = QuaternionBase(
        [
            math.radians(roll_deg),
            math.radians(pitch_deg),
            0.0,  # ArduPlane forced_rpy_cd.z = 0 -> nötr dümen
        ]
    )

    time_boot_ms = int((time.monotonic() - SCRIPT_START_MONOTONIC) * 1000.0) & 0xFFFFFFFF

    master.mav.set_attitude_target_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        0b00000111,  # ArduPlane: quaternion ve thrust aktif
        quaternion,
        0.0,
        0.0,
        0.0,
        command_state["thrust"],
    )


def ground_course_deg() -> Optional[float]:
    """GPS hız vektöründen yer iz açısını hesaplar: 0° kuzey, 90° doğu."""
    vx_north = float(state["vx"])
    vy_east = float(state["vy"])
    if math.hypot(vx_north, vy_east) < 1.0:
        return None
    return (math.degrees(math.atan2(vy_east, vx_north)) + 360.0) % 360.0


def flight_path_down_angle_deg() -> Optional[float]:
    horizontal_speed = math.hypot(float(state["vx"]), float(state["vy"]))
    if horizontal_speed < 1.0:
        return None
    return math.degrees(math.atan2(float(state["vz"]), horizontal_speed))


def predicted_pullout_trigger_altitude() -> float:
    """Karşılaştırma amaçlı eski dinamik pull-out başlangıcını hesaplar.

    Bu 50 m doğrudan tetik sürümünün ana döngüsünde kullanılmaz.

    NED dikey hızında pozitif değer aşağı yönlüdür. Pull-out boyunca dikey hızın
    yaklaşık doğrusal olarak sıfıra indiği kabul edilerek kayıp 0.5*vz*t alınır.
    """
    descent_rate = max(float(state["vz"]), 0.0)
    predicted_loss = 0.5 * descent_rate * PULLOUT_RESPONSE_TIME_SEC
    trigger_alt = DIVE_END_ALT + predicted_loss + PULLOUT_SAFETY_MARGIN_M
    return clamp(trigger_alt, PULLOUT_TRIGGER_MIN_ALT, PULLOUT_TRIGGER_MAX_ALT)


def recovery_pitch_schedule(path_angle_deg: Optional[float]) -> float:
    """Düz pull-out için uçuş-yolu açısına bağlı sınırlı pitch hedefi."""
    if path_angle_deg is None:
        return 8.0
    if path_angle_deg > 35.0:
        return 8.0
    if path_angle_deg > 18.0:
        return 10.0
    if path_angle_deg > RECOVERY_PATH_LEVEL_THRESHOLD_DEG:
        return 7.0
    if path_angle_deg > -6.0:
        return 10.0
    return RECOVERY_TARGET_PITCH_DEG


def recovery_thrust_from_actual_pitch(actual_pitch_deg: float) -> float:
    # -50 derece ve daha düşük pitchte %20; 0 derece ve üstünde %100.
    normalized = (actual_pitch_deg + 50.0) / 50.0
    return clamp(0.20 + 0.80 * normalized, 0.20, 1.00)


def smoothstep01(value: float) -> float:
    """0..1 aralığında sarsıntısız geçiş eğrisi."""
    x = clamp(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def rate_limit(
    target: float,
    previous: float,
    rate_per_second: float,
    dt: float,
) -> float:
    """Komut değişim hızını sınırlar."""
    max_step = max(rate_per_second, 0.0) * max(dt, 0.0)
    return previous + clamp(target - previous, -max_step, max_step)


def asymmetric_rate_limit(
    target: float,
    previous: float,
    decreasing_rate: float,
    increasing_rate: float,
    dt: float,
) -> float:
    """Negatif ve pozitif yöndeki komut değişim hızlarını ayrı sınırlar."""
    rate = increasing_rate if target >= previous else decreasing_rate
    return rate_limit(target, previous, rate, dt)


# Yanal PID durumları. Heading/yer izi dış PID, roll ise iç PID'dir.
lateral_pid_state: Dict[str, Optional[float]] = {
    "last_time": None,
    "course_integral": 0.0,
    "course_previous_error": 0.0,
    "course_derivative_filtered": 0.0,
    "roll_integral": 0.0,
    "roll_previous_error": 0.0,
    "roll_derivative_filtered": 0.0,
}


def reset_lateral_pid(target_heading_deg: Optional[float]) -> None:
    current_heading = ground_course_deg()
    if current_heading is None:
        current_heading = float(state["yaw"])
    heading_error = (
        0.0
        if target_heading_deg is None
        else wrap_180(float(target_heading_deg) - current_heading)
    )
    roll_error = -float(state["roll"])
    lateral_pid_state.update(
        {
            "last_time": time.monotonic(),
            "course_integral": 0.0,
            "course_previous_error": heading_error,
            "course_derivative_filtered": 0.0,
            "roll_integral": 0.0,
            "roll_previous_error": roll_error,
            "roll_derivative_filtered": 0.0,
        }
    )


def lateral_pid_command(
    target_heading_deg: Optional[float],
    control_phase: str,
) -> float:
    """Yer izini korur ve kanatları yatay tutar.

    Dış PID sınırlı bir hedef bank üretir. Uçak belirgin şekilde yatmışsa
    heading düzeltmesi tamamen kapatılmaz; azaltılmış biçimde korunurken iç roll
    PID kanatları yeniden hedef banka taşır. Yaw komutu üretilmez.
    """
    phase_name = control_phase.upper()
    if phase_name == "DIVE":
        course_kp, course_ki, course_kd = (
            DIVE_COURSE_KP,
            DIVE_COURSE_KI,
            DIVE_COURSE_KD,
        )
        desired_roll_max = DIVE_DESIRED_ROLL_MAX_DEG
        command_max = DIVE_ROLL_COMMAND_MAX_DEG
    elif phase_name == "RECOVERY":
        course_kp, course_ki, course_kd = (
            RECOVERY_COURSE_KP,
            RECOVERY_COURSE_KI,
            RECOVERY_COURSE_KD,
        )
        desired_roll_max = RECOVERY_DESIRED_ROLL_MAX_DEG
        command_max = RECOVERY_ROLL_COMMAND_MAX_DEG
    else:
        course_kp, course_ki, course_kd = (
            STRAIGHT_COURSE_KP,
            STRAIGHT_COURSE_KI,
            STRAIGHT_COURSE_KD,
        )
        desired_roll_max = STRAIGHT_DESIRED_ROLL_MAX_DEG
        command_max = STRAIGHT_ROLL_COMMAND_MAX_DEG

    now = time.monotonic()
    previous_time = lateral_pid_state["last_time"]
    dt = CONTROL_PERIOD if previous_time is None else clamp(now - float(previous_time), 0.01, 0.20)

    current_heading = ground_course_deg()
    if current_heading is None:
        current_heading = float(state["yaw"])

    heading_error = (
        0.0
        if target_heading_deg is None
        else wrap_180(float(target_heading_deg) - current_heading)
    )
    previous_heading_error = float(lateral_pid_state["course_previous_error"] or 0.0)
    raw_course_derivative = wrap_180(heading_error - previous_heading_error) / dt
    course_alpha = dt / (COURSE_D_FILTER_TAU + dt)
    course_derivative = float(lateral_pid_state["course_derivative_filtered"] or 0.0)
    course_derivative += course_alpha * (raw_course_derivative - course_derivative)

    actual_roll = float(state["roll"])
    roll_rate = float(state["roll_rate"])
    course_integral = float(lateral_pid_state["course_integral"] or 0.0)

    # Heading hedefi her durumda hesaplanır. Uçak 4°'den fazla yatmışsa
    # düzeltme sıfırlanmaz; sadece azaltılır. Böylece kanatlar toparlanırken
    # uçak yanlış yöne serbestçe sürüklenmeye devam etmez.
    if abs(actual_roll) < LATERAL_LEVEL_PRIORITY_DEG:
        course_integral = clamp(
            course_integral + heading_error * dt,
            -COURSE_INTEGRAL_LIMIT,
            COURSE_INTEGRAL_LIMIT,
        )
        heading_authority = 1.0
    else:
        course_integral *= 0.98
        heading_authority = LATERAL_HEADING_RETAIN_RATIO

    desired_roll = clamp(
        heading_authority
        * (
            course_kp * heading_error
            + course_ki * course_integral
            + course_kd * course_derivative
        ),
        -desired_roll_max,
        desired_roll_max,
    )

    roll_error = desired_roll - actual_roll
    previous_roll_error = float(lateral_pid_state["roll_previous_error"] or 0.0)
    raw_roll_derivative = (roll_error - previous_roll_error) / dt
    roll_alpha = dt / (ROLL_D_FILTER_TAU + dt)
    roll_derivative = float(lateral_pid_state["roll_derivative_filtered"] or 0.0)
    roll_derivative += roll_alpha * (raw_roll_derivative - roll_derivative)

    roll_integral = clamp(
        float(lateral_pid_state["roll_integral"] or 0.0) + roll_error * dt,
        -ROLL_INTEGRAL_LIMIT,
        ROLL_INTEGRAL_LIMIT,
    )

    # roll_rate doğrudan sönüm olarak da eklenir; V-kuyruk/yük asimetrisi bastırılır.
    command = (
        desired_roll
        + ROLL_PID_KP * roll_error
        + ROLL_PID_KI * roll_integral
        + ROLL_PID_KD * roll_derivative
        - 0.05 * roll_rate
    )
    command = clamp(command, -command_max, command_max)

    lateral_pid_state.update(
        {
            "last_time": now,
            "course_integral": course_integral,
            "course_previous_error": heading_error,
            "course_derivative_filtered": course_derivative,
            "roll_integral": roll_integral,
            "roll_previous_error": roll_error,
            "roll_derivative_filtered": roll_derivative,
        }
    )
    return command


def dive_roll_command(target_heading_deg: Optional[float]) -> float:
    """Dalış için kademeli yer-izi + roll PID komutu."""
    return lateral_pid_command(target_heading_deg, "DIVE")


def roll_level_command(
    kp: float = ROLL_LEVEL_KP,
    kd: float = ROLL_LEVEL_KD,
    max_command_deg: float = ROLL_LEVEL_MAX_CMD_DEG,
) -> float:
    """Gerçek roll ve roll hızına göre kanatları 0°'ye döndüren dış PD komutu."""
    actual_roll = float(state["roll"])
    roll_rate = float(state["roll_rate"])
    command = -(kp * actual_roll + kd * roll_rate)
    return clamp(command, -max_command_deg, max_command_deg)


def stabilized_roll_command(
    target_heading_deg: Optional[float],
    kp: float = ROLL_LEVEL_KP,
    kd: float = ROLL_LEVEL_KD,
    max_command_deg: float = ROLL_LEVEL_MAX_CMD_DEG,
    heading_kp: float = DIVE_HEADING_ROLL_KP,
    heading_max_deg: float = DIVE_HEADING_ROLL_MAX_DEG,
) -> float:
    """Heading hatasından sınırlı hedef roll üretir ve gerçek rollü o hedefe taşır.

    Sabit kanatlı uçakta yön, yaw zorlanarak değil kontrollü bank/roll ile tutulur.
    Heading hatası yoksa hedef roll 0° olur ve gerçek roll doğrudan sıfırlanır.
    """
    desired_roll = 0.0

    if target_heading_deg is not None:
        current_heading = ground_course_deg()
        if current_heading is None:
            current_heading = float(state["yaw"])
        heading_error = wrap_180(float(target_heading_deg) - current_heading)
        desired_roll = clamp(
            heading_kp * heading_error,
            -heading_max_deg,
            heading_max_deg,
        )

    actual_roll = float(state["roll"])
    roll_rate = float(state["roll_rate"])
    roll_error = desired_roll - actual_roll

    command = desired_roll + kp * roll_error - kd * roll_rate
    return clamp(command, -max_command_deg, max_command_deg)


def compact_status(
    phase: str,
    mode: str,
    seq: object,
    distance_m: float,
    altitude_m: float,
    actual_pitch_deg: float,
    path_angle_deg: Optional[float],
    course_deg: Optional[float],
    target_course_deg: Optional[float],
) -> str:
    airspeed = float(state["airspeed"])
    thrust_pct = int(round(command_state["thrust"] * 100.0))
    pitch_cmd = command_state["pitch"]
    pitch_tracking_error = pitch_cmd - actual_pitch_deg
    roll_cmd = command_state["roll"]

    if phase == "AUTO":
        return (
            f"{phase_label(phase)} WP {str(seq):>2} | "
            f"ALT {altitude_m:6.1f} m | DIST {distance_m:6.1f} m | "
            f"AIR {airspeed:4.1f} m/s | MODE {mode}"
        )

    path_text = "---" if path_angle_deg is None else f"{path_angle_deg:5.1f}°"
    course_text = "---" if course_deg is None else f"{course_deg:5.1f}°"
    target_text = "---" if target_course_deg is None else f"{target_course_deg:5.1f}°"
    heading_error_text = (
        "---"
        if course_deg is None or target_course_deg is None
        else f"{wrap_180(target_course_deg - course_deg):+5.1f}°"
    )

    pitch_segment = paint(
        f"PITCH {actual_pitch_deg:6.1f}→{pitch_cmd:5.1f}°",
        ANSI_NEON_PINK,
        bold=True,
    )

    actual_roll_deg = float(state["roll"])
    actual_yaw_deg = float(state["yaw"])
    rudder_cmd = command_state["rudder"]
    alignment_text = ""
    if phase == "HOLD":
        confirmed_for = (
            0.0
            if hold_alignment_since is None
            else min(time.monotonic() - hold_alignment_since, GUIDED_ALIGN_CONFIRM_SEC)
        )
        alignment_text = (
            f" | DOĞRULTU ONAY {confirmed_for:3.1f}/{GUIDED_ALIGN_CONFIRM_SEC:.1f} s"
        )

    return (
        f"{phase_label(phase)} ALT {altitude_m:6.1f} m | "
        f"{pitch_segment} ERR {pitch_tracking_error:+5.1f}° | "
        f"PATH↓ {path_text} | AIR {airspeed:4.1f} | "
        f"HDG {course_text}/{target_text} ERR {heading_error_text} | "
        f"ROLL GERÇEK/KOMUT {actual_roll_deg:5.1f}→{roll_cmd:5.1f}° | "
        f"YAW {actual_yaw_deg:5.1f}° SERBEST | DÜMEN KOMUT {rudder_cmd:+4.1f}° NÖTR | "
        f"THR {thrust_pct:3d}%{alignment_text}"
    )


# ============================================================
# GÖREVİ BAŞLAT
# ============================================================

try:
    clear_mission()
    time.sleep(0.4)
    upload_mission()
    time.sleep(0.5)
    set_current_mission(1)
    set_mode("AUTO")
except Exception as startup_error:
    print(f"\nGörev başlatma hatası: {startup_error}")
    safe_rtl("Görev yükleme/AUTO başlatma işlemi tamamlanamadı")
    raise

print("\nAUTO görev takibi başladı.")
print(
    "WP1-WP5 ve sanal WP AUTO modunda izlenecek; "
    f"dalış hedefine {GUIDED_ENTRY_DISTANCE_MIN_M:.0f}-"
    f"{GUIDED_ENTRY_DISTANCE_MAX_M:.0f} m kala doğrultu uygunsa GUIDED hazırlık başlayacak."
)
print(
    f"Dalış şartı: en az {GUIDED_HOLD_DURATION_SEC:.0f} s HOLD + "
    f"son {GUIDED_ALIGN_CONFIRM_SEC:.0f} s kesintisiz doğrultu/roll doğrulaması. "
    "Yaw serbest, dümen komutu 0.0° nötr.\n"
)


# ============================================================
# ANA UÇUŞ DÖNGÜSÜ
# ============================================================

phase = "AUTO"
hold_start_time: Optional[float] = None
hold_alignment_since: Optional[float] = None
hold_heading_deg: Optional[float] = None
hold_forward_wp: Optional[tuple[float, float]] = None
hold_entry_thrust = GUIDED_HOLD_MIN_THRUST
hold_previous_pitch_command = 0.0
hold_previous_roll_command = 0.0
dive_start_time: Optional[float] = None
dive_heading_deg: Optional[float] = None
low_airspeed_mode = False
dive_entry_pitch_deg = 0.0
dive_entry_roll_deg = 0.0
dive_entry_thrust = DIVE_ENTRY_MIN_THRUST
dive_previous_pitch_command = 0.0
dive_previous_roll_command = 0.0
recovery_start_time: Optional[float] = None
recovery_start_pitch = DIVE_BASE_PITCH_DEG
recovery_previous_pitch_command = 0.0
recovery_previous_roll_command = 0.0
recovery_heading_deg: Optional[float] = None
straight_start_time: Optional[float] = None
straight_entry_pitch_deg = 0.0
straight_entry_thrust = 1.0
straight_previous_roll_command = 0.0
guided_confirmed_at: Optional[float] = None
excessive_roll_since: Optional[float] = None
guided_entry_window_seen = False
last_control_send = 0.0
last_print = 0.0
minimum_altitude_seen = float("inf")

# Dalış uçuş-yolu PID durumu
dive_path_integral = 0.0
dive_path_previous_error = 0.0
dive_path_previous_time: Optional[float] = None
dive_path_derivative_filtered = 0.0

# Toparlanma uçuş-yolu PID durumu
recovery_path_integral = 0.0
recovery_path_previous_error = 0.0
recovery_path_previous_time: Optional[float] = None
recovery_path_derivative_filtered = 0.0

try:
    while True:
        loop_now = time.monotonic()
        receive_available_messages()

        # Haberleşme/telemetri sağlığı
        if loop_now - float(state["last_heartbeat"]) > HEARTBEAT_STALE_TIMEOUT:
            safe_rtl("HEARTBEAT akışı kesildi")
            break

        heartbeat_age = loop_now - float(state["last_heartbeat"])
        position_age = loop_now - float(state["last_position"])
        attitude_age = loop_now - float(state["last_attitude"])
        vfr_hud_age = loop_now - float(state["last_vfr_hud"])

        if position_age > POSITION_STALE_TIMEOUT:
            emergency_rtl_without_confirmation("Konum/irtifa telemetrisi güncellenmiyor")
            break

        if attitude_age > ATTITUDE_STALE_TIMEOUT:
            emergency_rtl_without_confirmation(
                f"ATTITUDE telemetrisi bayat ({attitude_age:.1f} s)"
            )
            break

        if vfr_hud_age > VFR_HUD_STALE_TIMEOUT:
            emergency_rtl_without_confirmation(
                f"VFR_HUD telemetrisi bayat ({vfr_hud_age:.1f} s)"
            )
            break

        if not bool(state["armed"]):
            print("\nAraç DISARM oldu. Program sonlandırılıyor.")
            break

        current_mode = str(state["mode"])
        current_alt = float(state["rel_alt"])
        current_lat = float(state["lat"])
        current_lon = float(state["lon"])
        current_pitch = float(state["pitch"])
        current_yaw = float(state["yaw"])
        current_seq = state["mission_seq"]

        if phase in ("HOLD", "DIVE", "RECOVERY", "STRAIGHT"):
            minimum_altitude_seen = min(minimum_altitude_seen, current_alt)

        target_distance = haversine_m(
            current_lat,
            current_lon,
            DIVE_TARGET[0],
            DIVE_TARGET[1],
        )
        target_bearing = bearing_deg(
            current_lat,
            current_lon,
            DIVE_TARGET[0],
            DIVE_TARGET[1],
        )
        path_angle = flight_path_down_angle_deg()

        # ArduPilot kendi failsafe'iyle RTL'ye geçtiyse komutla geri çevrilmez.
        if current_mode in ("RTL", "QRTL"):
            print("\nArduPilot RTL modunda. Harici kontrol bırakılıyor.")
            break

        if bool(state["failsafe"]):
            safe_rtl(f"Failsafe algılandı: {state['failsafe_text']}")
            break

        # Son emniyet katmanı: yalnızca harici attitude komutlarının aktif
        # olduğu GUIDED fazlarında çalışır. 50 m'de başlayan normal RECOVERY
        # önceliklidir; uçak ataletiyle yine de 30 m'ye inerse bu döngüde yeni
        # attitude komutu gönderilmeden RTL üç kez istenir ve kontrol bırakılır.
        # AUTO/kalkış fazına uygulanmaz; böylece görev başlangıcı yanlışlıkla
        # irtifa tabanına takılmaz.
        if (
            phase in ("HOLD", "DIVE", "RECOVERY", "STRAIGHT")
            and current_alt <= EMERGENCY_RTL_ALT
        ):
            emergency_rtl_without_confirmation(
                f"{EMERGENCY_RTL_ALT:.0f} m acil irtifa tabanı görüldü "
                f"(faz={phase}, irtifa={current_alt:.1f} m)"
            )
            break

        # -------------------------
        # AUTO -> GUIDED HOLD geçişi
        # Uçak tüm waypointleri AUTO'da takip eder. Dalış WP aktifken,
        # dalış noktasına göre doğrultu ve roll uygun olduğu ilk anda GUIDED'e geçer.
        # -------------------------
        if phase == "AUTO":
            if current_mode != "AUTO":
                if pilot_took_control(current_mode, phase):
                    break
                safe_rtl(f"AUTO görev takibi sırasında beklenmeyen mod: {current_mode}")
                break

            altitude_error = abs(current_alt - DIVE_TRIGGER_ALT)
            live_course = ground_course_deg()
            heading_error = (
                None
                if live_course is None
                else wrap_180(target_bearing - live_course)
            )

            auto_aligned = (
                current_seq == DIVE_WP_SEQ
                and live_course is not None
                and GUIDED_ENTRY_DISTANCE_MIN_M <= target_distance <= GUIDED_ENTRY_DISTANCE_MAX_M
                and abs(float(heading_error)) <= AUTO_ALIGN_HEADING_TOLERANCE_DEG
                and abs(float(state["roll"])) <= AUTO_ALIGN_ROLL_TOLERANCE_DEG
                and altitude_error <= DIVE_TRIGGER_ALT_TOLERANCE
            )

            if (
                current_seq == DIVE_WP_SEQ
                and GUIDED_ENTRY_DISTANCE_MIN_M
                <= target_distance
                <= GUIDED_ENTRY_DISTANCE_MAX_M
            ):
                guided_entry_window_seen = True

            if (
                current_seq == DIVE_WP_SEQ
                and guided_entry_window_seen
                and target_distance < GUIDED_ENTRY_DISTANCE_MIN_M
                and not auto_aligned
            ):
                print(
                    "\nGUIDED giriş penceresi kaçırıldı; kod GUIDED'a geçmeyecek. "
                    "AUTO görev ve emniyet LOITER kontrolü Pixhawk'a bırakılıyor."
                )
                break

            if current_seq == DIVE_WP_SEQ and not auto_aligned and loop_now - last_print >= TELEMETRY_PRINT_PERIOD:
                heading_text = "---" if heading_error is None else f"{heading_error:+.1f}°"
                print(
                    f"Son yaklaşma kontrolü | Mesafe: {target_distance:.1f} m | "
                    f"İrtifa: {current_alt:.1f} m | HDG hata: {heading_text} | "
                    f"Roll: {float(state['roll']):+.1f}° | AUTO doğrultuya oturuyor"
                )
                last_print = loop_now

            if auto_aligned:
                print(
                    "\nAUTO dalış doğrultusuna oturdu | "
                    f"SEQ: {current_seq} | Mesafe: {target_distance:.1f} m | "
                    f"HDG hata: {float(heading_error):+.1f}° | Roll: {float(state['roll']):+.1f}°"
                )
                set_mode("GUIDED")

                current_mode = str(state["mode"])
                if current_mode != "GUIDED":
                    raise RuntimeError(
                        f"GUIDED doğrulandıktan sonra beklenmeyen mod: {current_mode}"
                    )

                phase = "HOLD"
                hold_start_time = time.monotonic()
                hold_alignment_since = None
                guided_confirmed_at = hold_start_time
                excessive_roll_since = None
                reset_guided_lateral_safety()

                # AUTO'nun oturduğu dalış doğrultusunu sabitle. Bundan sonra hedef
                # geçilse bile bearing yeniden hesaplanmaz; ters dönüş oluşmaz.
                hold_heading_deg = target_bearing
                dive_heading_deg = hold_heading_deg

                forward_distance = max(
                    GUIDED_FORWARD_WP_MIN_DISTANCE_M,
                    max(float(state["groundspeed"]), 1.0) * GUIDED_HOLD_DURATION_SEC + 30.0,
                )
                hold_forward_wp = destination_point(
                    current_lat, current_lon, hold_heading_deg, forward_distance
                )

                auto_thrust = clamp(float(state["throttle_pct"]) / 100.0, 0.0, 1.0)
                hold_entry_thrust = max(auto_thrust, GUIDED_HOLD_MIN_THRUST)
                hold_previous_pitch_command = current_pitch
                hold_previous_roll_command = float(state["roll"])
                reset_lateral_pid(hold_heading_deg)

                set_attitude_target(
                    roll_deg=hold_previous_roll_command,
                    pitch_deg=hold_previous_pitch_command,
                    thrust=hold_entry_thrust,
                )
                last_control_send = time.monotonic()
                print(
                    f"GUIDED HOLD başladı | Süre: {GUIDED_HOLD_DURATION_SEC:.0f} s | "
                    f"Kilitli doğrultu: {hold_heading_deg:.1f}° | "
                    f"İleri doğrultu referansı: {hold_forward_wp[0]:.8f}, {hold_forward_wp[1]:.8f}\n"
                    "Yaw kontrolü kapalı: quaternion yaw=0.0°, dümen komutu nötr."
                )
                continue

        # -------------------------
        # GUIDED hazırlık / 6 saniyelik ileri sanal WP takibi
        # -------------------------
        if phase == "HOLD" and loop_now - last_control_send >= CONTROL_PERIOD:
            live_mode = str(state["mode"])
            if live_mode != "GUIDED":
                if (
                    guided_confirmed_at is not None
                    and loop_now - guided_confirmed_at <= GUIDED_ENTRY_GRACE
                ):
                    time.sleep(0.01)
                    continue
                if pilot_took_control(live_mode, phase):
                    break
                safe_rtl(f"GUIDED HOLD sırasında mod kaybedildi: {live_mode}")
                break

            assert hold_start_time is not None
            assert hold_heading_deg is not None
            elapsed = loop_now - hold_start_time
            command_dt = clamp(loop_now - last_control_send, 0.01, 0.20)

            if not bool(guided_rail_state["active"]):
                rescue_reason = guided_lateral_rescue_reason(
                    "HOLD", hold_heading_deg, elapsed, loop_now
                )
                if rescue_reason is not None:
                    start_guided_rail_recovery(rescue_reason, "HOLD")

            # HOLD boyunca öncelik kanatları yatay tutmaktır. Heading düzeltmesi
            # küçük tutulur; böylece sağa/sola bank yaparak dalışa girilmez.
            heading_hold_command = lateral_pid_command(hold_heading_deg, "DIVE")
            level_hold_command = roll_level_command(
                kp=0.85,
                kd=0.24,
                max_command_deg=GUIDED_HOLD_ROLL_COMMAND_MAX_DEG,
            )
            raw_roll_command = clamp(
                0.35 * heading_hold_command + 0.65 * level_hold_command,
                -GUIDED_HOLD_ROLL_COMMAND_MAX_DEG,
                GUIDED_HOLD_ROLL_COMMAND_MAX_DEG,
            )
            rail_active_this_cycle = bool(guided_rail_state["active"])
            rail_completed = False
            if rail_active_this_cycle:
                raw_roll_command, rail_completed, _ = guided_rail_recovery_command(
                    hold_heading_deg, loop_now
                )
                if rail_completed:
                    # Kurtarma sonrası son 2 saniyelik doğrultu doğrulamasını yeniden yap.
                    hold_start_time = loop_now - (
                        GUIDED_HOLD_DURATION_SEC - GUIDED_ALIGN_CONFIRM_SEC
                    )
                    elapsed = loop_now - hold_start_time
                    hold_alignment_since = None
            roll_command = rate_limit(
                target=raw_roll_command,
                previous=hold_previous_roll_command,
                rate_per_second=(
                    RAIL_COMMAND_RATE_LIMIT_DEG_PER_SEC
                    if rail_active_this_cycle
                    else GUIDED_HOLD_ROLL_RATE_LIMIT_DEG_PER_SEC
                ),
                dt=command_dt,
            )

            altitude_error_hold = DIVE_TRIGGER_ALT - current_alt
            vertical_speed_down = float(state["vz"])
            pitch_target = clamp(
                STRAIGHT_BASE_PITCH_DEG
                + altitude_error_hold * GUIDED_HOLD_ALT_KP
                + vertical_speed_down * GUIDED_HOLD_VZ_KD,
                GUIDED_HOLD_MIN_PITCH_DEG,
                GUIDED_HOLD_MAX_PITCH_DEG,
            )
            # Son 2 saniyede burun yaklaşık yataya getirilir. Dalış henüz başlamaz;
            # yalnızca pitch, roll ve gaz geçişe hazırlanır.
            final_prepare_ratio = smoothstep01(
                (elapsed - (GUIDED_HOLD_DURATION_SEC - GUIDED_HOLD_FINAL_LEVEL_SEC))
                / GUIDED_HOLD_FINAL_LEVEL_SEC
            )
            pitch_target = pitch_target * (1.0 - final_prepare_ratio)
            pitch_command = asymmetric_rate_limit(
                pitch_target,
                hold_previous_pitch_command,
                18.0,
                25.0,
                command_dt,
            )

            thrust_command = clamp(
                hold_entry_thrust
                + max(altitude_error_hold, 0.0) * 0.012
                + max(vertical_speed_down, 0.0) * 0.025,
                GUIDED_HOLD_MIN_THRUST,
                0.90,
            )
            thrust_command = (
                thrust_command * (1.0 - final_prepare_ratio)
                + max(hold_entry_thrust, GUIDED_HOLD_MIN_THRUST) * final_prepare_ratio
            )
            if 0.1 < float(state["airspeed"]) < DIVE_MIN_SAFE_AIRSPEED:
                thrust_command = max(thrust_command, DIVE_LOW_AIRSPEED_THRUST)
                pitch_command = min(pitch_command, 7.0)
            if rail_active_this_cycle:
                thrust_command = max(thrust_command, RAIL_MIN_THRUST)

            set_attitude_target(
                roll_deg=roll_command,
                pitch_deg=pitch_command,
                thrust=thrust_command,
            )
            hold_previous_pitch_command = pitch_command
            hold_previous_roll_command = roll_command
            last_control_send = loop_now

            # Sabit süre dolduğu için körlemesine dalışa geçme. Son iki saniye
            # boyunca GPS yer izi ve gerçek roll kesintisiz uygun kalmalıdır.
            live_hold_course = ground_course_deg()
            hold_heading_error = (
                None
                if live_hold_course is None
                else wrap_180(hold_heading_deg - live_hold_course)
            )
            hold_alignment_ok = (
                elapsed >= GUIDED_HOLD_DURATION_SEC - GUIDED_ALIGN_CONFIRM_SEC
                and hold_heading_error is not None
                and abs(hold_heading_error) <= GUIDED_ALIGN_HEADING_TOLERANCE_DEG
                and abs(float(state["roll"])) <= GUIDED_ALIGN_ROLL_TOLERANCE_DEG
            )

            if hold_alignment_ok:
                if hold_alignment_since is None:
                    hold_alignment_since = loop_now
            else:
                hold_alignment_since = None

            alignment_confirmed = (
                hold_alignment_since is not None
                and loop_now - hold_alignment_since >= GUIDED_ALIGN_CONFIRM_SEC
            )

            if elapsed >= GUIDED_HOLD_DURATION_SEC and alignment_confirmed:
                print(
                    f"\nGUIDED doğrulandı | Süre: {elapsed:.1f} s | "
                    f"Son {GUIDED_ALIGN_CONFIRM_SEC:.1f} s kesintisiz uygun | "
                    f"HDG hata: {float(hold_heading_error):+.1f}° | "
                    f"Roll: {float(state['roll']):+.1f}° | Dümen komut: 0.0° NÖTR\n"
                    "Aynı kilitli doğrultuda düz dalış başlıyor."
                )
                phase = "DIVE"
                dive_start_time = time.monotonic()
                excessive_roll_since = None
                reset_guided_lateral_safety()

                # HOLD sonunda gerçek pitch pozitif olsa bile yeni bir tırmanış
                # komutu taşınmaz; dalış rampası en fazla 0°'den başlar.
                dive_entry_pitch_deg = min(current_pitch, 0.0)
                dive_entry_roll_deg = float(state["roll"])
                dive_entry_thrust = clamp(command_state["thrust"], DIVE_ENTRY_MIN_THRUST, 1.0)
                dive_previous_pitch_command = dive_entry_pitch_deg
                # Dalışa bank komutuyla değil, yatay kanat hedefiyle gir.
                dive_previous_roll_command = clamp(
                    float(state["roll"]),
                    -GUIDED_HOLD_ROLL_COMMAND_MAX_DEG,
                    GUIDED_HOLD_ROLL_COMMAND_MAX_DEG,
                )
                reset_lateral_pid(dive_heading_deg)
                low_airspeed_mode = False

                initial_path_error = (
                    0.0
                    if path_angle is None
                    else TARGET_FLIGHT_PATH_DOWN_DEG - path_angle
                )
                dive_path_integral = 0.0
                dive_path_previous_error = initial_path_error
                dive_path_previous_time = dive_start_time
                dive_path_derivative_filtered = 0.0
                reset_lateral_pid(dive_heading_deg)
                continue

            if (
                elapsed >= GUIDED_HOLD_MAX_DURATION_SEC
                and not bool(guided_rail_state["active"])
            ):
                heading_text = (
                    "---" if hold_heading_error is None else f"{hold_heading_error:+.1f}°"
                )
                safe_rtl(
                    "GUIDED doğrultu doğrulanamadı: "
                    f"HDG hata {heading_text}, roll {float(state['roll']):+.1f}°"
                )
                break

        # -------------------------
        # Dalış kontrolü
        # -------------------------
        if phase == "DIVE" and loop_now - last_control_send >= CONTROL_PERIOD:
            live_mode = str(state["mode"])
            if live_mode != "GUIDED":
                # GUIDED'e yeni girildiyse tek bir heartbeat gecikmesini mod kaybı
                # olarak değerlendirme. Tolerans sonrasında gerçek mod kaybında RTL.
                if (
                    guided_confirmed_at is not None
                    and loop_now - guided_confirmed_at <= GUIDED_ENTRY_GRACE
                ):
                    time.sleep(0.01)
                    continue

                if pilot_took_control(live_mode, phase):
                    break
                safe_rtl(f"Dalış sırasında GUIDED modu kaybedildi: {live_mode}")
                break

            assert dive_start_time is not None
            dive_safety_elapsed = loop_now - dive_start_time
            if not bool(guided_rail_state["active"]):
                rescue_reason = guided_lateral_rescue_reason(
                    "DIVE", dive_heading_deg, dive_safety_elapsed, loop_now
                )
                if rescue_reason is not None:
                    start_guided_rail_recovery(rescue_reason, "DIVE")

            # 0.02 kaydındaki dal-kalk davranışının ana kaynağı uçuş-yolu PID'sinin
            # -44/-22/-28 derece arasında komut değiştirmesiydi. Dalışta geri
            # beslemeli pitch düzeltmesi yapılmaz: giriş rampasından sonra hedef
            # tek ve değişmezdir. Uçuş-yolu açısı yalnızca telemetride izlenir.
            raw_pitch_command = DIVE_BASE_PITCH_DEG
            dive_path_integral = 0.0
            dive_path_previous_time = loop_now
            if path_angle is not None:
                dive_path_previous_error = TARGET_FLIGHT_PATH_DOWN_DEG - path_angle

            assert dive_start_time is not None
            command_dt = clamp(loop_now - last_control_send, 0.01, 0.20)
            entry_elapsed = loop_now - dive_start_time
            # İlk 1.5 saniye yalnızca kanatları yatayla. Dalış pitch rampası,
            # ani sola/sağa yatışı önlemek için bu süreden sonra başlar.
            pitch_entry_elapsed = max(0.0, entry_elapsed - DIVE_ENTRY_LEVEL_SEC)
            entry_ratio = smoothstep01(
                pitch_entry_elapsed / DIVE_ENTRY_TRANSITION_SEC
            )

            # Mevcut pitch'ten dalış PID hedefine yumuşak geç.
            ramped_pitch_target = (
                dive_entry_pitch_deg * (1.0 - entry_ratio)
                + raw_pitch_command * entry_ratio
            )

            airspeed = float(state["airspeed"])

            pitch_command = asymmetric_rate_limit(
                ramped_pitch_target,
                dive_previous_pitch_command,
                DIVE_PITCH_NOSE_DOWN_RATE_DEG_PER_SEC,
                DIVE_PITCH_NOSE_UP_RATE_DEG_PER_SEC,
                command_dt,
            )

            # Dalış doğrultusu AUTO sonunda kilitlenmiştir. Dalış boyunca yeniden
            # hedef bearing'ine çevrilmez; hedef geçilse bile ters dönüş oluşmaz.

            level_roll = roll_level_command(
                kp=0.90,
                kd=0.25,
                max_command_deg=DIVE_ROLL_COMMAND_MAX_DEG,
            )
            heading_roll = lateral_pid_command(dive_heading_deg, "DIVE")
            if entry_elapsed < DIVE_ENTRY_LEVEL_SEC or abs(float(state["roll"])) > DIVE_ENTRY_LEVEL_ROLL_MAX_DEG:
                raw_roll_command = level_roll
            else:
                # Dalış boyunca kanat yataylama ana önceliktir; heading düzeltmesi
                # yalnızca küçük bir payla eklenir.
                raw_roll_command = clamp(
                    0.75 * level_roll + 0.25 * heading_roll,
                    -DIVE_ROLL_COMMAND_MAX_DEG,
                    DIVE_ROLL_COMMAND_MAX_DEG,
                )

            rail_active_this_cycle = bool(guided_rail_state["active"])
            if rail_active_this_cycle:
                raw_roll_command, _, _ = guided_rail_recovery_command(
                    dive_heading_deg, loop_now
                )

            roll_command = rate_limit(
                target=raw_roll_command,
                previous=dive_previous_roll_command,
                rate_per_second=(
                    RAIL_COMMAND_RATE_LIMIT_DEG_PER_SEC
                    if rail_active_this_cycle
                    else DIVE_ROLL_RATE_LIMIT_DEG_PER_SEC
                ),
                dt=command_dt,
            )

            # Roll düzeltmeleri pitch hedefini değiştirmez. Böylece yanal bir
            # düzeltme dalış ekseninde burun kaldırma/daldırma darbesi üretmez.
            roll_abs = abs(float(state["roll"]))

            # Gazı AUTO değerinden dalış gazına kademeli indir.
            # Düşük hız koruması yalnızca gazı etkiler; pitch hedefini değiştirmez.
            if low_airspeed_mode:
                if airspeed >= DIVE_LOW_AIRSPEED_OFF:
                    low_airspeed_mode = False
            elif airspeed <= DIVE_LOW_AIRSPEED_ON:
                low_airspeed_mode = True

            thrust_command = (
                dive_entry_thrust * (1.0 - entry_ratio)
                + DIVE_THRUST * entry_ratio
            )
            # Kanatlar yataylanmadan gaz düşürülmez.
            if entry_elapsed < DIVE_ENTRY_LEVEL_SEC:
                thrust_command = max(thrust_command, DIVE_ENTRY_LEVEL_THRUST)
            if low_airspeed_mode:
                thrust_command = max(thrust_command, DIVE_LOW_AIRSPEED_THRUST)
            if rail_active_this_cycle:
                thrust_command = max(thrust_command, RAIL_MIN_THRUST)

            set_attitude_target(
                roll_deg=roll_command,
                pitch_deg=pitch_command,
                thrust=thrust_command,
            )
            dive_previous_pitch_command = pitch_command
            dive_previous_roll_command = roll_command
            last_control_send = loop_now

            # Kullanıcı isteği: dalış 50.0 m bağıl irtifaya kadar sürer.
            # Buradaki 50 m, RECOVERY komutunun başlangıç irtifasıdır; uçağın
            # ataleti nedeniyle gerçek minimum irtifa 50 m'nin altında olabilir.
            if current_alt <= DIVE_END_ALT:
                print(
                    f"\n50.0 m görüldü; doğrudan pull-out başlıyor | "
                    f"İrtifa: {current_alt:.1f} m | "
                    f"Aşağı hız: {max(float(state['vz']), 0.0):.1f} m/s"
                )
                phase = "RECOVERY"
                if bool(guided_rail_state["active"]):
                    print(
                        "[RAY KURTARMA] 50 m toparlanma sınırı öncelikli; "
                        "ray yakalama sonlandırılıp pull-out başlatılıyor."
                    )
                reset_guided_rail_recovery()
                recovery_start_time = time.monotonic()
                excessive_roll_since = None
                recovery_start_pitch = current_pitch
                recovery_previous_pitch_command = dive_previous_pitch_command
                recovery_previous_roll_command = dive_previous_roll_command

                # Dalış başlangıcında dalış noktasına göre hesaplanan hedef doğrultu
                # toparlanmada da korunur. Pull-out anındaki sapmış GPS yönü hedef yapılmaz.
                recovery_heading_deg = dive_heading_deg
                if recovery_heading_deg is None:
                    recovery_heading_deg = ground_course_deg()
                if recovery_heading_deg is None:
                    recovery_heading_deg = current_yaw
                reset_lateral_pid(recovery_heading_deg)

                recovery_path_integral = 0.0
                recovery_path_previous_error = (
                    0.0
                    if path_angle is None
                    else RECOVERY_TARGET_PATH_DOWN_DEG - path_angle
                )
                recovery_path_previous_time = recovery_start_time
                recovery_path_derivative_filtered = 0.0
                print(
                    f"Toparlanma doğrultusu sabitlendi: "
                    f"{recovery_heading_deg:.1f}°"
                )

        # -------------------------
        # Toparlanma kontrolü
        # Dalış noktasına göre kilitlenen hedef doğrultu sabit tutulur.
        # -------------------------
        if phase == "RECOVERY" and loop_now - last_control_send >= CONTROL_PERIOD:
            live_mode = str(state["mode"])
            if live_mode != "GUIDED":
                if pilot_took_control(live_mode, phase):
                    break
                safe_rtl(f"Toparlanma sırasında GUIDED modu kaybedildi: {live_mode}")
                break

            assert recovery_start_time is not None
            assert recovery_heading_deg is not None
            elapsed = loop_now - recovery_start_time
            command_dt = clamp(loop_now - last_control_send, 0.01, 0.20)

            # İki aşamalı ve sınırlı pull-out. 90-100 m arasında tırmanış hedefi
            # -12°'den 0°'ye kademeli yaklaşır; düz uçuşa geçişte irtifa taşması azalır.
            level_blend = clamp((current_alt - 90.0) / 10.0, 0.0, 1.0)
            recovery_target_path = RECOVERY_TARGET_PATH_DOWN_DEG * (1.0 - level_blend)
            recovery_pitch_target = recovery_pitch_schedule(path_angle)
            if path_angle is not None:
                # İşaret önemlidir:
                # path_angle hedefin üzerindeyse uçak hâlâ aşağı gidiyordur ve
                # pitch artırılmalıdır. Hedefin altındaysa uçak fazla tırmanıyordur
                # ve pitch azaltılmalıdır.
                path_correction = 0.60 * (path_angle - recovery_target_path)

                # 90-100 m arasında düz uçuşa yaklaşırken negatif/küçük pitch
                # komutuna izin ver. Alt irtifalarda güçlü pull-out için mevcut
                # minimum pitch sınırı korunur.
                recovery_min_pitch = (
                    RECOVERY_MIN_PITCH_DEG * (1.0 - level_blend)
                    + STRAIGHT_MIN_PITCH_DEG * level_blend
                )

                recovery_pitch_target = clamp(
                    recovery_pitch_target + path_correction,
                    recovery_min_pitch,
                    RECOVERY_TARGET_PITCH_DEG,
                )

            # Uçuş-yolu zaten tırmanışa geçtiğinde 14°'yi aşma; böylece pitch overshoot azalır.
            if path_angle is not None and path_angle <= RECOVERY_CLIMB_PATH_TARGET_DEG:
                recovery_pitch_target = min(recovery_pitch_target, 12.0)

            # Yatış varsa önce kanatları yatayla; pitch-up komutunu geçici olarak azalt.
            roll_abs = abs(float(state["roll"]))
            if roll_abs >= BANK_PROTECTION_CRITICAL_DEG:
                recovery_pitch_target = min(recovery_pitch_target, 5.0)
            elif roll_abs >= BANK_PROTECTION_START_DEG:
                recovery_pitch_target = min(recovery_pitch_target, 8.0)

            pitch_command = asymmetric_rate_limit(
                recovery_pitch_target,
                recovery_previous_pitch_command,
                RECOVERY_PITCH_NOSE_DOWN_RATE_DEG_PER_SEC,
                RECOVERY_PITCH_NOSE_UP_RATE_DEG_PER_SEC,
                command_dt,
            )

            # Gaz %20 seviyesinden %100'e bir anda sıçramaz; 1 saniyelik
            # yumuşak geçişle tam gaza çıkar. Böylece motor torku darbesi azaltılır.
            descent_rate = max(float(state["vz"]), 0.0)
            airspeed = float(state["airspeed"])
            thrust_ratio = smoothstep01(elapsed / RECOVERY_THRUST_RAMP_SEC)
            thrust_command = (
                RECOVERY_MIN_THRUST * (1.0 - thrust_ratio)
                + RECOVERY_NOMINAL_THRUST * thrust_ratio
            )
            if 0.1 < airspeed < DIVE_MIN_SAFE_AIRSPEED:
                thrust_command = max(thrust_command, RECOVERY_LOW_SPEED_THRUST)
            if current_alt <= DIVE_END_ALT + 2.0 and descent_rate > 1.0:
                thrust_command = max(thrust_command, 0.95)

            # Dalış başlangıcında kilitlenen doğrultuyu RECOVERY boyunca koru.
            # Heading hatası küçük roll/bank komutuna çevrilir; yaw zorlanmaz.
            heading_recovery_roll = lateral_pid_command(
                recovery_heading_deg,
                "RECOVERY",
            )
            level_recovery_roll = roll_level_command(
                kp=1.00,
                kd=0.28,
                max_command_deg=RECOVERY_ROLL_COMMAND_MAX_DEG,
            )
            # İlk saniyelerde tamamen kanat yataylama; daha sonra heading düzeltmesi
            # en fazla küçük bir payla devreye girer.
            recovery_heading_authority = clamp(
                elapsed / RECOVERY_HEADING_RAMP_SEC,
                0.0,
                1.0,
            )
            raw_roll_command = clamp(
                level_recovery_roll
                + 0.20 * recovery_heading_authority * heading_recovery_roll,
                -RECOVERY_ROLL_COMMAND_MAX_DEG,
                RECOVERY_ROLL_COMMAND_MAX_DEG,
            )
            roll_command = rate_limit(
                target=raw_roll_command,
                previous=recovery_previous_roll_command,
                rate_per_second=DIVE_ROLL_RATE_LIMIT_DEG_PER_SEC,
                dt=command_dt,
            )

            # Yatış büyürse motor torkunu azalt; düşük hız güvenliği bunu geçersiz kılabilir.
            if roll_abs >= BANK_PROTECTION_CRITICAL_DEG and airspeed >= DIVE_MIN_SAFE_AIRSPEED:
                thrust_command = min(thrust_command, 0.78)
            elif roll_abs >= BANK_PROTECTION_START_DEG and airspeed >= DIVE_MIN_SAFE_AIRSPEED:
                thrust_command = min(thrust_command, 0.82)

            set_attitude_target(
                roll_deg=roll_command,
                pitch_deg=pitch_command,
                thrust=thrust_command,
            )
            recovery_previous_pitch_command = pitch_command
            recovery_previous_roll_command = roll_command
            last_control_send = loop_now

            recovery_path_level = (
                path_angle is not None
                and abs(path_angle) <= RECOVERY_LEVEL_PATH_TOLERANCE_DEG
            )
            if (
                current_alt >= RECOVERY_COMPLETE_ALT
                and current_pitch >= -3.0
                and recovery_path_level
            ):
                print(
                    f"\nToparlanma tamamlandı: {current_alt:.1f} m, "
                    f"pitch {current_pitch:.1f}°. "
                    "Aynı doğrultuda düz uçuşa geçiliyor."
                )
                phase = "STRAIGHT"
                straight_start_time = time.monotonic()
                straight_entry_pitch_deg = current_pitch
                straight_entry_thrust = clamp(command_state["thrust"], 0.72, 1.00)
                straight_previous_roll_command = recovery_previous_roll_command
                reset_lateral_pid(recovery_heading_deg)
                continue

        # -------------------------
        # Düz devam kontrolü
        # Araç GUIDED'da kalır, sonraki mission noktasına veya HOME'a dönmez.
        # Ctrl+C verilirse mevcut güvenlik davranışı gereği RTL istenir.
        # -------------------------
        if phase == "STRAIGHT" and loop_now - last_control_send >= CONTROL_PERIOD:
            live_mode = str(state["mode"])
            if live_mode != "GUIDED":
                if pilot_took_control(live_mode, phase):
                    break
                safe_rtl(f"Düz uçuş sırasında GUIDED modu kaybedildi: {live_mode}")
                break

            assert recovery_heading_deg is not None
            assert straight_start_time is not None

            live_course = ground_course_deg()
            command_dt = clamp(loop_now - last_control_send, 0.01, 0.20)

            # Düz uçuşta da aynı kilitli GPS yer izini koru.
            raw_roll_command = lateral_pid_command(
                recovery_heading_deg,
                "STRAIGHT",
            )
            roll_command = rate_limit(
                target=raw_roll_command,
                previous=straight_previous_roll_command,
                rate_per_second=DIVE_ROLL_RATE_LIMIT_DEG_PER_SEC,
                dt=command_dt,
            )

            altitude_error = STRAIGHT_HOLD_ALT - current_alt
            vertical_speed_down = float(state["vz"])

            # İrtifa hatası + aşağı yönlü hız geri beslemesi. Araç alçalıyorsa
            # komut otomatik olarak daha pozitif pitch'e gider.
            hold_pitch = (
                STRAIGHT_BASE_PITCH_DEG
                + altitude_error * STRAIGHT_ALT_KP
                + vertical_speed_down * STRAIGHT_VZ_KD
            )
            hold_pitch = clamp(
                hold_pitch,
                STRAIGHT_MIN_PITCH_DEG,
                STRAIGHT_MAX_PITCH_DEG,
            )

            # +22° toparlanma pitch'inden düz uçuş pitch'ine ani geçiş yapılmaz.
            transition_ratio = clamp(
                (loop_now - straight_start_time) / STRAIGHT_TRANSITION_SEC,
                0.0,
                1.0,
            )
            pitch_command = (
                straight_entry_pitch_deg * (1.0 - transition_ratio)
                + hold_pitch * transition_ratio
            )

            hold_thrust = clamp(
                STRAIGHT_BASE_THRUST
                + max(altitude_error, 0.0) * 0.015
                + max(vertical_speed_down, 0.0) * 0.035,
                0.68,
                1.0,
            )
            thrust_command = (
                straight_entry_thrust * (1.0 - transition_ratio)
                + hold_thrust * transition_ratio
            )

            airspeed = float(state["airspeed"])
            if airspeed > 0.1 and airspeed < 15.0:
                thrust_command = max(thrust_command, 0.88)
                pitch_command = min(pitch_command, 8.0)

            # İrtifa 90 m altına sarkarsa kontrollü tırmanış desteği verilir.
            if current_alt < STRAIGHT_HOLD_ALT - 10.0:
                pitch_command = max(pitch_command, 8.0)
                thrust_command = max(thrust_command, 0.90)

            set_attitude_target(
                roll_deg=roll_command,
                pitch_deg=pitch_command,
                thrust=thrust_command,
            )
            straight_previous_roll_command = roll_command
            last_control_send = loop_now

        # -------------------------
        # Terminal telemetrisi
        # -------------------------
        if loop_now - last_print >= TELEMETRY_PRINT_PERIOD:
            live_course = ground_course_deg()
            target_course = (
                recovery_heading_deg
                if phase in ("RECOVERY", "STRAIGHT")
                else (hold_heading_deg if phase == "HOLD" else (dive_heading_deg if phase == "DIVE" else None))
            )
            print(
                compact_status(
                    phase=phase,
                    mode=current_mode,
                    seq=current_seq,
                    distance_m=target_distance,
                    altitude_m=current_alt,
                    actual_pitch_deg=current_pitch,
                    path_angle_deg=path_angle,
                    course_deg=live_course,
                    target_course_deg=target_course,
                )
            )
            last_print = loop_now

        time.sleep(0.01)

except KeyboardInterrupt:
    safe_rtl("Program kullanıcı tarafından durduruldu")

except Exception as error:
    print(f"\nProgram hatası: {error}")
    safe_rtl("Program istisnası")
    raise

finally:
    if minimum_altitude_seen < float("inf"):
        print(paint(f"Minimum görülen irtifa: {minimum_altitude_seen:.1f} m", ANSI_MAGENTA, bold=True))
    print("Program sonlandırıldı.")
