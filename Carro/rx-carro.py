# =================================================================
# WILLIAM LEON - INGENIERIA EN TELECOMUNICACIONES
# CARRO RX (RP2040 / Pico W / Pico 2 W)
#   NRF24L01 : SPI0 SCK GP2, MOSI GP3, MISO GP4, CSN GP5, CE GP6
#   OLED     : I2C1 SCL GP15, SDA GP14
#   GPS      : UART1 TX GP8, RX GP9
#   SERVO    : GP16
#   ESC      : GP17
#   TCRT     : GP26 (ADC0)  --- Laps por N-B-N-B pegado (Detector v2)
# =================================================================
from machine import Pin, SPI, I2C, PWM, UART, ADC
import utime
from ssd1306 import SSD1306_I2C
from micropyGPS import MicropyGPS

# ==================== OLED (I2C rápido con fallback) ====================
try:
    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=1_000_000)  # 1 MHz
except:
    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400_000)    # fallback
oled = SSD1306_I2C(128, 32, i2c)

def draw_main(hora="--:--:--", laps=0, vel_txt="--", sats=0, tcrt_txt="--", rf_ok=True):
    oled.fill(0)
    oled.text(f"Hora: {hora}",                 0, 0)
    oled.text(f"Laps:{laps} Vel:{vel_txt}",    0, 8)
    oled.text(f"Sats:{sats}  TCRT:{tcrt_txt}", 0, 16)
    oled.text("RF:OK" if rf_ok else "RF:--",   0, 24)
    oled.show()

def draw_lap_flash(lap_num, hora, lat, lon):
    oled.fill(0)
    oled.text(f"LAP {lap_num}", 32, 0)
    oled.text(f"{hora}",        18, 8)
    lat_s = "{:.6f}".format(lat) if lat is not None else "--"
    lon_s = "{:.6f}".format(lon) if lon is not None else "--"
    oled.text(f"Lat:{lat_s[:12]}", 0, 16)
    oled.text(f"Lon:{lon_s[:12]}", 0, 24)
    oled.show()

def oled_ganaste(prom_kmh=None):
    oled.fill(0)
    oled.text("!GANASTE!", 20, 6)
    if prom_kmh is not None:
        oled.text("Prom:{:.1f}km/h".format(prom_kmh), 0, 20)
    else:
        oled.text("Prom: -- km/h", 0, 20)
    oled.show()

# ==================== NRF24L01 (SPI rápido) ====================
try:
    spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=8_000_000)  # 8 MHz
except:
    spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=1_000_000)  # fallback
csn = Pin(5, Pin.OUT, value=1)
ce  = Pin(6, Pin.OUT, value=0)

# LED interno: Pico W / Pico 2 W -> "LED"; Pico clásico -> GP25
try:
    led = Pin("LED", Pin.OUT)
    HAVE_LED = True
except:
    try:
        led = Pin(25, Pin.OUT)
        HAVE_LED = True
    except:
        HAVE_LED = False

CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR = 0x00,0x01,0x02,0x03,0x04
RF_CH, RF_SETUP, STATUS                        = 0x05,0x06,0x07
RX_ADDR_P0, TX_ADDR                            = 0x0A,0x10
RX_PW_P0, FIFO_STATUS                          = 0x11,0x17
RX_DR, TX_DS, MAX_RT                           = 0x40,0x20,0x10
W_REGISTER, R_REGISTER, REGISTER_MASK          = 0x20,0x00,0x1F
FLUSH_TX, FLUSH_RX, R_RX_PAYLOAD               = 0xE1,0xE2,0x61

ADDR    = b'\xe7\xe7\xe7\xe7\xe7'
CHANNEL = 40
PAYLOAD = 32

def reg_write(reg, val):
    csn(0); spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK), val])); csn(1)

def reg_read(reg):
    csn(0); spi.write(bytearray([R_REGISTER | (reg & REGISTER_MASK)])); b = spi.read(1); csn(1); return b[0]

def reg_write_bytes(reg, data):
    csn(0); spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK)]) + data); csn(1)

def flush():
    csn(0); spi.write(bytearray([FLUSH_TX])); csn(1)
    csn(0); spi.write(bytearray([FLUSH_RX])); csn(1)
    reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

# ==================== SERVO / ESC ====================
SERVO_PIN = 16
servo = PWM(Pin(SERVO_PIN)); servo.freq(50)
SERVO_MIN_US = 500; SERVO_MAX_US = 2400; PERIOD_US = 20000

def servo_write_deg(angle):
    angle = 0 if angle < 0 else (180 if angle > 180 else angle)
    us = int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0))
    duty = int(us * 65535 // PERIOD_US); servo.duty_u16(duty)

ESC_PIN = 17
esc = PWM(Pin(ESC_PIN)); esc.freq(50)
ESC_MIN_US = 1000; ESC_MID_US = 1500; ESC_MAX_US = 2000

def esc_write_us(us):
    us = ESC_MIN_US if us < ESC_MIN_US else (ESC_MAX_US if us > ESC_MAX_US else us)
    duty = int(us * 65535 // PERIOD_US); esc.duty_u16(duty)

def speed_to_us(percent):
    percent = 100 if percent > 100 else (-100 if percent < -100 else percent)
    return int(ESC_MID_US + (ESC_MAX_US - ESC_MID_US) * (percent / 100.0))

# Traducción de órdenes de radio (texto)
def dir_to_angle_jst1(txt):
    t = (txt or "").strip().lower()
    if "izquierda" in t: return 30
    if "derecha"   in t: return 150
    return 90

def dir_to_speed_jst2(txt):
    t = (txt or "").strip().lower()
    if "adelante" in t: return 100
    if "reversa"  in t: return -100
    return 0

# ==================== GPS (light) ====================
uart_gps = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9), timeout=1000)
gps = MicropyGPS(-5)  # Zona horaria Bogotá

def fmt_hora(ts):
    try:
        h, m, s = ts; return "{:02d}:{:02d}:{:02d}".format(int(h), int(m), int(s))
    except: return "--:--:--"

def conv_grados(secciones):
    try:
        g, m, hemi = secciones[0], secciones[1], secciones[2]
    except: return None
    if g == 0 and m == 0: return None
    dec = g + (m/60.0)
    if hemi in ('S','W'): dec = -dec
    return dec

def vel_kmh(v):
    if v is None: return None
    try:
        if isinstance(v,(list,tuple)):
            if len(v)>=3: return float(v[2])      # km/h (algunas versiones)
            if len(v)>=1: return float(v[0])*1.852  # nudos -> km/h
        return float(v)
    except: return None

# ==================== TCRT (Detector v2: adaptativo por umbral simétrico) ====================
adc_tcrt = ADC(26)  # GP26 / ADC0

# Filtros / adaptación
ALPHA_FILTER      = 0.6    # EMA rápido (seguir transiciones)  [↑=más rápido]
ALPHA_BASE        = 0.005  # EMA lento (luz ambiente)
ALPHA_PK          = 0.05   # EMA de pico (para TH adaptativo)

# Ventanas / tiempos
VENTANA_PATRON_MS = 900    # todo N-B-N-B (3 edges) debe caber aquí
GAP_MAX_MS        = 250    # máximo entre edges contiguos
DEBOUNCE_MS       = 8      # antirrebote mínimo por edge

# Umbrales
TH_MIN            = 700    # mínimo absoluto de umbral (ADC u16)
TH_SCALE          = 0.6    # TH = max(TH_MIN, TH_SCALE * pk_ema)

# Polaridad (si “negro” baja ADC, invierte)
INVERT_SIGNAL     = False  # pon True si ves que “negro” = negativo en v_hp

# Estado adaptativo
filt_v = None     # EMA rápido del ADC
base_v = None     # baseline lento
pk_ema = 0        # estimación de amplitud típica (|v_hp|)

def _filtro_y_umbral(v_raw):
    """Actualiza filtro rápido, baseline y umbral adaptativo."""
    global filt_v, base_v, pk_ema
    if filt_v is None:
        filt_v = v_raw
        base_v = v_raw
        pk_ema = 2000   # arranque razonable
    else:
        filt_v = int(filt_v + ALPHA_FILTER * (v_raw - filt_v))
        base_v = int(base_v + ALPHA_BASE   * (v_raw - base_v))
    v_hp = filt_v - base_v
    if INVERT_SIGNAL:
        v_hp = -v_hp
    # actualiza pico EMA (módulo)
    av = abs(v_hp)
    pk_ema = int(pk_ema + ALPHA_PK * (av - pk_ema))
    th = TH_MIN if (TH_MIN > int(TH_SCALE * pk_ema)) else int(TH_SCALE * pk_ema)
    return v_hp, th

# FSM por edges con Schmitt simétrico
state_high = None   # None/False/True -> bajo/alto
last_edge_ms = 0
edge_count   = 0
win_start_ms = 0

# ==================== Parseo payload RF ====================
def parse_dirs(s):
    try:
        parts = s.split(';')
        d1 = d2 = None
        for p in parts:
            kv = p.split(':', 1)
            if len(kv) != 2: continue
            k = kv[0].strip().upper(); v = kv[1].strip()
            if k == "JST1": d1 = v
            if k == "JST2": d2 = v
        return d1, d2
    except:
        return None, None

# ==================== Inicialización ====================
utime.sleep_ms(300)
# NRF
def _nrf_init():
    reg_write(CONFIG, 0x0C)
    reg_write(SETUP_AW, 0x03)
    reg_write(EN_AA, 0x01)
    reg_write(EN_RXADDR, 0x01)
    reg_write(SETUP_RETR, 0x3F)
    reg_write(RF_CH, CHANNEL)
    reg_write(RX_PW_P0, PAYLOAD)
    reg_write(RF_SETUP, 0x20 | (3<<1))   # 250 kbps, 0 dBm
    reg_write_bytes(RX_ADDR_P0, ADDR)
    reg_write_bytes(TX_ADDR,   ADDR)
    reg_write(CONFIG, 0x0B)              # PRIM_RX=1, PWR_UP=1
    utime.sleep_ms(2)
    flush()
    ce(1)
_nrf_init()

# Posiciones seguras iniciales
servo_write_deg(90)
esc_write_us(ESC_MID_US)

# Pantalla inicial
draw_main("INICIAL", 0, "--", 0, "--", rf_ok=False)
utime.sleep_ms(500)

# Seguridad de enlace RF
last_rx_ms = utime.ticks_ms()
KEEPALIVE_TIMEOUT_MS = 1200

# Laps/estadísticas
VUELTAS_META = 6
vueltas = 0
meta = False
velocidades_vuelta = []
last_lap_hora = "--:--:--"
last_lap_lat  = None
last_lap_lon  = None
lap_flash_until = 0

# Ritmos
OLED_PERIOD_MS = 100  # 10 Hz
next_oled_ms   = utime.ticks_ms()
LOG_PERIOD_MS  = 2000
next_log_ms    = utime.ticks_ms()

# ==================== Bucle principal ====================
while True:
    now = utime.ticks_ms()

    # ----- RF: recibe órdenes y gobierna SERVO/ESC (FAST PATH) -----
    st = reg_read(STATUS)
    rf_event = False
    if st & RX_DR:
        latest = None
        while True:
            csn(0); spi.write(bytearray([R_RX_PAYLOAD])); data = spi.read(PAYLOAD); csn(1)
            latest = data
            if reg_read(FIFO_STATUS) & 0x01:  # RX_EMPTY
                break
        reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        msg = latest.rstrip(b"\x00")
        try: s = msg.decode("utf-8")
        except: s = str(msg)

        j1, j2 = parse_dirs(s)
        # === APLICAR CONTROL INMEDIATO ===
        servo_write_deg(dir_to_angle_jst1(j1))
        esc_write_us(speed_to_us(dir_to_speed_jst2(j2)))
        # ================================
        last_rx_ms = now
        rf_event = True

    # ----- RF estado / failsafe / LED interno -----
    rf_ok = (utime.ticks_diff(now, last_rx_ms) <= KEEPALIVE_TIMEOUT_MS)
    if rf_ok:
        if HAVE_LED:
            try: led.value(0)  # LED APAGADO cuando RF OK
            except: pass
    else:
        esc_write_us(ESC_MID_US)
        servo_write_deg(90)
        if HAVE_LED:
            try: led.value(1)  # LED ENCENDIDO al perder RF
            except: pass

    # Tras un comando RF, saltamos el resto del ciclo para máxima reactividad
    if rf_event:
        utime.sleep_ms(1)
        continue

    # ----- GPS no bloqueante "light" -----
    n = uart_gps.any()
    if n:
        n = min(n, 128)
        data = uart_gps.read(n)
        if data:
            for b in data:
                try: gps.update(chr(b))
                except: pass

    # ----- TCRT Detector v2: umbral adaptativo + Schmitt simétrico -----
    v_raw = adc_tcrt.read_u16()
    v_hp, th = _filtro_y_umbral(v_raw)   # v_hp centrado, th adaptativo
    # Estado actual por Schmitt:
    #   HIGH si v_hp >= +th
    #   LOW  si v_hp <= -th
    #   si queda entre -th y +th, conservamos estado previo (no generamos edge)

    prev_state = state_high
    if state_high is None:
        # inicializa estado con el primer cruce fuerte que aparezca
        if v_hp >= th:
            state_high = True
            last_edge_ms = now
            edge_count = 0
            win_start_ms = 0
        elif v_hp <= -th:
            state_high = False
            last_edge_ms = now
            edge_count = 0
            win_start_ms = 0
    else:
        # evaluar posibles transiciones con antirrebote
        if state_high is True:
            # ahora solo aceptamos transición a LOW si pasa por -th
            if v_hp <= -th and utime.ticks_diff(now, last_edge_ms) >= DEBOUNCE_MS:
                # edge válido HIGH->LOW
                if edge_count == 0:
                    win_start_ms = now
                    edge_count = 1
                else:
                    # verificar ventana total y gap
                    if utime.ticks_diff(now, win_start_ms) <= VENTANA_PATRON_MS and utime.ticks_diff(now, last_edge_ms) <= GAP_MAX_MS:
                        edge_count += 1
                    else:
                        # reinicia ventana con este edge como primero
                        win_start_ms = now
                        edge_count = 1
                last_edge_ms = now
                state_high = False
        else:
            # state_low -> aceptar transición a HIGH si pasa por +th
            if v_hp >= th and utime.ticks_diff(now, last_edge_ms) >= DEBOUNCE_MS:
                # edge válido LOW->HIGH
                if edge_count == 0:
                    win_start_ms = now
                    edge_count = 1
                else:
                    if utime.ticks_diff(now, win_start_ms) <= VENTANA_PATRON_MS and utime.ticks_diff(now, last_edge_ms) <= GAP_MAX_MS:
                        edge_count += 1
                    else:
                        win_start_ms = now
                        edge_count = 1
                last_edge_ms = now
                state_high = True

        # ¿se cerró patrón N-B-N-B? (3 edges alternados dentro de la ventana)
        if edge_count >= 3 and utime.ticks_diff(now, win_start_ms) <= VENTANA_PATRON_MS:
            # ===== VUELTA COMPLETA =====
            vueltas += 1

            last_lap_hora = fmt_hora(gps.timestamp)
            last_lap_lat  = conv_grados(gps.latitude)
            last_lap_lon  = conv_grados(gps.longitude)
            spd_now = vel_kmh(getattr(gps, "speed", None))
            if spd_now is not None:
                velocidades_vuelta.append(spd_now)

            print("\n=== VUELTA {} ===".format(vueltas))
            print("Hora:", last_lap_hora, "Lat:", last_lap_lat, "Lon:", last_lap_lon,
                  "Vel(km/h):", "{:.1f}".format(spd_now) if spd_now is not None else "--")
            print("====================")

            lap_flash_until = utime.ticks_add(now, 1000)

            if vueltas >= VUELTAS_META:
                meta = True
                prom = (sum(velocidades_vuelta)/len(velocidades_vuelta)) if velocidades_vuelta else None
                esc_write_us(ESC_MID_US)
                servo_write_deg(90)
                oled_ganaste(prom)
                print("=== FIN DE CARRERA: GANASTE ===")

            # reinicia contador de edges para el siguiente paso por meta
            edge_count = 0
            win_start_ms = 0
            # no forzamos estado; seguimos con el Schmitt normal

        # timeout de ventana si se quedó a medias
        if edge_count != 0 and utime.ticks_diff(now, win_start_ms) > VENTANA_PATRON_MS:
            edge_count = 0
            win_start_ms = 0

    # Texto de pantalla aproximado del color actual (según v_hp y th)
    if state_high is None:
        txt_color = "--"
    else:
        txt_color = "NEGRO" if state_high else "BLANCO"

    # ----- OLED (limitado a 10 Hz) -----
    if not meta and utime.ticks_diff(now, next_oled_ms) >= 0:
        next_oled_ms = utime.ticks_add(now, OLED_PERIOD_MS)
        if lap_flash_until and utime.ticks_diff(lap_flash_until, now) > 0:
            draw_lap_flash(vueltas, last_lap_hora, last_lap_lat, last_lap_lon)
        else:
            hora = fmt_hora(gps.timestamp)
            sats = gps.satellites_in_use
            spd  = vel_kmh(getattr(gps, "speed", None))
            vtxt = "{:.1f}".format(spd) if spd is not None else "--"
            draw_main(hora, vueltas, vtxt, sats, txt_color, rf_ok)
            if lap_flash_until and utime.ticks_diff(lap_flash_until, now) <= 0:
                lap_flash_until = 0

    # ----- Log (cada 2 s) -----
    if not meta and utime.ticks_diff(now, next_log_ms) >= 0:
        next_log_ms = utime.ticks_add(now, LOG_PERIOD_MS)
        hora = fmt_hora(gps.timestamp)
        sats = gps.satellites_in_use
        spd  = vel_kmh(getattr(gps, "speed", None))
        vtxt = "{:.1f}".format(spd) if spd is not None else "--"
        print(f"[GPS] {hora}  Sats:{sats}  Vel:{vtxt} km/h  Laps:{vueltas}  TCRT:{txt_color}  RF:{'OK' if rf_ok else '--'}  vhp/th:{v_hp}/{th}")

    # Ritmo general (baja latencia)
    utime.sleep_ms(2)
