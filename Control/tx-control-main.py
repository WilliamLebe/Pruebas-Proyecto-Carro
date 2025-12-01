# ======================================= 
# WILLIAM LEON
# INGENIERIA EN TELECOMUNICACIONES
# TRANSMISOR CONTROL v3
# NRF24L01 + OLED 128x32 + 2 JOYSTICKS
#
# Enlace:
#   - Dirección RF: 0xE7E7E7E7E7
#   - Canal: 100  (debe coincidir con el CARRO RX v8)
#   - Potencia: 0 dBm (máxima)
#
# Lógica:
#   - Lee 2 joysticks por ADC (0..65535)
#   - Mapea a: Izquierda/Centro/Derecha y Reversa/Centro/Adelante
#   - Envía por NRF24L01 tramas de texto:
#       "JST1:<dir1>;JST2:<dir2>"
#   - Muestra en OLED:
#       J1: xxx
#       J2: xxx
#       RF:OK / RF:--
#   - LED:
#       Apagado -> último envío con ACK OK
#       Encendido -> fallo de ACK (MAX_RT o timeout)
# =======================================
from machine import Pin, SPI, I2C, ADC
from ssd1306 import SSD1306_I2C
import utime

# ===== LED interno =====
try:
    led = Pin("LED", Pin.OUT)
except:
    led = Pin(25, Pin.OUT)  # fallback en Pico no-W
led.value(0)   # LED apagado al inicio

# ===== Pines =====
# nRF24L01 por SPI0
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=1_000_000)
csn = Pin(5, Pin.OUT, value=1)
ce  = Pin(6, Pin.OUT, value=0)

# OLED SSD1306 por I2C1 (SCL=15, SDA=14) -> 128x32
i2c  = I2C(1, scl=Pin(15), sda=Pin(14), freq=400_000)
oled = SSD1306_I2C(128, 32, i2c)

def show_status(d1, d2, rf_ok):
    """
    Muestra:
      Línea 0: J1:<dir>
      Línea 1: J2:<dir>
      Línea 2: RF:OK / RF:--
    OLED es 128x32 → y=0,10,20 están dentro del alto.
    """
    oled.fill(0)
    oled.text("J1: " + d1[:10], 0, 0)
    oled.text("J2: " + d2[:10], 0, 10)
    oled.text("RF:OK" if rf_ok else "RF:--", 0, 20)
    oled.show()

# ===== Joysticks (ADC) =====
ADC0_PIN = 26   # JST1: Izquierda/Derecha
ADC1_PIN = 27   # JST2: Adelante/Reversa
jst1 = ADC(ADC0_PIN)
jst2 = ADC(ADC1_PIN)

# Calibración/umbrales (read_u16 -> 0..65535)
MID      = 32768
DEADZONE = 4000     # +/- zona muerta
LOW_T    = MID - DEADZONE
HIGH_T   = MID + DEADZONE

def dir_lr(val_u16):
    # Izquierda / Centro / Derecha
    if val_u16 < LOW_T:
        return "Izquierda"
    elif val_u16 > HIGH_T:
        return "Derecha"
    else:
        return "Centro"

def dir_fb(val_u16):
    # Reversa / Centro / Adelante
    if val_u16 < LOW_T:
        return "Reversa"
    elif val_u16 > HIGH_T:
        return "Adelante"
    else:
        return "Centro"

# ===== nRF: registros / comandos =====
CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR = 0x00,0x01,0x02,0x03,0x04
RF_CH, RF_SETUP, STATUS                        = 0x05,0x06,0x07
RX_ADDR_P0, TX_ADDR                            = 0x0A,0x10
RX_PW_P0, FIFO_STATUS                          = 0x11,0x17
RX_DR, TX_DS, MAX_RT                           = 0x40,0x20,0x10
W_REGISTER, R_REGISTER, REGISTER_MASK          = 0x20,0x00,0x1F
FLUSH_TX, FLUSH_RX, W_TX_PAYLOAD               = 0xE1,0xE2,0xA0

# ===== Parámetros RF (coinciden con el RX) =====
ADDR      = b'\xe7\xe7\xe7\xe7\xe7'  # 5 bytes
CHANNEL   = 100                      # <<< IMPORTANTE: igual que CARRO v8
PAYLOAD   = 32

# ===== utilidades de registro =====
def reg_write(reg, val):
    csn(0)
    spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK), val]))
    csn(1)

def reg_read(reg):
    csn(0)
    spi.write(bytearray([R_REGISTER | (reg & REGISTER_MASK)]))
    b = spi.read(1)
    csn(1)
    return b[0]

def reg_write_bytes(reg, data):
    csn(0)
    spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK)]) + data)
    csn(1)

def flush():
    csn(0); spi.write(bytearray([FLUSH_TX])); csn(1)
    csn(0); spi.write(bytearray([FLUSH_RX])); csn(1)
    reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

# ===== Init & configuración NRF =====
utime.sleep_ms(300)

# Power-down inicial, luego configuración completa
reg_write(CONFIG, 0x0C)        # EN_CRC=1, PWR_UP=0
reg_write(SETUP_AW, 0x03)      # 5 bytes
reg_write(EN_AA, 0x01)         # Auto-ACK en pipe0
reg_write(EN_RXADDR, 0x01)     # habilita pipe0
reg_write(SETUP_RETR, 0x3F)    # max retries + delay
reg_write(RF_CH, CHANNEL)      # canal

# RF_SETUP: 250 kbps (bit5=1) + potencia
# Potencia (bits 2:1):
#   0b00 -> -18 dBm
#   0b01 -> -12 dBm
#   0b10 ->  -6 dBm
#   0b11 ->   0 dBm (máxima)
PWR_BITS = 0b11   
reg_write(RF_SETUP, 0x20 | (PWR_BITS << 1))

reg_write_bytes(TX_ADDR,    ADDR)
reg_write_bytes(RX_ADDR_P0, ADDR)   # necesario para ACK
reg_write(RX_PW_P0, PAYLOAD)

# Power up en PTX (PRIM_RX=0)
reg_write(CONFIG, 0x0A)        # EN_CRC=1, PWR_UP=1, PRIM_RX=0
utime.sleep_ms(2)
flush()

# Mensaje inicial
show_status("Centro", "Centro", rf_ok=False)
utime.sleep_ms(500)

# ===== Timings =====
# Cada cuánto se obliga a enviar aunque no cambien los joysticks
KEEPALIVE_MS = 300   # antes 500 -> más responsivo
# Periodo de lectura de joysticks
PERIOD_MS    = 80    # antes 120 -> más fluido

last_payload = None
last_send_ms = utime.ticks_ms()
last_ack_ok  = False   # último resultado de ACK

def build_payload(d1, d2):
    # Texto corto y claro para RX y OLED
    # Ej: "JST1:Izquierda;JST2:Adelante"
    msg = f"JST1:{d1};JST2:{d2}"
    s = msg.encode()
    if len(s) > PAYLOAD:
        s = s[:PAYLOAD]
    pad = s + b"\x00" * (PAYLOAD - len(s))
    return pad, msg

# ===== Bucle principal =====
while True:
    t0 = utime.ticks_ms()

    # Leer joysticks
    v1 = jst1.read_u16()
    v2 = jst2.read_u16()
    d1 = dir_lr(v1)  # Izquierda/Centro/Derecha
    d2 = dir_fb(v2)  # Reversa/Centro/Adelante

    # Mostrar en pantalla siempre
    show_status(d1, d2, last_ack_ok)

    # Construir payload
    payload, msg = build_payload(d1, d2)

    # Sólo enviar si cambió o por keep-alive
    changed = (msg != last_payload)
    expired = utime.ticks_diff(t0, last_send_ms) >= KEEPALIVE_MS
    if changed or expired:
        # Cargar y transmitir
        csn(0)
        spi.write(bytearray([W_TX_PAYLOAD]) + payload)
        csn(1)
        ce(1)
        utime.sleep_us(15)
        ce(0)

        # Esperar TX_DS o MAX_RT (hasta 100 ms)
        t1 = utime.ticks_ms()
        ok = False
        while utime.ticks_diff(utime.ticks_ms(), t1) < 100:
            st = reg_read(STATUS)
            if st & TX_DS:
                reg_write(STATUS, TX_DS)
                ok = True
                break
            if st & MAX_RT:
                reg_write(STATUS, MAX_RT)
                flush()
                ok = False
                break

        # Guardar estado de ACK
        last_ack_ok = ok

        # Log por consola (Thonny)
        print(f'TX: "{msg}" | ACK: {ok}')

        # LED: ok=True -> apagado, ok=False -> encendido
        if ok:
            led.value(0)  # comunicación OK
        else:
            led.value(1)  # posible pérdida de enlace RF

        last_payload = msg
        last_send_ms = t0

    # Periodo de muestreo
    while utime.ticks_diff(utime.ticks_ms(), t0) < PERIOD_MS:
        utime.sleep_ms(1)
