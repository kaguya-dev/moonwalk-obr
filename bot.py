import RPi.GPIO as GPIO
import smbus # type: ignore
import time
import asyncio 
import threading
import sys

GPIO.setmode(GPIO.BCM)

class TCRT5000Array:
    def __init__(self, pins, pull_up=True):
        """
        pins: lista de pinos GPIO conectados aos sensores
        pull_up: ativa resistor pull-up interno se True
        """
        self.pins = pins
        for pin in self.pins:
            if pull_up:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            else:
                GPIO.setup(pin, GPIO.IN)

    def read_all(self):
        """
        Reads all sensors at once.
        Returns a list of boolean values.
        """
        return [GPIO.input(pin) for pin in self.pins]
    
##APDS CONSTANTS
APDS9960_ADDR = 0x39

# Registradores principais
ENABLE   = 0x80
ATIME    = 0x81
CONTROL  = 0x8F
STATUS   = 0x93
CDATAL   = 0x94  # Red/Green/Blue/Clear (cada cor ocupa 2 bytes)

class APDS9960:
    def __init__(self, bus_channel=1, delaySpeed=0.1, integration_time=0xFF, gain=0x01):
        self.bus_channel = bus_channel
        self.bus = smbus.SMBus(bus_channel)
        self.delaySpeed = delaySpeed
        self.integration_time = integration_time
        self.gain = gain
        self.running = False
        self.latest_data = None
        self.thread = None

        self._init_sensor()

    def _init_sensor(self):
        """Configura registradores básicos do APDS9960"""
        # Tempo de integração e ganho
        self.bus.write_byte_data(APDS9960_ADDR, ATIME, self.integration_time)
        self.bus.write_byte_data(APDS9960_ADDR, CONTROL, self.gain)
        # Liga sensor (Power ON + ALS enable)
        self.bus.write_byte_data(APDS9960_ADDR, ENABLE, 0x03)
        # Espera o sensor inicializar
        time.sleep(0.02)  # 20 ms

    def _data_ready(self):
        """Verifica se há dados válidos prontos"""
        status = self.bus.read_byte_data(APDS9960_ADDR, STATUS)
        return bool(status & 0x01)  # Bit AVALID

    def read_raw_color(self):
        """Lê valores brutos de cor (Clear, Red, Green, Blue)"""
        if not self._data_ready():
            return None  # Ainda não tem dado válido

        data = self.bus.read_i2c_block_data(APDS9960_ADDR, CDATAL, 8)
        c = data[1] << 8 | data[0]
        r = data[3] << 8 | data[2]
        g = data[5] << 8 | data[4]
        b = data[7] << 8 | data[6]
        return {"clear": c, "red": r, "green": g, "blue": b}

    def start_reading(self):
        """Inicia loop de leitura em thread separada"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._update_loop, daemon=True)
            self.thread.start()

    def _update_loop(self):
        """Loop contínuo de leitura"""
        while self.running:
            result = self.read_raw_color()
            if result is not None:
                self.latest_data = result
            time.sleep(self.delaySpeed)

    def stop_reading(self):
        """Para a thread de leitura"""
        self.running = False
        # como é daemon, não precisa join()

class A4988Dual:
    def __init__(self, dir_pins, step_pins, steps_per_rev=(700, 700), delays=(0.005, 0.005)):
        """
        Classe para controle de dois drivers A4988, cada um com delay e passos independentes.
        :param dir_pins: tupla/lista com GPIOs conectados aos pinos DIR dos A4988 (esquerda, direita)
        :param step_pins: tupla/lista com GPIOs conectados aos pinos STEP dos A4988 (esquerda, direita)
        :param steps_per_rev: tupla/lista com passos por volta de cada motor (esquerda, direita)
        :param delays: tupla/lista com delays de cada motor (esquerda, direita)
        """
        self.dir_pins = dir_pins
        self.step_pins = step_pins
        self.steps_per_rev = list(steps_per_rev)
        self.delays = list(delays)

        for pin in self.dir_pins + self.step_pins:
            GPIO.setup(pin, GPIO.OUT)

    def set_direction(self, left_cw=True, right_cw=True):
        GPIO.output(self.dir_pins[0], GPIO.HIGH if left_cw else GPIO.LOW)
        GPIO.output(self.dir_pins[1], GPIO.HIGH if right_cw else GPIO.LOW)

    def setSpeed(self, left_speed, right_speed):
        """
        Configura a velocidade dos motores individualmente.
        1 = rápido (0.0025s), 2 = médio (0.005s), 3 = lento (0.01s)
        """
        speed_map = {1: 0.0025, 2: 0.005, 3: 0.01}
        self.delays[0] = speed_map.get(left_speed, 0.005)
        self.delays[1] = speed_map.get(right_speed, 0.005)
        if left_speed not in speed_map or right_speed not in speed_map:
            print("Invalid Value, Standard Speed Set")

    def step(self, steps=(1, 1), delays=None):
        """
        Dá passos independentes para cada motor.
        :param steps: tupla/lista com número de passos para cada motor (esquerda, direita)
        :param delays: tupla/lista com delays para cada motor (opcional)
        """
        delays = delays if delays is not None else self.delays
        max_steps = max(steps)
        for i in range(max_steps):
            for idx in (0, 1):
                if i < steps[idx]:
                    GPIO.output(self.step_pins[idx], GPIO.HIGH)
            time.sleep(min(delays))
            for idx in (0, 1):
                if i < steps[idx]:
                    GPIO.output(self.step_pins[idx], GPIO.LOW)
            time.sleep(min(delays))

    def rotate(self, revolutions=(1, 1)):
        """
        Gira cada motor o número de voltas especificado.
        :param revolutions: tupla/lista com voltas para cada motor (esquerda, direita)
        """
        steps = [int(self.steps_per_rev[i] * revolutions[i]) for i in (0, 1)]
        self.step(steps)

    def move_steps_time(self, steps, duration):
        """
        Move ambos os motores um número de passos em um tempo total (segundos).
        Os motores são sincronizados para terminar juntos.
        :param steps: tupla/lista com passos para cada motor (esquerda, direita)
        :param duration: tempo total (segundos) para completar os passos
        """
        steps = list(steps)
        if duration <= 0 or steps[0] <= 0 or steps[1] <= 0:
            return
        delays = []
        for s in steps:
            delays.append(duration / s if s > 0 else duration)
        max_steps = max(steps)
        for i in range(max_steps):
            for idx in (0, 1):
                if i < steps[idx]:
                    GPIO.output(self.step_pins[idx], GPIO.HIGH)
            time.sleep(min(delays))
            for idx in (0, 1):
                if i < steps[idx]:
                    GPIO.output(self.step_pins[idx], GPIO.LOW)
            time.sleep(min(delays))

    def forward(self, steps, duration):
        """
        Move para frente (ambos motores para frente) por um número de passos em um tempo.
        """
        self.set_direction(True, True)
        self.move_steps_time((steps, steps), duration)

    def backward(self, steps, duration):
        """
        Move para trás (ambos motores para trás) por um número de passos em um tempo.
        """
        self.set_direction(False, False)
        self.move_steps_time((steps, steps), duration)

    def left(self, steps, duration):
        """
        Gira no próprio eixo para a esquerda (motor esquerdo para trás, direito para frente).
        """
        self.set_direction(False, True)
        self.move_steps_time((steps, steps), duration)

    def right(self, steps, duration):
        """
        Gira no próprio eixo para a direita (motor esquerdo para frente, direito para trás).
        """
        self.set_direction(True, False)
        self.move_steps_time((steps, steps), duration)

    def curve_left(self, steps_left, steps_right, duration):
        """
        Curva aberta para a esquerda: esquerdo mais devagar, direito mais rápido.
        """
        self.set_direction(True, True)
        self.move_steps_time((steps_left, steps_right), duration)

    def curve_right(self, steps_left, steps_right, duration):
        """
        Curva aberta para a direita: direito mais devagar, esquerdo mais rápido.
        """
        self.set_direction(True, True)
        self.move_steps_time((steps_left, steps_right), duration)

class HC_SR04:
    def __init__(self, trigger_pin, echo_pin, timeout=0.04):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = timeout

        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def measure_distance(self):
        # Garante que o trigger está baixo
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.0002)  # 200 µs

        # Envia pulso de trigger
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)  # 10 µs
        GPIO.output(self.trigger_pin, False)

        # Espera pelo pulso de echo
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
            if time.time() - start_time > self.timeout:
                return None  # Timeout

        while GPIO.input(self.echo_pin) == 1:
            end_time = time.time()
            if time.time() - start_time > self.timeout:
                return None  # Timeout

        elapsed = end_time - start_time
        distance = (elapsed * 34300) / 2  # Distância em cm
        return distance

    def is_object_at_distance(self, target_distance, tolerance=2.0):
        """
        Returns True if an object is detected within target_distance ± tolerance (in cm).
        """
        distance = self.measure_distance()
        if distance is None:
            return False
        return abs(distance - target_distance) <= tolerance

def async_update_info(irValues, irSum, irPro, apds_left=None, apds_right=None, hc_sr04_list=None, hc_sr04_targets=None):
    # Use carriage return and flush to overwrite the same line
    info = (
        f"IR: {irValues} = {irSum} | IR Pro: {irPro} | "
        f"Motor L: {motores.delays[0]:.4f}s | Motor R: {motores.delays[1]:.4f}s"
    )
    if apds_left is not None:
        green_l = apds_left.get("green", "-")
        clear_l = apds_left.get("clear", "-")
        info += f" | Left Green: {green_l} | Left Darkness: {clear_l}"
    if apds_right is not None:
        green_r = apds_right.get("green", "-")
        clear_r = apds_right.get("clear", "-")
        info += f" | Right Green: {green_r} | Right Darkness: {clear_r}"
    if hc_sr04_list is not None:
        for idx, hc_sr04 in enumerate(hc_sr04_list):
            distance = hc_sr04.measure_distance()
            if distance is not None:
                info += f" | HC-SR04[{idx}]: {distance:.1f}cm"
                if hc_sr04_targets is not None and idx < len(hc_sr04_targets):
                    presence = hc_sr04.is_object_at_distance(hc_sr04_targets[idx])
                    info += f" | Obj@{hc_sr04_targets[idx]}cm: {'YES' if presence else 'NO'}"
            else:
                info += f" | HC-SR04[{idx}]: ---"
    sys.stdout.write('\r' + info.ljust(180))
    sys.stdout.flush()

# --------
# Exemplo
# --------

#Motores (A4988Dual espera listas de pinos DIR e STEP)
motores = A4988Dual(dir_pins=[23, 10], step_pins=[24, 9])

# Sensor 1 no barramento padrão (i2c-1)
sensor_Left = APDS9960(bus_channel=1, delaySpeed=0.5)
# Sensor 2 em barramento emulado (precisa ter configurado i2c-3)
sensor_Right = APDS9960(bus_channel=3, delaySpeed=0.5)

irSensors = TCRT5000Array(pins=[4, 17, 18, 14 ,15])

ultrassonic_left = HC_SR04(trigger_pin=27, echo_pin=22)
ultrassonic_right = HC_SR04(trigger_pin=5, echo_pin=6)
ultrassonic_frontal = HC_SR04(trigger_pin=13, echo_pin=19)
hc_sr04_sensors = [ultrassonic_left, ultrassonic_right, ultrassonic_frontal]


sensor_Left.start_reading()
sensor_Right.start_reading()
try:
    lost_line_time = None
    lost_line_timeout = 2.5  # seconds

    while True:
        async_update_info(
            irSensors.read_all(),
            sum(irSensors.read_all()),
            sum(val * (i + 1) for i, val in enumerate(irSensors.read_all())) - 3,
            apds_left=sensor_Left.latest_data,
            apds_right=sensor_Right.latest_data,
            hc_sr04_list=hc_sr04_sensors
        )
        ir_values = irSensors.read_all()
        ir_sum = sum(val * (i + 1) for i, val in enumerate(ir_values))
        ir_pro = ir_sum - 3

        green_left = sensor_Left.latest_data and sensor_Left.latest_data.get("green", 0) > 1000
        green_right = sensor_Right.latest_data and sensor_Right.latest_data.get("green", 0) > 1000
        central_black = ir_values[2] == 0

        # Detect if all sensors are "off" (tape lost)
        tape_lost = all(val == 1 for val in ir_values)

        if tape_lost:
            if lost_line_time is None:
                lost_line_time = time.time()
            elapsed = time.time() - lost_line_time
            if elapsed < lost_line_timeout:
                # Keep moving forward
                motores.setSpeed(2, 2)
                motores.rotate((0.25, 0.25))
                continue
            else:
                # Stop and go back
                motores.setSpeed(3, 3)
                motores.backward(200, lost_line_timeout)
                lost_line_time = None  # Reset after action
                continue
        else:
            lost_line_time = None  # Reset if tape found

        if central_black and green_left and green_right:
            motores.backward(200, 1)
            motores.left(350, 1.5)
        elif central_black and green_left:
            motores.left(100, 0.5)
        elif central_black and green_right:
            motores.right(100, 0.5)
        elif ir_pro == 0:
            motores.setSpeed(2, 2)
        elif ir_pro > 0:
            motores.setSpeed(1, 3)
        elif ir_pro < 0:
            motores.setSpeed(3, 1)
        else:
            motores.setSpeed(3, 3)

        motores.rotate((0.25, 0.25))

except KeyboardInterrupt:
    GPIO.cleanup()
