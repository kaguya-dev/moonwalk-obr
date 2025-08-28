import RPi.GPIO as GPIO
import time

# Usar numeração BCM (GPIO)
GPIO.setmode(GPIO.BCM)

# Definição dos pinos dos motores
DIR1 = 23    # Motor 1 - DIR
STEP1 = 24   # Motor 1 - STEP

DIR2 = 10   # Motor 2 - DIR
STEP2 = 9  # Motor 2 - STEP

# Configuração dos pinos como saída
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)

# Função para mover dois motores sincronizados, cada um com seus próprios passos
def stepper_move_sync(DIR1, STEP1, dir1, steps1, DIR2, STEP2, dir2, steps2, delay=0.001):
    """
    DIR1, STEP1: pinos do motor 1
    dir1: direção do motor 1
    steps1: número de passos do motor 1
    DIR2, STEP2: pinos do motor 2
    dir2: direção do motor 2
    steps2: número de passos do motor 2
    delay: tempo entre pulsos
    """
    GPIO.output(DIR1, dir1)
    GPIO.output(DIR2, dir2)
    max_steps = max(steps1, steps2)
    count1 = count2 = 0
    for _ in range(max_steps):
        if count1 < steps1:
            GPIO.output(STEP1, GPIO.HIGH)
        if count2 < steps2:
            GPIO.output(STEP2, GPIO.HIGH)
        time.sleep(delay)
        if count1 < steps1:
            GPIO.output(STEP1, GPIO.LOW)
            count1 += 1
        if count2 < steps2:
            GPIO.output(STEP2, GPIO.LOW)
            count2 += 1
        time.sleep(delay)

def move_front(steps1, steps2=None, delay=0.001):
    # Ambos motores para frente (horário)
    if steps2 is None:
        steps2 = steps1
    stepper_move_sync(DIR1, STEP1, 1, steps1, DIR2, STEP2, 1, steps2, delay)

def move_back(steps1, steps2=None, delay=0.001):
    # Ambos motores para trás (anti-horário)
    if steps2 is None:
        steps2 = steps1
    stepper_move_sync(DIR1, STEP1, 0, steps1, DIR2, STEP2, 0, steps2, delay)

def turn_left(steps1, steps2=None, delay=0.001):
    # Motor 1 para trás, Motor 2 para frente (gira no próprio eixo)
    if steps2 is None:
        steps2 = steps1
    stepper_move_sync(DIR1, STEP1, 0, steps1, DIR2, STEP2, 1, steps2, delay)

def turn_right(steps1, steps2=None, delay=0.001):
    # Motor 1 para frente, Motor 2 para trás (gira no próprio eixo)
    if steps2 is None:
        steps2 = steps1
    stepper_move_sync(DIR1, STEP1, 1, steps1, DIR2, STEP2, 0, steps2, delay)

try:
    print("Frente...")
    move_front(200)
    time.sleep(1)
    print("Trás...")
    move_back(200)
    time.sleep(1)
    print("Esquerda...")
    turn_left(100)
    time.sleep(1)
    print("Direita...")
    turn_right(100)
    time.sleep(1)
finally:
    GPIO.cleanup()
