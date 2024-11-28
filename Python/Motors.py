import RPi.GPIO as GPIO

# Настройка режима нумерации пинов
GPIO.setmode(GPIO.BCM)

# Пины для моторов
motor1_in1 = 17
motor1_in2 = 18
motor1_en = 22

motor2_in1 = 23
motor2_in2 = 24
motor2_en = 25

motor3_in1 = 12
motor3_in2 = 16
motor3_en = 20

motor4_in1 = 26
motor4_in2 = 19
motor4_en = 21

# Инициализация пинов
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)
GPIO.setup(motor1_en, GPIO.OUT)

GPIO.setup(motor2_in1, GPIO.OUT)
GPIO.setup(motor2_in2, GPIO.OUT)
GPIO.setup(motor2_en, GPIO.OUT)

GPIO.setup(motor3_in1, GPIO.OUT)
GPIO.setup(motor3_in2, GPIO.OUT)
GPIO.setup(motor3_en, GPIO.OUT)

GPIO.setup(motor4_in1, GPIO.OUT)
GPIO.setup(motor4_in2, GPIO.OUT)
GPIO.setup(motor4_en, GPIO.OUT)

# PWM для регулировки скорости
pwm1 = GPIO.PWM(motor1_en, 100)
pwm2 = GPIO.PWM(motor2_en, 100)
pwm3 = GPIO.PWM(motor3_en, 100)
pwm4 = GPIO.PWM(motor4_en, 100)

# Запуск PWM с начальной скоростью 50%
pwm1.start(50)
pwm2.start(50)
pwm3.start(50)
pwm4.start(50)

# Комбинированные функции для движения робота
def robot_forward():
    motor1_forward()
    motor2_forward()
    motor3_forward()
    motor4_forward()

def robot_backward():
    motor1_backward()
    motor2_backward()
    motor3_backward()
    motor4_backward()

def robot_right_turn():
    motor1_backward()
    motor2_forward()
    motor3_backward()
    motor4_forward()

def robot_left_turn():
    motor1_forward()
    motor2_backward()
    motor3_forward()
    motor4_backward()

def robot_spin(direction):
    if direction == "left":
        # Левый поворот
        motor1_forward()
        motor2_backward()
        motor3_forward()
        motor4_backward()
    elif direction == "right":
        # Правый поворот
        motor1_backward()
        motor2_forward()
        motor3_backward()
        motor4_forward()
    else:
        raise ValueError(f"Неправильное направление: {direction}")

def robot_stop():
    motor1_stop()
    motor2_stop()
    motor3_stop()
    motor4_stop()

finally:
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()
    GPIO.cleanup()  # Освобождение ресурсов GPIO
