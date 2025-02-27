import asyncio
import RPi.GPIO as GPIO
import aiohttp
import numpy as np
import cv2
import os
import time
import serial
from pyzbar.pyzbar import decode
from PIL import Image
import math

# Адреса для подключения к серверу
URL = "http://192.168.135.124:5298/orders/get/robot"
UPDATE_URL = "http://192.168.135.124:5298/orders/update/loc"

# 1/2/3 Контейнер для выгрузки
case_out = 0

# Основные глобальные переменные для доступа из любых частей приложения
qr_data = None
order_matrix = None

# Инициализация
ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

# Перечень используемых портов GPIO
GPIO.setmode(GPIO.BCM)
TRIG_FRONT_LEFT = 23
ECHO_FRONT_LEFT = 24
TRIG_FRONT_RIGHT = 25
ECHO_FRONT_RIGHT = 26
TRIG_BACK_LEFT = 14
ECHO_BACK_LEFT = 15
TRIG_BACK_RIGHT = 5
ECHO_BACK_RIGHT = 6
TRIG_ARM = 2
ECHO_ARM = 3
LOWER_LOW_MOTOR = 13
HIGH_LOW_MOTOR = 19

# Настройка GPIO портов
GPIO.setup(TRIG_FRONT_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_FRONT_RIGHT, GPIO.IN)
GPIO.setup(TRIG_FRONT_LEFT, GPIO.OUT)
GPIO.setup(ECHO_FRONT_LEFT, GPIO.IN)
GPIO.setup(TRIG_BACK_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_BACK_RIGHT, GPIO.IN)
GPIO.setup(TRIG_BACK_LEFT, GPIO.OUT)
GPIO.setup(ECHO_BACK_LEFT, GPIO.IN)
GPIO.setup(LOWER_LOW_MOTOR, GPIO.OUT)
GPIO.setup(HIGH_LOW_MOTOR, GPIO.OUT)

# Настройка подключения сервоприводов
SERVO_PINS = [17, 18, 27, 22]
GPIO.setup(SERVO_PINS, GPIO.OUT)

# Инициализация PWM для сервоприводов
SERVO_FREQ = 50
servos = [GPIO.PWM(pin, SERVO_FREQ) for pin in SERVO_PINS]
for servo in servos:
    servo.start(0)


# Класс для управления сервами
class MultiServoController:
    def __init__(self, pins=[17, 18, 27, 22], freq=50):
        self.pins = pins
        self.servos = []

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Инициализация всех сервоприводов
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, freq)
            pwm.start(0)
            self.servos.append(pwm)

    # Установка сервоприводов
    def set_angle(self, servo_num, angle):
        angle = max(0, min(180, angle))
        duty = 2.5 + (angle / 180) * 10
        self.servos[servo_num].ChangeDutyCycle(duty)
        sleep(0.3)


# Контроллер сервоприводов
controller = MultiServoController()


# Получение данных о заказах
async def fetch_order(session):
    try:
        async with session.get(URL) as response:
            if response.status == 200:
                return await response.json()
            print(f"Ошибка запроса: {response.status}")
    except Exception as e:
        print(f"Ошибка соединения: {e}")
    return None


# Обработка данных о заказе
async def process_order():
    global order_matrix
    async with aiohttp.ClientSession() as session:
        while True:
            order_data = await fetch_order(session)
            if order_data:
                items = order_data.get("items", [])
                order_matrix = np.array([
                    [item["goods"]["name"], item["quantity"], item["rack"], item["cell"], item["shelf"]] for item in
                    items
                ]) if items else np.array([])
                break
            await asyncio.sleep(2)


# Перевод заказа в статус завершен
async def put_order_done():
    url = "http://192.168.135.124:5298/orders/put/done"
    async with aiohttp.ClientSession() as session:
        async with session.put(url) as response:
            return await response.text()


# Фильтрация полученной матрицы заказов
async def filter_order_matrix():
    global order_matrix
    if order_matrix is not None and order_matrix.size > 0:
        order_matrix = np.array([
            [item[0], item[1], item[2], item[3], item[4]] for item in order_matrix if int(item[1]) > 0
        ])
        print("Отфильтрованный order_matrix:", order_matrix)
    else:
        print("order_matrix пуст или None, фильтрация не выполнена.")


# Отправка напряжений для моторов
def send_to_arduino(v1, v2, v3, v4):
    global ser

    # Формируем строку данных
    data = f"{v1},{v2},{v3},{v4}\n"

    # Отправляем данные
    ser.write(data.encode())
    print(f"Sent: {data.strip()}")


# Установка углов на сервы
def set_servo_angle(servo_id, angle):
    """Установка угла для сервопривода"""
    global controller

    if 0 <= servo_id < len(servos) and 0 <= angle <= 180:
        controller.set_angle(servo_id, angle)
    else:
        print(f"Ошибка: Недопустимый servo_id или angle")


# Чтение ультразвуковых датчиков
def measure_distance(TRIG, ECHO):
    # Убедимся, что пин TRIG в низком состоянии
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    pulse_end = None
    pulse_start = None

    # Запускаем ультразвуковой сигнал
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # Подаем импульс на 10 микросекунд
    GPIO.output(TRIG, False)

    # Засекаем время, которое требуется эхо, чтобы вернуться
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Расстояние рассчитывается по формуле:
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # 17150 — это скорость звука в см/с
    distance = round(distance, 2)  # Округляем до 2 знаков после запятой

    return distance


# Поворот
def turn(degrees):
    speed = 32  # Градус/секунда
    duration = abs(degrees) / speed  # Время поворота

    # Определяем направление поворота
    if degrees > 0:
        send_to_arduino(70, 70, -70, -70)  # Поворот вправо
    else:
        send_to_arduino(-70, -70, 70, 70)  # Поворот влево

    time.sleep(duration)  # Ждем завершения поворота
    send_to_arduino(0, 0, 0, 0)  # Остановка
    sleep(0.1)


# Движение
def move(side, length):
    speed = 60  # Сантиметр/секунда
    duration = length / speed

    match side:
        case "forward":
            send_to_arduino(255, 255, 255, 255)
        case "back":
            send_to_arduino(-255, -255, -255, -255)
        case "left":
            send_to_arduino(-255, 255, 255, -255)
        case "right":
            send_to_arduino(255, -255, -255, 255)

    sleep(duration)

    if side == "left" or "right":
        sleep(0.15)

    send_to_arduino(0, 0, 0, 0)
    sleep(0.1)


# Движение по ультразвуковым датчикам
def move_ultrasonic_sensor(side, expression, value):
    match side:
        case "forward":
            match expression:
                case "more":
                    while measure_distance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT) > value:
                        send_to_arduino(255, 255, 255, 255)
                case "less":
                    while measure_distance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT) < value:
                        send_to_arduino(255, 255, 255, 255)
        case "back":
            match expression:
                case "more":
                    while measure_distance(TRIG_BACK_RIGHT, ECHO_BACK_RIGHT) > value:
                        send_to_arduino(-255, -255, -255, -255)
                case "less":
                    while measure_distance(TRIG_BACK_RIGHT, ECHO_BACK_RIGHT) < value:
                        send_to_arduino(-255, -255, -255, -255)
        case "left":
            match expression:
                case "more":
                    while measure_distance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT) >= value:
                        send_to_arduino(-255, 255, 255, -255)
                case "less":
                    while measure_distance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT) <= value:
                        send_to_arduino(-255, 255, 255, -255)
        case "right":
            match expression:
                case "more":
                    while measure_distance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT) >= value:
                        send_to_arduino(255, -255, -255, 255)
                case "less":
                    while measure_distance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT) <= value:
                        send_to_arduino(255, -255, -255, 255)


    if side == "left" or "right":
        sleep(0.17)

    send_to_arduino(0, 0, 0, 0)  # Остановка перед следующим этапом
    sleep(0.1)


# Движение на старт
def go_start_position():
    move_ultrasonic_sensor("forward", "more", 20)
    move_ultrasonic_sensor("left", "less", 20)
    move_ultrasonic_sensor("forward", "more", 20)
    move_ultrasonic_sensor("left", "less", 20)


# Движение на финиш
def go_finish_position():
    turn(90)
    move_ultrasonic_sensor("forward", "more", 20)
    move("right", 145)
    move_ultrasonic_sensor("forward", "more", 20)
    move_ultrasonic_sensor("left", "less", 20)
    move("forward", 85)
    turn(90)
    move_ultrasonic_sensor("forward", "more", 30)


# Движение к ряду
async def go_rack(rack_number):
    match rack_number:
        case 1:
            turn(-90)
            move("right", 60)
            move_ultrasonic_sensor("forward", "more", 30)
            move_ultrasonic_sensor("back", "less", 30)
            for i in [1, 2, 3]:
                await cell()
                if i != 3:
                    move("right", 30)
        case 2:
            turn(180)
            move_ultrasonic_sensor("forward", "more", 30)
            move_ultrasonic_sensor("back", "less", 30)
            for i in [1, 2, 3]:
                await cell()
                if i != 3:
                    move("right", 30)
        case 3:
            move("right", 60)
            turn(-90)
            move_ultrasonic_sensor("right", "more", 20)
            move_ultrasonic_sensor("right", "less", 20)
            turn(90)
            move_("left", 60)
            move_ultrasonic_sensor("forward", "more", 30)
            move_ultrasonic_sensor("back", "less", 30)
            for i in [1, 2, 3]:
                await cell()
                if i != 3:
                    move("left", 30)


# Обработка заказов в конкретной ячейке
async def cell():
    for i in [1, 2]:
        if i == 1:
            set_servo_angle(0, 80)
            sleep(0.3)
            set_servo_angle(1, 70)
            sleep(0.3)
            set_servo_angle(2, 165)
            set_servo_angle(3, 120)
        else:
            set_servo_angle(0, 150)
            sleep(0.3)
            set_servo_angle(1, 60)
            sleep(0.3)
            set_servo_angle(2, 90)
            set_servo_angle(3, 120)

        read_qr_code()
        if str(qr_data).lower in order_matrix[:, 0]:
            index = np.where(order_matrix[:, 0] == qr_data)[0][0]
            count = order_matrix[index, 1]
            await get_object(count, i, index, str(qr_data).lower)


# Взятие объекта
async def get_object(count, shelf, index, name):
    global order_matrix

    while count > 0:
        if track_lr() == "Non object":
            break

        set_servo_angle(3, 120)
        if shelf == 1:
            if track_lr("корзина") == "Non object":
                track_lr(name)
                theta1, theta2 = calculate_extension(80, 70, 165, get_distance())
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
            else:
                theta1, theta2 = calculate_extension(80, 70, 165,  5)
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
                set_servo_angle(2, 40)
                track_lr(name)
                theta1, theta2 = calculate_extension(theta1, theta2, 40,  get_distance())
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
        else:
            track_lr(name)
            theta1, theta2 = calculate_extension(150, 60, 90, get_distance())
            set_servo_angle(0, theta1)
            set_servo_angle(1, theta2)

        time.sleep(0.3)
        set_servo_angle(3, 0)
        put_bag(shelf)

        if order_matrix[index, 1] > 1:
            order_matrix[index, 1] -= 1
        else:
            order_matrix = np.delete(order_matrix, index, axis=0)
        count -= 1


# Подсчет удлинения
def calculate_extension(theta1, theta2, theta3, d):
    # Длины плеч
    L1, L2 = 23, 17

    # Преобразуем углы в радианы
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)

    # Текущие координаты конца второго плеча
    X = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    Y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

    # Смещение вдоль направления третьего плеча
    X_new = X + d * math.cos(theta1 + theta2 + theta3)
    Y_new = Y + d * math.sin(theta1 + theta2 + theta3)

    # Новый угол theta2'
    cos_theta2_new = (X_new ** 2 + Y_new ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    theta2_new = math.acos(cos_theta2_new)

    # Новый угол theta1'
    theta1_new = math.atan2(Y_new, X_new) - math.atan2(L2 * math.sin(theta2_new), L1 + L2 * math.cos(theta2_new))

    # Преобразуем обратно в градусы
    theta1_new = math.degrees(theta1_new)
    theta2_new = math.degrees(theta2_new)

    return theta1_new, theta2_new


# Кладем деталь в корзину
def put_bag(shelf):
    if shelf == 2:
        set_servo_angle(0, 140)
        sleep(0.3)
        set_servo_angle(1, 180)
        sleep(0.3)
        set_servo_angle(2, 40)
        move_ultrasonic_sensor("back", "more", 10)
        set_servo_angle(1, 30)
        set_servo_angle(3, 140)
        set_servo_angle(1, 60)
        move_ultrasonic_sensor("forward", "less", 30)
        move_ultrasonic_sensor("forward", "more", 30)
    else:
        move_ultrasonic_sensor("back", "more", 10)
        set_servo_angle(0, 140)
        sleep(0.3)
        set_servo_angle(1, 30)
        sleep(0.3)
        set_servo_angle(2, 40)
        set_servo_angle(3, 140)
        set_servo_angle(1, 60)
        move_ultrasonic_sensor("forward", "less", 30)
        move_ultrasonic_sensor("forward", "more", 30)


# Выгрузка
def discharge(box):
    match box:
        case 1:
            move("left", 13)
        case 2:
            pass
        case 3:
            move("right", 13)

    # Складываемся и опускаемся вниз
    set_servo_angle(0, 150)
    set_servo_angle(1, 30)
    set_servo_angle(2, 40)
    set_servo_angle(3, 120)
    GPIO.output(LOWER_LOW_MOTOR, GPIO.HIGH)
    sleep(8)
    GPIO.output(LOWER_LOW_MOTOR, GPIO.LOW)

    # Выравниваемся
    while track_servo() != "Non object":
        # Выдвигаемся
        theta1, theta2 = calculate_extension(150, 30, 40, get_distance())
        set_servo_angle(0, theta1)
        set_servo_angle(1, theta2)
        sleep(0.3)

        # Хватаем и поднимаемся
        set_servo_angle(3, 0)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)

        # Подносим на уровне корзины
        set_servo_angle(0, 150)
        set_servo_angle(1, 60)
        set_servo_angle(2, 90)

        # Подъезжаем, опускаем и отъезжаем
        move("forward", 25)
        set_servo_angle(3, 80)
        sleep(0.5)
        move("back", 25)

        # Складываемся
        set_servo_angle(0, 140)
        set_servo_angle(2, 40)
        set_servo_angle(1, 60)
        set_servo_angle(3, 120)

        # Опускаемся
        GPIO.output(LOWER_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(LOWER_LOW_MOTOR, GPIO.LOW)


# Позиционирование на детали на полке
def track_lr(name=None):
    reference_folder = "/home/rjd/rzd"
    match name:
        case "прокладки":
            reference_folder = "/home/rjd/pro"
        case "гайки":
            reference_folder = "/home/rjd/gay"
        case "болты":
            reference_folder = "/home/rjd/bol"
        case "бруски":
            reference_folder = "/home/rjd/bru"
        case "шайбы":
            reference_folder = "/home/rjd/sha"
        case "гайка-барашек":
            reference_folder = "/home/rjd/bar"
        case "корзина":
            reference_folder = "/home/rjd/cor"
        case None:
            reference_folder = "/home/rjd/rzd"


    scale_factor = 0.6
    max_keypoints = 150
    frame_skip = 1
    frame_width = 640
    center_tolerance = 30  # Точность центрирования

    orb = cv2.ORB_create(nfeatures=max_keypoints)
    reference_descs = []

    for filename in os.listdir(reference_folder):
        img_path = os.path.join(reference_folder, filename)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            _, desc = orb.detectAndCompute(img, None)
            if desc is not None:
                reference_descs.append(desc)

    if not reference_descs:
        print("Нет валидных образцов!")
        return "Non object"

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    cap = cv2.VideoCapture("gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink")
    frame_counter = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_counter += 1
        if frame_counter % frame_skip != 0:
            continue

        small_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor)
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        kp_frame, desc_frame = orb.detectAndCompute(gray, None)

        if desc_frame is None or len(kp_frame) == 0:
            continue  # Если в кадре нет ключевых точек, продолжаем цикл

        best_matches, best_kp = [], []
        for ref_desc in reference_descs:
            try:
                matches = matcher.knnMatch(ref_desc, desc_frame, k=2)
                good = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good) > len(best_matches):
                    best_matches, best_kp = good, kp_frame
            except cv2.error:
                continue

        if not best_kp:
            continue  # Если совпадений не найдено, продолжаем цикл

        # Поиск ближайшего объекта
        object_centers = [kp.pt for kp in best_kp]
        frame_center_x = frame_width / 2
        closest_object = min(object_centers, key=lambda p: abs(p[0] - frame_center_x))

        object_x = closest_object[0]
        if object_x < frame_center_x - center_tolerance:
            send_to_arduino(-255, 255, -255, 255)  # Двигаемся вправо
        elif object_x > frame_center_x + center_tolerance:
            send_to_arduino(-255, 255, 255, -255)  # Двигаемся влево
        else:
            send_to_arduino(0, 0, 0, 0)  # Остановиться

        cv2.circle(frame, (int(object_x), int(closest_object[1])), 5, (0, 255, 0), -1)
        cv2.imshow("Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return "Non object"  # Вернется только, если программа завершится без объектов


# Позиционирование на детали в корзине
def track_servo():
    reference_folder = "/home/rjd/rzd"
    scale_factor = 0.6
    max_keypoints = 150
    frame_skip = 1
    frame_width = 640
    center_tolerance = 30  # Точность центрирования

    servo_id = 2
    servo_angle = 40  # Начальный угол сервопривода
    angle_step = 5  # Шаг изменения угла

    orb = cv2.ORB_create(nfeatures=max_keypoints)
    reference_descs = []

    for filename in os.listdir(reference_folder):
        img_path = os.path.join(reference_folder, filename)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            _, desc = orb.detectAndCompute(img, None)
            if desc is not None:
                reference_descs.append(desc)

    if not reference_descs:
        print("Нет валидных образцов!")
        return "Non object"

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    cap = cv2.VideoCapture("gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink")
    frame_counter = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_counter += 1
        if frame_counter % frame_skip != 0:
            continue

        small_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor)
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        kp_frame, desc_frame = orb.detectAndCompute(gray, None)

        if desc_frame is None or len(kp_frame) == 0:
            continue  # Если в кадре нет ключевых точек, продолжаем цикл

        best_matches, best_kp = [], []
        for ref_desc in reference_descs:
            try:
                matches = matcher.knnMatch(ref_desc, desc_frame, k=2)
                good = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good) > len(best_matches):
                    best_matches, best_kp = good, kp_frame
            except cv2.error:
                continue

        if not best_kp:
            continue  # Если совпадений не найдено, продолжаем цикл

        # Поиск ближайшего объекта
        object_centers = [kp.pt for kp in best_kp]
        frame_center_x = frame_width / 2
        closest_object = min(object_centers, key=lambda p: abs(p[0] - frame_center_x))

        object_x = closest_object[0]
        if object_x < frame_center_x - center_tolerance:
            servo_angle = max(0, servo_angle - angle_step)  # Двигаем сервопривод назад
        elif object_x > frame_center_x + center_tolerance:
            servo_angle = min(180, servo_angle + angle_step)  # Двигаем сервопривод вперед

        set_servo_angle(servo_id, servo_angle)  # Устанавливаем новый угол сервопривода

        cv2.circle(frame, (int(object_x), int(closest_object[1])), 5, (0, 255, 0), -1)
        cv2.imshow("Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return "Non object"  # Вернется только, если программа завершится без объектов


# Чтение qr-code
def read_qr_code():
    global qr_data

    cap = cv2.VideoCapture("gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        qr_codes = decode(frame)
        for qr in qr_codes:
            qr_data = qr.data.decode("utf-8")
            print(f"QR-код считан: {qr_data}")
            break

        cv2.imshow("QR Scanner", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


#  Получаем расстояние до детали
def get_distance():
    move("left", 2)
    distance = measure_distance(TRIG_ARM, ECHO_ARM)
    move("right", 2)
    return distance


# Основная функция
async def main():
    global order_matrix, case_out

    await process_order()
    print(order_matrix)
    await filter_order_matrix()

    go_start_position()
    GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
    sleep(8)
    GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)

    for i in [1, 2, 3]:
        await go_rack(i)

    go_finish_position()
    discharge(case_out)
    await put_order_done()


# Цикл приложения
if __name__ == "__main__":
    run = True
    try:
        while run:
            asyncio.run(main())
            time.sleep(1)  # Задержка для удобства
    except KeyboardInterrupt:
        run = False
