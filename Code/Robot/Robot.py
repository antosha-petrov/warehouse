import asyncio
import RPi.GPIO as GPIO
import aiohttp
import numpy as np
import cv2
import os
import time
import serial

URL = "http://192.168.135.124:5298/orders/get/robot"
UPDATE_URL = "http://192.168.135.124:5298/orders/update/loc"

case_out = 0 # 1/2/3 Контейнер для выгрузки

qr_data = None
order_matrix = None
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

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

# Настроим Raspberry Pi
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

SERVO_PINS = [17, 18, 27, 22]  # GPIO-пины для сервоприводов
GPIO.setup(SERVO_PINS, GPIO.OUT)

# Инициализация PWM для сервоприводов
SERVO_FREQ = 50  # Частота PWM для сервоприводов (50 Гц)
servos = [GPIO.PWM(pin, SERVO_FREQ) for pin in SERVO_PINS]
for servo in servos:
    servo.start(0)  # Запуск PWM с нулевым коэффициентом заполнен


# Получение данных о заказах
async def fetch_order(session):
    """Получение данных о заказах."""
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
    """Обработка данных о заказах."""
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


# Завершение заказа
async def put_order_done():
    url = "http://192.168.135.124:5298/orders/put/done"
    async with aiohttp.ClientSession() as session:
        async with session.put(url) as response:
            return await response.text()


# Фильтрация матрицы
async def filter_order_matrix():
    """Фильтрация матрицы заказов по количеству товаров."""
    global order_matrix
    if order_matrix is not None and order_matrix.size > 0:
        order_matrix = np.array([
            [item[0], item[1], item[2], item[3], item[4]] for item in order_matrix if int(item[1]) > 0
        ])
        print("Отфильтрованный order_matrix:", order_matrix)
    else:
        print("order_matrix пуст или None, фильтрация не выполнена.")


# Работа с физическими элементами
def send_to_arduino(v1, v2, v3, v4):
    global ser
    """Отправка команд на Arduino"""
    try:
        with ser:
            # Формируем строку данных
            data = f"{v1},{v2},{v3},{v4}\n"
            serv.write(data.encode())
            print(f"Sent: {data.strip()}")
    except Exception as e:
        print(f"Ошибка:{e}")
def set_servo_angle(servo_id, angle):
    """Установка угла для сервопривода"""
    if 0 <= servo_id < len(servos) and 0 <= angle <= 180:
        duty_cycle = (angle / 18) + 2  # Преобразование угла в коэффициент заполнения
        servos[servo_id].ChangeDutyCycle(duty_cycle)
        time.sleep(0.1)  # Даем сервоприводу время на перемещение
    else:
        print(f"Ошибка: Недопустимый servo_id или angle")
def measure_distance(trig, echo):
    """Измерение расстояния с медианным фильтром"""
    distances = []
    for _ in range(3):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        pulse_start = time.time()
        while GPIO.input(echo) == 0:
            pulse_start = time.time()

        pulse_end = time.time()
        while GPIO.input(echo) == 1:
            pulse_end = time.time()

        distances.append((pulse_end - pulse_start) * 17150)

    return sorted(distances)[1]


# Поворот
def turn(degrees):
    """Поворот на заданное количество градусов"""
    speed = 89  # Градус/секунда

    if degrees > 0:
        start_time = time.time()
        while time.time() - start_time < degrees / speed:  # Поворот на 180 градусов (время подбирается экспериментально)
            send_to_arduino(255, 255, -255, -255)  # Поворот направо
    else:
        start_time = time.time()
        while time.time() - start_time < degrees / speed:  # Поворот на 180 градусов (время подбирается экспериментально)
            send_to_arduino(-255, -255, 255, 255)  # Поворот направо

    send_to_arduino(0, 0, 0, 0)


# Движение
def move(side, length):
    """Движение по заданному расстоянию"""
    speed = 60  # Сантиметр/секунда
    start_time = time.time()
    while time.time() - start_time < length / speed:
        match side:
            case "forward":
                send_to_arduino(255, 255, 255, 255)
            case "back":
                send_to_arduino(-255, -255, -255, -255)
            case "left":
                send_to_arduino(-255, 255, 255, -255)
            case "right":
                send_to_arduino(255, -255, -255, 255)

    send_to_arduino(0, 0, 0, 0)


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

    send_to_arduino(0, 0, 0, 0)  # Остановка перед следующим этапом


# Движение на старт
def go_start_position():
    """Движение на стартовую позицию"""
    move_ultrasonic_sensor("forward", "more", 20)
    move_ultrasonic_sensor("left", "less", 20)
    move_ultrasonic_sensor("forward", "more", 20)
    move_ultrasonic_sensor("left", "less", 20)


# Движение на финиш
def go_finish_position():
    """"Логика движения на выгрузку"""
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
    """Движение к ряду"""
    match rack_number:
        case 1:
            turn(-90)
            move("right", 60)
            move_ultrasonic_sensor("forward", "more", 30)
            move_ultrasonic_sensor("back", "less", 30)
            for i in [1, 2, 3]:
                cell()
                if i != 3:
                    move("right", 30)
        case 2:
            turn(180)
            move_ultrasonic_sensor("forward", "more", 30)
            move_ultrasonic_sensor("back", "less", 30)
            for i in [1, 2, 3]:
                cell()
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
                cell()
                if i != 3:
                    move("left", 30)


# Обработка заказов в конкретной ячейке
async def cell():
    """Обработка заказов в конкретной ячейке."""
    for i in [1, 2]:
        if i == 1:
            set_servo_angle(0, 150)
            set_servo_angle(1, 60)
            set_servo_angle(2, 90)
            set_servo_angle(3, 40)
        else:
            set_servo_angle(0, 80)
            set_servo_angle(1, 70)
            set_servo_angle(2, 165)
            set_servo_angle(3, 80)

        read_qr_code()
        if str(qr_data).lower in order_matrix[:, 0]:
            index = np.where(order_matrix[:, 0] == qr_data)[0][0]
            count = order_matrix[index, 1]
            get_object(count, i, index, str(qr_data).lower)


# Взятие объекта
async def get_object(count, shelf, index, name):
    global order_matrix

    while count > 0:
        if track_lr() == "Non object":
            break

        set_servo_angle(3, 80)
        if shelf == 1:
            if track_lr("корзина") == "Non object":
                track_lr(name)
                theta1, theta2 = calculate_extension(150, 60, get_distance())
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
            else:
                theta1, theta2 = calculate_extension(150, 60, 5)
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
                set_servo_angle(2, 30)
                track_lr(name)
                theta1, theta2 = calculate_extension(150, 60, get_distance())
                set_servo_angle(0, theta1)
                set_servo_angle(1, theta2)
        else:
            track_lr(name)
            theta1, theta2 = calculate_extension(80, 70, get_distance())
            set_servo_angle(0, theta1)
            set_servo_angle(1, theta2)

        time.sleep(1)
        set_servo_angle(3, 0)
        put_bag(shelf)

        if order_matrix[index, 1] > 1:
            order_matrix[index, 1] -= 1
        else:
            order_matrix = np.delete(order_matrix, index, axis=0)
        count -= 1


# Подсчет удлинения
def calculate_extension(theta1, theta2, L):
    """
    Вычисляет новые углы theta1' и theta2' для удлинения вперед на L, сохраняя высоту.

    theta1, theta2 - текущие углы (в градусах)
    L - требуемое удлинение
    """
    # Длины плеч фиксированы
    l1 = 23  # горизонтальное плечо
    l2 = 17  # вертикальное плечо

    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)

    # Численный метод решения
    result = np.linalg.solve(np.array([
        [-l1 * np.sin(theta1), -l2 * np.sin(theta2)],
        [l1 * np.cos(theta1), l2 * np.cos(theta2)]
    ]), np.array([-L, 0]))

    theta1_new, theta2_new = theta1 + result[0], theta2 + result[1]

    return np.degrees(theta1_new), np.degrees(theta2_new)


# Выгрузка
def discharge(box):
    """Логика выгрузки товаров из корзины"""
    match box:
        case 1:
            move("left", 13)
        case 2:
            pass
        case 3:
            move("right", 13)

    GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
    sleep(8)
    GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)
    set_servo_angle(0, 140)
    set_servo_angle(1, 30)
    set_servo_angle(3, 80)
    GPIO.output(LOWER_LOW_MOTOR, GPIO.HIGH)
    sleep(8)
    GPIO.output(LOWER_LOW_MOTOR, GPIO.LOW)


    while track_servo() != "Non object":
        theta1, theta2 = calculate_extension(140, 30, get_distance())

        set_servo_angle(0, theta1)
        set_servo_angle(1, theta2)
        sleep(0.5)
        set_servo_angle(3, 0)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)

        set_servo_angle(0, 80)
        set_servo_angle(1, 70)
        set_servo_angle(2, 165)

        move("forward", 25)
        set_servo_angle(3, 80)
        sleep(0.5)
        move("back", 25)


        set_servo_angle(0, 140)
        set_servo_angle(1, 30)
        set_servo_angle(3, 80)
        GPIO.output(LOWER_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(LOWER_LOW_MOTOR, GPIO.LOW)


# Кладем деталь в корзину
def put_bag(shelf):
    if shelf == 2:
        set_servo_angle(0, 140)
        set_servo_angle(1, 180)
        set_servo_angle(3, 90)
        move_ultrasonic_sensor("back", "more", 10)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)
        set_servo_angle(1, 30)
        set_servo_angle(3, 140)
        set_servo_angle(1, 60)
        move_ultrasonic_sensor("forward", "less", 30)
        move_ultrasonic_sensor("forward", "more", 30)
    else:
        move_ultrasonic_sensor("back", "more", 10)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.HIGH)
        sleep(8)
        GPIO.output(HIGH_LOW_MOTOR, GPIO.LOW)
        set_servo_angle(0, 140)
        set_servo_angle(1, 30)
        set_servo_angle(3, 140)
        set_servo_angle(1, 60)
        move_ultrasonic_sensor("forward", "less", 30)
        move_ultrasonic_sensor("forward", "more", 30)


# Позиционирование на детали
def track_lr(name=None):
    """Отслеживание и центрирование ближайшего объекта в кадре."""
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
    cap = cv2.VideoCapture(0)
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


# Поиск деталей в корзине
def track_servo():
    """Отслеживание объекта и центрирование с помощью сервопривода."""
    reference_folder = "/home/rjd/rzd"
    scale_factor = 0.6
    max_keypoints = 150
    frame_skip = 1
    frame_width = 640
    center_tolerance = 30  # Точность центрирования

    servo_id = 2
    servo_angle = 90  # Начальный угол сервопривода
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
    cap = cv2.VideoCapture(0)
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
    """Функция для считывания QR-кода и сохранения данных в переменную"""
    global qr_data

    cap = cv2.VideoCapture(0)  # Здесь 0 — индекс основной камеры
    detector = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        data, bbox, _ = detector.detectAndDecode(frame)
        if data:
            qr_data = data
            print(f"QR-код считан: {qr_data}")

        time.sleep(0.1)  # Небольшая задержка для снижения нагрузки на CPU

    cap.release()


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

    for i in [1, 2, 3]:
        go_rack(i)

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
