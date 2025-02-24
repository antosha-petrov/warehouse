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

qr_data = None
order_matrix = None
counter = 1
from_three = False
from_one = False
from_one_first = False
first_ride= True


GPIO.setmode(GPIO.BCM)
TRIG_LEFT = 23
ECHO_LEFT = 24
TRIG_RIGHT = 25
ECHO_RIGHT = 26
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)


SERVO_PINS = [17, 18, 27, 22]  # GPIO-пины для сервоприводов
GPIO.setup(SERVO_PINS, GPIO.OUT)

# Инициализация PWM для сервоприводов
SERVO_FREQ = 50  # Частота PWM для сервоприводов (50 Гц)
servos = [GPIO.PWM(pin, SERVO_FREQ) for pin in SERVO_PINS]
for servo in servos:
    servo.start(0)  # Запуск PWM с нулевым коэффициентом заполнен

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


async def update_item_location(goods_name, locations):
    """Обновление информации о местоположении товара."""
    if len(locations) != 3:
        print("Ошибка: массив должен содержать ровно 3 числа.")
        return

    async with aiohttp.ClientSession() as session:
        payload = {"GoodsName": goods_name, "Locations": locations}
        try:
            async with session.put(UPDATE_URL, json=payload) as response:
                match response.status:
                    case 200:
                        print("Обновление прошло успешно")
                        return await response.json()
                    case 400:
                        print("Ошибка: массив должен содержать ровно 3 числа.")
                    case 404:
                        print("Ошибка: заказов нет или товар не найден.")
                    case _:
                        print(f"Ошибка запроса: {response.status}")
        except Exception as e:
            print(f"Ошибка соединения: {e}")


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


def fill_order_matrix():
    """Заполняем матрицу заглушками"""
    global order_matrix
    if order_matrix is None or order_matrix.size == 0:
        order_matrix = np.array([])
        return

    # Генерация всех возможных локаций (3 ряда, 3 стеллажа, 2 полки)
    all_locations = set((r, c, s) for r in range(1, 4) for c in range(1, 4) for s in range(1, 3))

    # Извлекаем уже занятые локации
    used_locations = set((int(item[2]), int(item[3]), int(item[4])) for item in order_matrix)

    # Определяем незанятые локации
    empty_locations = all_locations - used_locations

    # Добавляем заглушки в пустые локации
    placeholders = [["Заглушка", 1, r, c, s] for r, c, s in empty_locations]

    if placeholders:
        order_matrix = np.vstack([order_matrix, placeholders]) if order_matrix.size > 0 else np.array(placeholders)

    print("Матрица с заглушками:", order_matrix)


async def process_rack(rack_number):
    """Обработка заказов в конкретном ряду."""
    global from_one, from_three, from_one_first

    rack_matrix = np.array(
        [item for item in (order_matrix if order_matrix is not None else []) if int(item[2]) == rack_number])
    cell_values = rack_matrix[:, 3].astype(int)

    if rack_matrix is not None:
        match rack_number:
            case 3:
                move_ultrasonic_sensor("right", "more", 20)
                move_ultrasonic_sensor("right", "less", 20)
                turn(90)
                from_three = True
            case 1:
                if from_three:
                    for i in [1, 3]:
                        move_rl("right", 0)  # Подобрать расстояние
                        i += 1
                    turn(-90)
                    move_ultrasonic_sensor("left", "more", 20)
                    move_ultrasonic_sensor("left", "less", 20)

                turn(-90)
                from_one = True
            case 2:
                if from_one:
                    turn(180)
                else:
                    turn(90)

        for cell in range(1, 4):
            if cell in cell_values:
                await process_cell(rack_matrix, cell, rack_number)


async def process_cell(rack_matrix, cell_number, rack_number):
    """Обработка заказов в конкретной ячейке."""
    global from_one, from_three, from_one_first

    cell_matrix = np.array([item for item in rack_matrix if int(item[3]) == cell_number])
    shelf_values = cell_matrix[:, 4].astype(int)

    if cell_matrix is not None:
        match rack_number:
            case 3:
                move_rl("left", 0)      # Подобрать значение
            case 1:
                move_rl("right", 0)      # Подобрать значение
                from_one_first = True
            case 2:
                if from_one_first:
                    pass
                else:
                    move_rl("right", 0)  # Подобрать значение

        for shelf in range(1, 3):
            if shelf in shelf_values:
                shelf_matrix = np.array([item for item in cell_matrix if int(item[4]) == shelf])
                num = shelf_matrix[0][1]
                await get_object(num, rack_number, cell_number, shelf)


def track_and_align_robot():
    """Функция отслеживания и центрирования объекта в кадре."""

    reference_folder = "/home/rjd/rzd"
    scale_factor = 0.6
    max_keypoints = 150
    frame_skip = 1
    use_bf_matcher = True
    frame_width = 640
    center_tolerance = 50

    orb = cv2.ORB_create(nfeatures=max_keypoints, scaleFactor=1.2, edgeThreshold=15, patchSize=20)
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
        return

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False) if use_bf_matcher else cv2.FlannBasedMatcher(
        dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1), dict(checks=15))

    cap = cv2.VideoCapture("libcamerasrc ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    frame_counter = 0
    processing_time = 0

    while cap.isOpened():
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        frame_counter += 1
        if frame_counter % frame_skip != 0:
            continue

        small_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor)
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        kp_frame, desc_frame = orb.detectAndCompute(gray, None)

        if desc_frame is None or len(desc_frame) < 10:
            continue

        best_matches, best_kp = [], []
        for ref_desc in reference_descs:
            try:
                matches = matcher.knnMatch(ref_desc, desc_frame, k=2)
                good = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good) > len(best_matches):
                    best_matches, best_kp = good, kp_frame
            except cv2.error:
                continue

        if best_kp and scale_factor != 1.0:
            best_kp = [
                cv2.KeyPoint(k.pt[0] / scale_factor, k.pt[1] / scale_factor, k.size / scale_factor, k.angle, k.response,
                             k.octave, k.class_id) for k in best_kp]

        if best_kp:
            object_x = np.mean([p.pt[0] for p in best_kp])
            frame_center = frame_width / 2
            if object_x < frame_center - center_tolerance:
                send_to_arduino(0, 0, 0, 0)
                # логика движения влево
                pass
            elif object_x > frame_center + center_tolerance:
                send_to_arduino(0, 0, 0, 0)
                # логика движения вправо
                pass

        processing_time = 0.9 * processing_time + 0.1 * (time.time() - start_time)

        if best_kp:
            for p in best_kp:
                cv2.circle(frame, (int(p.pt[0]), int(p.pt[1])), 3, (0, 255, 0), -1)

        cv2.putText(frame, f"Matches: {len(best_matches)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Optimized Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


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


def send_to_arduino(v1, v2, v3, v4):
    """Отправка команд на Arduino"""
    try:
        with serial.Serial('/dev/ttyACM0', 9600, timeout=1) as ser:
            ser.write(f"{v1},{v2},{v3},{v4}\n".encode())
            print(f"Команда: {v1},{v2},{v3},{v4}")
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


def move_ultrasonic_sensor_straight(side, expression, value):
    """Движение по ультразвуковым датчикам вперед"""
    if expression == "more":
        while measure_distance(TRIG_LEFT if side == "right" else TRIG_RIGHT, ECHO_LEFT if side == "right" else ECHO_RIGHT) > value:
            if side == "forward":
                send_to_arduino(255, 255, 255, 255)  # Вперед
                time.sleep(0.2)
            else:
                send_to_arduino(255, 255, 255, 255)  # Назад
                time.sleep(0.2)
    else:
        while measure_distance(TRIG_LEFT if side == "right" else TRIG_RIGHT, ECHO_LEFT if side == "right" else  ECHO_RIGHT) < value:
            if side == "forward":
                send_to_arduino(255, 255, 255, 255)  # Вперед
                time.sleep(0.2)
            else:
                send_to_arduino(255, 255, 255, 255)  # Назад
                time.sleep(0.2)


def move(side, length):
    """Движение по заданному расстоянию"""
    speed = 0  # Скорость вращения робота сантиметр/секунда
    start_time = time.time()
    while time.time() - start_time < length / speed:
        if side == "forward":
            send_to_arduino(0, 0, 0, 0)     # Вперед
        else:
            send_to_arduino(0, 0, 0, 0)     # Назад


def move_ultrasonic_sensor(side, expression, value):
    """Движение боком"""
    if expression == "more":
        while measure_distance(TRIG_LEFT if side == "right" else TRIG_RIGHT, ECHO_LEFT if side == "right" else ECHO_RIGHT) > value:
            if side == "right":
                send_to_arduino(255, 255, 255, 255)  # Вправо
                time.sleep(0.2)
            else:
                send_to_arduino(255, 255, 255, 255)  # Влево
                time.sleep(0.2)
    else:
        while measure_distance(TRIG_LEFT if side == "right" else TRIG_RIGHT, ECHO_LEFT if side == "right" else  ECHO_RIGHT) < value:
            if side == "right":
                send_to_arduino(255, 255, 255, 255)  # Вправо
                time.sleep(0.2)
            else:
                send_to_arduino(255, 255, 255, 255)  # Влево
                time.sleep(0.2)

    time.sleep(0.5)     # Погрешность
    send_to_arduino(0, 0, 0, 0)     # Остановка перед следующим этапом


def move_rl(side, length):
    """Движение по заданному расстоянию в стороны"""
    speed = 0  # Скорость вращения робота сантиметр/секунда
    start_time = time.time()
    while time.time() - start_time < length / speed:
        if side == "right":
            send_to_arduino(0, 0, 0, 0)     # Вправо
        else:
            send_to_arduino(0, 0, 0, 0)  # Влево

        time.sleep(0.2)


async def get_object(count, rack_number, cell_number, shelf):
    """Взятие объектов"""
    global order_matrix
    if first_ride:
        await update_item_location(read_qr_code(), [rack_number, cell_number, shelf])
        if get_item_name(rack_number, cell_number, shelf) != "Заглушка":
            arm(count)
    else:
        arm(count)


def arm(count):
    i = 0
    while i < count:
        track_and_align_robot()  # Центрирование по объекту
        set_servo_angle(0, 0)  # Взятие объекта
        i += 1


def get_item_name(row, rack, shelf):
    """Возвращает имя элемента из order_matrix по указанным координатам."""
    global order_matrix

    # Проверяем, что order_matrix не None и является массивом NumPy
    if order_matrix is not None and isinstance(order_matrix, np.ndarray):
        order_list = order_matrix.tolist()  # Преобразуем в список кортежей
    else:
        return ""  # Если order_matrix None или не является массивом NumPy, возвращаем пустую строку

    for item in order_list:
        if int(item[2]) == row and int(item[3]) == rack and int(item[4]) == shelf:
            return item[0]  # Имя товара
    return ""  # Возвращает пустую строку вместо None




def go_start_position():
    """Движение на стартовую позицию"""
    move_ultrasonic_sensor_straight("forward", "more", 5)
    move_ultrasonic_sensor("left", "less", 20)
    move("forward", 30)


def go_finish_position():
    """"Логика движения на выгрузку"""
    global from_one, from_three, from_one_first, first_ride

    from_three = False
    from_one = False
    from_one_first = False
    first_ride = False

    move_ultrasonic_sensor("left", "more", 20)
    move("back", 10)
    turn(180)
    move_ultrasonic_sensor_straight("forward", "more", 5)
    move_rl("left", 15)
    move("forward", 20)
    turn(90)
    move("forward", 5)

def turn(degrees):
    """Поворот на заданное количество градусов"""
    speed = 0 # Скорость вращения робота градус/секунда
    start_time = time.time()
    while time.time() - start_time < degrees/speed:  # Поворот на 180 градусов (время подбирается экспериментально)
        send_to_arduino(0, 100, 100, 0)  # Поворот направо
        time.sleep(0.2)

    # Остановка перед движением вперед
    send_to_arduino(0, 0, 0, 0)
    time.sleep(0.5)


async def main():
    global order_matrix
    await process_order()
    print(order_matrix)

    go_start_position()
    await filter_order_matrix()

    if counter != 0:
        fill_order_matrix()

    rack_values = order_matrix[:, 2].astype(int)

    await process_rack(3)
    for rack in [1, 2]:
        if rack in rack_values:
            await process_rack(rack)
            return

    move_rl("right", 0)  # Подобрать значение
    turn(-90)
    go_finish_position()


if __name__ == "__main__":
    run = True
    try:
        while run:
            asyncio.run(main())
            time.sleep(1)  # Задержка для удобства
    except KeyboardInterrupt:
        run = False