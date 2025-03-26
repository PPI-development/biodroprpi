#!/usr/bin/env python3
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import time
import pigpio  # <-- pigpio вместо RPi.GPIO
import serial
from math import radians, sin, cos, sqrt, atan2

app = Flask(__name__)
CORS(app)

# ------------------- Константы и глобальные переменные -------------------
SERVO_PIN = 18        # BCM номер пина, к которому подключен серво
SERVO_OFF = 0         # pw = 0 полностью выключает servo-сигнал
SERVO_STOP = 1500     # ~1.5 мс - теоретическая остановка для непрерывного servo
SERVO_FORWARD = 2000  # ~2.0 мс - полная скорость вперёд
SERVO_BACKWARD = 1000 # ~1.0 мс - полная скорость назад (может не понадобиться)

# GPS-порт
serial_port = "/dev/serial0"

# Глобальные переменные состояния
mission_running = False
mode = "timer"
interval = 5.0            # Интервал в секундах (для режима таймера)
distance_threshold = 10.0 # Пороговое расстояние в метрах (для режима distance)
current_coords = None     # Текущие координаты
path = []                 # История маршрута
events = []               # Лог событий запусков серво

# Инициализация pigpio
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("Не удалось подключиться к pigpio. Убедитесь, что запущен демон pigpiod.")

# ------------------- Вспомогательные функции -------------------
def haversine(lat1, lon1, lat2, lon2):
    """Вычисление расстояния (м) между двумя координатами (градусы)."""
    R = 6371000
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def servo_forward():
    """Запускает движение сервопривода вперёд (полная скорость)."""
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_FORWARD)

def servo_stop():
    """Останавливает серво (для непрерывного - ставим ~1.5 мс)."""
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_STOP)

def servo_off():
    """Полностью отключает PWM-сигнал (серво перестаёт держать положение)."""
    pi.set_servo_pulsewidth(SERVO_PIN, SERVO_OFF)

def rotate_servo_single():
    """
    Один оборот непрерывного сервопривода:
    1) включаем полную скорость
    2) ждём нужное время (подбирай ~1 сек)
    3) останавливаем
    4) отключаем
    """
    servo_forward()
    time.sleep(1)  # Подбери под свой SG5010 (часто 0.9-1.2 сек)
    servo_stop()
    time.sleep(0.5)
    servo_off()

def continuous_rotation():
    """
    Включаем непрерывное вращение (при интервал=1).
    По факту — подаём сигнал на полную вперёд.
    """
    servo_forward()

# ------------------- GPS-функции -------------------
def get_gps_coordinates():
    """Считывает строку GPGGA и парсит координаты."""
    ser = serial.Serial(serial_port, baudrate=9600, timeout=1)
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore")
            if "$GPGGA" in line:
                parts = line.split(",")
                if len(parts) > 5 and parts[2] and parts[4]:
                    try:
                        lat = float(parts[2]) / 100.0
                        lon = float(parts[4]) / 100.0
                        return {"lat": lat, "lon": lon}
                    except ValueError:
                        pass
    except Exception as e:
        print(f"GPS Error: {e}")
    return None

def broadcast_location():
    """Постоянно обновляет current_coords и записывает в path."""
    global current_coords, path
    ser = serial.Serial(serial_port, baudrate=9600, timeout=1)
    while True:
        coords = get_gps_coordinates()
        if coords:
            current_coords = coords
            path.append(coords)
        else:
            print("GPS coordinates not determined")
        time.sleep(1)

# ------------------- Поток выполнения миссии -------------------
def mission_runner():
    """
    Запускается в отдельном потоке.
    Если mode=timer, через каждые 'interval' секунд крутим серво.
    Если mode=distance, крутим серво, когда пройдено distance_threshold метров.
    """
    global mission_running, current_coords, mode, interval, distance_threshold
    last_coords = current_coords.copy() if current_coords else None

    while mission_running:
        if mode == "timer":
            # Делаем оборот
            rotate_servo_single()
            # Логируем событие
            events.append({"event": "rotate", "coords": current_coords, "mode": mode, "time": time.time()})
            # Ждём заданный интервал
            t0 = time.time()
            while (time.time() - t0 < interval) and mission_running:
                time.sleep(0.1)

        elif mode == "distance":
            # Проверяем расстояние
            if current_coords and last_coords:
                dist = haversine(
                    last_coords["lat"], last_coords["lon"],
                    current_coords["lat"], current_coords["lon"]
                )
                if dist >= distance_threshold:
                    # Делаем оборот
                    rotate_servo_single()
                    events.append({"event": "rotate", "coords": current_coords, "mode": mode, "time": time.time()})
                    # Обновляем точку последнего сброса
                    last_coords = current_coords.copy()
            else:
                # Инициализируем, если нет координат
                if current_coords:
                    last_coords = current_coords.copy()
            # Небольшая задержка, чтобы не циклиться на 100%
            time.sleep(1)

    # Выход из цикла: обязательно остановить серво
    servo_off()

# ------------------- Flask API -------------------
@app.route("/start", methods=["POST"])
def start_mission():
    """
    Запуск миссии:
    - mode: 'timer' или 'distance'
    - interval: число (сек) для таймера
    - distance: число (метры) для режима distance
    Если interval=1 -> непрерывное вращение (continuous_rotation).
    """
    global mission_running, mode, interval, distance_threshold
    # Если что-то уже идёт, останавливаем
    if mission_running:
        mission_running = False
        time.sleep(0.5)
        servo_off()

    data = request.json
    mode = data.get("mode", "timer")
    interval = float(data.get("interval", 5))
    distance_threshold = float(data.get("distance", 10))

    # Если interval=1 и mode=timer, включаем непрерывку
    if mode == "timer" and interval == 1:
        # Запуск непрерывного вращения в отдельном потоке
        mission_running = True
        def run_continuous():
            continuous_rotation()
            while mission_running:
                time.sleep(0.1)
            servo_off()

        threading.Thread(target=run_continuous, daemon=True).start()

    else:
        # Запуск общего runner-а
        mission_running = True
        threading.Thread(target=mission_runner, daemon=True).start()

    return jsonify({
        "status": "Mission started",
        "mode": mode,
        "interval": interval,
        "distance": distance_threshold
    })

@app.route("/stop", methods=["POST"])
def stop_mission():
    """Принудительная остановка миссии и серво."""
    global mission_running
    mission_running = False
    servo_off()
    return jsonify({"status": "Mission stopped"})

@app.route("/location", methods=["GET"])
def get_location():
    """Текущая позиция, история пути и события."""
    return jsonify({
        "current_coords": current_coords,
        "path": path,
        "events": events
    })

@app.route("/shutdown", methods=["POST"])
def shutdown():
    """Завершение работы сервера и очистка ресурсов."""
    global mission_running
    mission_running = False
    servo_off()
    pi.stop()      # Отключаем pigpio
    return jsonify({"status": "Server shutdown"})

# ------------------- Главный блок -------------------
if __name__ == "__main__":
    # Запускаем GPS-поток
    threading.Thread(target=broadcast_location, daemon=True).start()
    # Запуск Flask
    app.run(host="0.0.0.0", port=5000)
