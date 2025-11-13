#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent  # noqa: E501
from ev3dev2.motor import MoveDifferential
from ev3dev2.wheel import EV3Tire


# ---------------- PID ----------------
class PIDController:
    def __init__(self, kp, ki, kd, dt,
                 integral_limit=None,
                 d_alpha=0.0,
                 derivative_on_measurement=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = integral_limit
        self.d_alpha = float(d_alpha)
        self.derivative_on_measurement = derivative_on_measurement
        self._previous_measured = None
        self._previous_derivative = 0.0

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def reset(self, integral=0.0, previous_error=0.0):
        self.integral = float(integral)
        self.previous_error = float(previous_error)
        self._previous_measured = None
        self._previous_derivative = 0.0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = self._clamp(
                self.integral, -self.integral_limit, self.integral_limit)
        if self.dt > 0:
            if self.derivative_on_measurement:
                if self._previous_measured is None:
                    raw_derivative = 0.0
                else:
                    dm = measured_value - self._previous_measured
                    raw_derivative = - dm / self.dt
                self._previous_measured = measured_value
            else:
                de = error - self.previous_error
                raw_derivative = de / self.dt
        else:
            raw_derivative = 0.0
        derivative = (1.0 - self.d_alpha) * raw_derivative + \
            self.d_alpha * self._previous_derivative
        self._previous_derivative = derivative
        output = (self.kp * error) + (self.ki * self.integral) + \
            (self.kd * derivative)
        self.previous_error = error
        return output


# ---------------- Inicjalizacja EV3 ----------------
left_color_sensor = ColorSensor(INPUT_1)
right_color_sensor = ColorSensor(INPUT_2)
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)
robot = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 114)
robot.stop()
try:
    gripper = MediumMotor(OUTPUT_C)
    GRIPPER_AVAILABLE = True
except Exception:
    gripper = None
    GRIPPER_AVAILABLE = False


# ---------------- Bezpieczny bufor stanu robota ----------------
class RobotState:
    """
    Ta klasa przechowuje najnowsze odczyty z sensorów.
    Używa threading.Lock, aby zapobiec jednoczesnemu zapisowi i odczytowi
    z różnych wątków (tzw. "race condition").
    """

    def __init__(self):
        self.lock = threading.Lock()
        # Inicjalizujemy domyślnymi wartościami
        self._L_intensity = 50
        self._R_intensity = 50
        self._left_name = 'black'  # Używamy 'black' zamiast None by uniknąć błędów
        self._right_name = 'black'
        self._running = True  # Flaga do zatrzymania wątku

    def update(self, L_intensity, R_intensity, left_name, right_name):
        """Wywoływane przez wątek sensorów do zapisu danych."""
        with self.lock:
            self._L_intensity = L_intensity
            self._R_intensity = R_intensity
            self._left_name = left_name
            self._right_name = right_name

    def get_readings(self):
        """Wywoływane przez główny wątek logiki do odczytu danych."""
        with self.lock:
            # Zwracamy kopię danych, aby były spójne
            return (self._L_intensity, self._R_intensity,
                    self._left_name, self._right_name)

    def is_running(self):
        """Sprawdza, czy wątek sensorów ma nadal działać."""
        with self.lock:
            return self._running

    def stop(self):
        """Ustawia flagę do zatrzymania wątku sensorów."""
        with self.lock:
            self._running = False


# ---------------- Wątek odczytujący sensory ----------------
class SensorPoller(threading.Thread):
    def __init__(self, robot_state, left_sensor, right_sensor):
        # daemon=True -> wątek zamknie się automatycznie, gdy główny program się zakończy
        super().__init__(daemon=True)
        self.robot_state = robot_state
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        print("SensorPoller: Inicjalizacja.")

    def run(self):
        """Główna pętla wątku sensorów. Działa w tle."""
        print("SensorPoller: Start pętli odczytu.")
        while self.robot_state.is_running():
            try:
                # 1. Odczyty światła (BLOKUJĄCE)
                L = self.left_sensor.reflected_light_intensity + 4
                R = self.right_sensor.reflected_light_intensity
                time.sleep(0.005)
                # 2. Odczyty kolorów (BLOKUJĄCE)
                left_name, right_name = self.read_both_colors_internal()

                # 3. Zapis do bufora (szybkie)
                self.robot_state.update(L, R, left_name, right_name)

            except Exception as e:
                print("BŁĄD w SensorPoller:{}".format(e))
                # Kontynuuj pętlę nawet po błędzie

            # Pauza, aby ten wątek nie zajął 100% CPU
            # i "oddał" czas procesora głównemu wątkowi logiki.
            time.sleep(0.005)  # Ok. 200 odczytów na sekundę

        print("SensorPoller: Zatrzymano.")

    # --- Funkcje pomocnicze sensora ---

    def normalized_color_name(self, sensor):
        """Zwraca jednolita nazwę koloru z sensora (np. 'red','green') lub None."""
        try:
            code = sensor.color
            mapping = {
                sensor.COLOR_GREEN: 'green',
                sensor.COLOR_RED: 'red',
            }
            return mapping.get(code, None)
        except Exception:
            try:
                name = sensor.color_name
                if name is None:
                    return None
                name = name.lower()
                for cname in ['red', 'green']:
                    if cname in name:
                        return cname
                return None
            except Exception:
                return None

    def read_both_colors_internal(self):
        """Wersja 'read_both_colors' dla wątku sensorów."""
        return (self.normalized_color_name(self.left_sensor),
                self.normalized_color_name(self.right_sensor))


# ---------------- Parametry ruchu i PID ----------------
BASE_SPEED = 20
DEADBAND = 5
MAX_CORRECTION = 100
SLEW_LIMIT = 200
leftcirclespeed = BASE_SPEED
ricghtcirclespeed = BASE_SPEED
pid = PIDController(kp=0.80, ki=0.0, kd=0.015, dt=0.01,
                    integral_limit=10, d_alpha=0.2)
try:
    left_color_sensor.calibrate_white()
    right_color_sensor.calibrate_white()
except Exception as e:
    print("Kalibracja bieli (można zignorować) ->", e)

# ---------------- Logika kolorów ----------------
COLORS_OF_INTEREST = ['blue', 'green', 'yellow']
COLOR_ERROR_GAINS = {
    'blue': 1.2, 'green': 1.15, 'yellow': 1.1, 'black': 1.0
}
CHECK_INTERVAL = 0.12
MIN_COLOR_CONFIRM = 1
COLOR_LOST_TIMEOUT = 99999
ROTATE_SPEED = 10
ROTATE_TIMEOUT = 99999
GRIPPER_SPEED = 40
GRIPPER_CLOSE_DEG = 360
GRIPPER_OPEN_DEG = -360
GRIPPER_CLOSE_SECONDS = 0.8
BACKUP_SPEED = 30
BACKUP_SECONDS = 1.0
ACTION_COOLDOWN = 1.0

# ---------------- Stan początkowy ----------------
current_target = 'black'
seen_colors = set(['black'])
last_check_time = 0.0
color_seen_time = None
color_confirm_count = 0
is_rotating = False
grip_state = 'open'
last_action_time = 0.0
current_state = 'FOLLOW_BLACK'

# ---------------- Funkcje pomocnicze (logiki) ----------------


def clamp_speed(pct):
    return max(0, min(100, int(round(pct))))


def limit_delta(delta, limit):
    if delta > limit:
        return limit
    if delta < -limit:
        return -limit
    return delta


def choose_detected_color(left_name, right_name, prefer_list):
    """
    Jeśli któryś z sensorów widzi kolor z prefer_list zwróć go według priorytetu prefer_list.
    W przeciwnym razie zwróć None.
    """
    for c in prefer_list:
        if left_name == c or right_name == c:
            return c
    return None


def apply_pid_speed_control(current_left_speed, current_right_speed, error_value, gain=1.0):
    """Wspólna logika PID + aktualizacji prędkości, zwraca (new_left, new_right, correction)."""
    scaled_error = error_value * gain
    correction = pid.compute(0.0, scaled_error)
    if abs(correction) < DEADBAND:
        correction = 0.0
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
    delta_left = limit_delta(correction,  SLEW_LIMIT)
    delta_right = limit_delta(-correction, SLEW_LIMIT)
    new_left = clamp_speed(current_left_speed + delta_left)
    new_right = clamp_speed(current_right_speed + delta_right)
    new_left = min(2*BASE_SPEED, new_left)
    new_right = min(2*BASE_SPEED, new_right)
    return new_left, new_right, correction


def rotate_until_both_see(robot_state, target_color, initial_side, timeout=ROTATE_TIMEOUT):
    """
    Obraca robota w stronę initial_side ('left' lub 'right') aż obydwa czujniki zobaczą target_color
    lub upłynie timeout. Zwraca True jeśli oba czujniki widzą kolor, False jeśli timeout.
    Odczytuje kolory z robot_state.get_readings() zamiast bezpośrednio z sensorów.
    """
    global is_rotating
    is_rotating = True
    start = time.time()

    if initial_side == 'left':
        left_speed = -ROTATE_SPEED
        right_speed = ROTATE_SPEED
    else:
        left_speed = ROTATE_SPEED
        right_speed = -ROTATE_SPEED

    try:
        robot.on(SpeedPercent(left_speed),
                 SpeedPercent(right_speed))
    except Exception:
        try:
            left_motor.on(SpeedPercent(left_speed))
            right_motor.on(SpeedPercent(right_speed))
        except Exception:
            pass

    success = False
    while True:
        now = time.time()

        # Odczyt z bufora stanu
        _, _, l, r = robot_state.get_readings()

        if (l == target_color and initial_side == 'right') or \
           (r == target_color and initial_side == 'left'):
            success = True
            break
        if now - start > timeout:
            success = False
            break
        sleep(0.01)  # Mała pauza w tej pętli jest nadal potrzebna

    robot.off()
    is_rotating = False
    return success

# --- Funkcje akcji ---


def close_gripper():
    """Zamknięcie chwytaka i ustawienie stanu."""
    global grip_state
    if not GRIPPER_AVAILABLE:
        print("Chwytak niedostępny... symuluję zamknięcie.")
        grip_state = 'closed'
        return
    try:
        gripper.on_for_degrees(SpeedPercent(
            GRIPPER_SPEED), GRIPPER_CLOSE_DEG, block=True)
        grip_state = 'closed'
        print("Chwytak: zamknięto.")
    except Exception as e:
        print("Błąd przy zamykaniu chwytaka:", e)
        grip_state = 'closed'


def open_gripper():
    """Otwarcie chwytaka i ustawienie stanu."""
    global grip_state
    if not GRIPPER_AVAILABLE:
        print("Chwytak niedostępny... symuluję otwarcie.")
        grip_state = 'open'
        return
    try:
        gripper.on_for_degrees(SpeedPercent(
            GRIPPER_SPEED), GRIPPER_OPEN_DEG, block=True)
        grip_state = 'open'
        print("Chwytak: otwarto.")
    except Exception as e:
        print("Błąd przy otwieraniu chwytaka:", e)
        grip_state = 'open'


def rotate_180_degrees(speed_pct=30, fallback_seconds=1.5):
    """Obrót o ~180 stopni."""
    try:
        print(">>> Rozpoczynam obrót o 180°.")
        robot.turn_degrees(180, speed_pct)
    except Exception as e:
        print("Error during rotation: {}".format(e))
        print(">>> Używam fallbacku: obrót w miejscu przez określony czas.")
        robot.on(SpeedPercent(-speed_pct), SpeedPercent(speed_pct))
        time.sleep(fallback_seconds)
        robot.off()


def backup_backward(speed_pct=BACKUP_SPEED, seconds=BACKUP_SECONDS):
    """Cofanie do tyłu przez określony czas (blokujące)."""
    print(">>> Cofam do tyłu przez {:.2f}s...".format(seconds))
    try:
        robot.on_for_seconds(SpeedPercent(-speed_pct),
                             SpeedPercent(-speed_pct), seconds, brake=True, block=True)
    except Exception:
        try:
            robot.on(SpeedPercent(-speed_pct),
                     SpeedPercent(-speed_pct))
            sleep(seconds)
        finally:
            robot.off()
    print(">>> Cofanie zakończone.")

# ---------------- Funkcje stanów ----------------


def state_follow_black(now, L, R, left_name, right_name):
    """
    Stan: śledzimy czarną linię i szukamy nowych kolorów.
    """
    global current_target, seen_colors, last_check_time, leftcirclespeed, ricghtcirclespeed, pid, color_seen_time

    # Periodyczne sprawdzanie kolorów
    if now - last_check_time >= CHECK_INTERVAL:
        last_check_time = now
        seen_color_now = choose_detected_color(
            left_name, right_name, COLORS_OF_INTEREST)
        if seen_color_now is not None and seen_color_now not in seen_colors:
            initial_side = None
            if left_name == seen_color_now:
                initial_side = 'left'
            elif right_name == seen_color_now:
                initial_side = 'right'
            if initial_side is not None:
                print(">>> Nowy kolor {} wykryty przez {}. Przechodzę do ROTATING.".format(
                    seen_color_now, initial_side))
                current_target = seen_color_now
                return 'ROTATING'

    # Standardowe sterowanie PID
    current_gain = COLOR_ERROR_GAINS.get('black', 1.0)
    leftcirclespeed, ricghtcirclespeed, correction = apply_pid_speed_control(
        leftcirclespeed, ricghtcirclespeed, (R - L), current_gain
    )
    if not is_rotating:
        robot.on(SpeedPercent(leftcirclespeed),
                 SpeedPercent(ricghtcirclespeed))

    # Sprawdzanie warunku do akcji
    if (not is_rotating) and (left_name is not None) and (left_name == right_name):
        if now - last_action_time >= ACTION_COOLDOWN:
            return 'ACTION'
    return 'FOLLOW_BLACK'


def state_rotating(robot_state, now, L, R, left_name, right_name):
    """
    Stan: obracamy aby wyrównać oba sensory na current_target.
    """
    global current_target, seen_colors, leftcirclespeed, ricghtcirclespeed, pid

    target = current_target
    initial_side = None
    if left_name == target and right_name != target:
        initial_side = 'left'
    elif right_name == target and left_name != target:
        initial_side = 'right'
    else:
        # Domyślny kierunek obrotu, jeśli stan jest niejasny
        initial_side = 'right'

    print(">>> ROTATING: próbuję wyrównać kolor '{}' od strony {}.".format(
        target, initial_side))

    # Przekazujemy 'robot_state' do funkcji obrotu
    success = rotate_until_both_see(
        robot_state, target, initial_side, timeout=ROTATE_TIMEOUT)

    if success:
        seen_colors.add(target)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        print(">>> ROTATING zakończone sukcesem. Przechodzę do FOLLOW_COLOR.")
        return 'FOLLOW_COLOR'
    else:
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        print(">>> ROTATING timeout — wracam do FOLLOW_BLACK.")
        return 'FOLLOW_BLACK'


def state_follow_color(now, L, R, left_name, right_name):
    """
    Stan: śledzimy kolor (inny niż czarny).
    """
    global current_target, color_seen_time, leftcirclespeed, ricghtcirclespeed, pid, last_action_time

    # Powrót do czarnego
    if left_name == 'black' or right_name == 'black':
        prev = current_target
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None
        print(">>> Wykryto CZARNY... Wracam do FOLLOW_BLACK.".format(prev))
        return 'FOLLOW_BLACK'

    # Odświeżanie czasu widzenia koloru
    if left_name == current_target or right_name == current_target:
        color_seen_time = now

    # Timeout zgubienia koloru
    if color_seen_time is not None and (now - color_seen_time) > COLOR_LOST_TIMEOUT:
        prev = current_target
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None
        print(">>> UTRACONO KOLOR '{}', wracam do FOLLOW_BLACK.".format(prev))
        return 'FOLLOW_BLACK'

    # Sterowanie PID dla koloru
    current_gain = COLOR_ERROR_GAINS.get(current_target, 1.0)
    leftcirclespeed, ricghtcirclespeed, correction = apply_pid_speed_control(
        leftcirclespeed, ricghtcirclespeed, (R - L), current_gain
    )
    if not is_rotating:
        robot.on(SpeedPercent(leftcirclespeed),
                 SpeedPercent(ricghtcirclespeed))

    # Warunek akcji dla koloru
    if (not is_rotating) and (left_name is not None) and (left_name == right_name) and (left_name == current_target):
        if now - last_action_time >= ACTION_COOLDOWN:
            return 'ACTION'
    return 'FOLLOW_COLOR'


def state_action(now, L, R, left_name, right_name):
    """
    Stan: wykonanie akcji chwytaka.
    """
    global grip_state, last_action_time, leftcirclespeed, ricghtcirclespeed, pid, color_seen_time

    if is_rotating:
        print(">>> ACTION: anulowane, bo trwa rotacja.")
        return 'FOLLOW_COLOR' if current_target != 'black' else 'FOLLOW_BLACK'

    # Zatrzymanie robota na czas akcji
    robot.off()
    print(">>> ACTION: oba sensory widzą '{}'. Stan chwytaka = '{}'".format(
        left_name, grip_state))

    if grip_state == 'open':
        print(">>> Zamykam chwytak.")
        close_gripper()
        sleep(0.2)
        print(">>> Obrót 180° po zamknięciu.")
        rotate_180_degrees(speed_pct=30, fallback_seconds=1.6)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = time.time()  # Reset czasu widzenia
    else:
        print(">>> Otwieram chwytak (rozluźniam).")
        open_gripper()
        sleep(0.15)
        print(">>> Cofanie po otwarciu.")
        backup_backward(speed_pct=BACKUP_SPEED, seconds=BACKUP_SECONDS)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None

    last_action_time = time.time()  # Reset cooldownu akcji

    if current_target == 'black':
        return 'FOLLOW_BLACK'
    else:
        return 'FOLLOW_COLOR'


# ---------------- Główna pętla (FSM) ----------------
last_time = time.time()

# Inicjalizujemy bufor i wątek sensorów
robot_state = RobotState()
sensor_thread = SensorPoller(
    robot_state, left_color_sensor, right_color_sensor)

try:
    print("Start. Uruchamiam wątek sensorów...")
    sensor_thread.start()

    # Dajemy wątkowi chwilę na pierwszy odczyt
    sleep(0.1)

    print("Start. Początkowy target =", current_target,
          "| seen:", seen_colors, "| grip_state:", grip_state)

    while True:
        now = time.time()
        dt = now - last_time
        if dt <= 0:
            dt = 0.001  # dt nie może być zerem ani ujemne
        last_time = now
        pid.dt = dt  # Aktualizuj dt dla PID w każdej pętli logiki

        # Odczyt z bufora stanu (natychmiastowy)
        L, R, left_name, right_name = robot_state.get_readings()

        error = R - L

        # Dispatch (przekierowanie) do odpowiedniego stanu
        if current_state == 'FOLLOW_BLACK':
            next_state = state_follow_black(now, L, R, left_name, right_name)

        elif current_state == 'ROTATING':
            # Przekazujemy 'robot_state' do funkcji stanu
            next_state = state_rotating(
                robot_state, now, L, R, left_name, right_name)

        elif current_state == 'FOLLOW_COLOR':
            next_state = state_follow_color(now, L, R, left_name, right_name)

        elif current_state == 'ACTION':
            next_state = state_action(now, L, R, left_name, right_name)

        else:
            print(">>> Nieznany stan, wracam do FOLLOW_BLACK.")
            next_state = 'FOLLOW_BLACK'

        # Przejście stanu
        if next_state != current_state:
            print(">>> STATE TRANSITION: {} -> {}".format(current_state, next_state))
        current_state = next_state

        # Logowanie
        current_gain = COLOR_ERROR_GAINS.get(current_target, 1.0)
        if int(now * 10) % 5 == 0:  # co ok. 0.5 s
            print("[T:{}] L_ref={}, R_ref={}, L_col={}, R_col={}, error={:.2f}, "
                  "speeds L={}, R={}, seen={}, grip={}, state={}, dt={:.4f}s".format(
                      current_target.upper(), L, R, left_name, right_name, error,
                      leftcirclespeed, ricghtcirclespeed, seen_colors, grip_state, current_state, dt
                  ))

        # Mała pauza w głównej pętli logiki
        # Zapobiega zajęciu 100% CPU i pozwala wątkowi sensorów działać.
        # Twoja pętla PID będzie teraz działać z częstotliwością ~100-200Hz
        sleep(0.005)

except KeyboardInterrupt:
    print("Zatrzymano (KeyboardInterrupt).")

finally:
    # Bezpieczne zakończenie programu
    print("Kończenie pracy...")

    # 1. Powiedz wątkowi sensorów, aby się zatrzymał
    robot_state.stop()

    # 2. Poczekaj, aż wątek sensorów faktycznie się zakończy
    sensor_thread.join()

    # 3. Wyłącz silniki
    robot.off()
    if GRIPPER_AVAILABLE:
        gripper.off()

    print("Program zakończony.")
