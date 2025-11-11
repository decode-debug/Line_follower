#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent
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
 
        # anti-windup
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, -self.integral_limit, self.integral_limit)
 
        # derivative
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
 
        derivative = (1.0 - self.d_alpha) * raw_derivative + self.d_alpha * self._previous_derivative
        self._previous_derivative = derivative
 
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output
 
# ---------------- Inicjalizacja EV3 ----------------
left_color_sensor  = ColorSensor(INPUT_1)
right_color_sensor = ColorSensor(INPUT_2)
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)
robot = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 114)
robot.stop()
 
# --- Chwytak (opcjonalny) ---
try:
    gripper = MediumMotor(OUTPUT_C)
    GRIPPER_AVAILABLE = True
except Exception:
    gripper = None
    GRIPPER_AVAILABLE = False
 
# ---------------- Parametry ruchu i PID ----------------
BASE_SPEED = 40          # % bazowa prędkość
DEADBAND = 5              # ignoruj drobne korekty
MAX_CORRECTION = 100      # maksymalna korekta
SLEW_LIMIT = 200          # ograniczenie delta prędkości/iterację
 
def clamp_speed(pct):
    return max(0, min(100, int(round(pct))))
 
def limit_delta(delta, limit):
    if delta > limit:
        return limit
    if delta < -limit:
        return -limit
    return delta
 
leftcirclespeed = BASE_SPEED
ricghtcirclespeed = BASE_SPEED
 
pid = PIDController(kp=0.80, ki=0.0, kd=0.015, dt=0.01, integral_limit=10, d_alpha=0.2)
 
# opcjonalna kalibracja bieli
try:
    left_color_sensor.calibrate_white()
    right_color_sensor.calibrate_white()
except Exception as e:
    print("Kalibracja bieli (można zignorować) ->", e)
 
# ---------------- Logika kolorów ----------------
COLORS_OF_INTEREST = ['blue', 'green', 'yellow']  # jakie kolory bierzemy pod uwagę
COLOR_ERROR_GAINS = {
    'blue': 1.2,
    'green': 1.15,
    'yellow': 1.1,
    'black': 1.0
}
 
# Debounce / timeouty
CHECK_INTERVAL = 0.12       # co ile sekund sprawdzać kolor
MIN_COLOR_CONFIRM = 1       # ile kolejnych wykryć by potwierdzić (w tej wersji nieużyteczne, zostawione)
COLOR_LOST_TIMEOUT = 99999    # timeout gdy nie widzimy danego koloru (s)
 
# Parametry obrotu (jeżeli wykryjemy kolor jednym sensorem -> obracamy aż drugi zobaczy)
ROTATE_SPEED = 10         # procentowy speed przy obracaniu (dla koła jadącego do przodu)
ROTATE_TIMEOUT = 99999        # maks czas obracania aby wyrównać drugi czujnik (s)
 
# Parametry chwytaka i cofania
GRIPPER_SPEED = 40          # procent dla silnika chwytaka
GRIPPER_CLOSE_DEG = 360     # stopnie obrotu (przykład) - zamknięcie
GRIPPER_OPEN_DEG = -360      # stopnie (negatywne) do otwarcia (dostosuj)
GRIPPER_CLOSE_SECONDS = 0.8 # alternatywnie: czas w sekundach przy on_for_seconds
BACKUP_SPEED = 30           # % prędkość cofania
BACKUP_SECONDS = 1.0        # ile sekund się cofa
 
# Cooldown by uniknąć powtórzeń akcji przy jednym detekcie
ACTION_COOLDOWN = 1.0       # s
 
# ---------------- Stan początkowy ----------------
current_target = 'black'     # zaczynamy od śledzenia CZARNEJ
seen_colors = set(['black']) # czarny już "widoczny"
last_check_time = 0.0
color_seen_time = None
color_confirm_count = 0
 
# flaga obracania (żeby nie aktywować chwytaka podczas rotacji)
is_rotating = False
 
# stan chwytaka: 'open' lub 'closed'
grip_state = 'open'  # startujemy z otwartym chwytakiem
last_action_time = 0.0
 
# maszyna stanów - możliwe stany:
# 'FOLLOW_BLACK', 'ROTATING', 'FOLLOW_COLOR', 'ACTION'
current_state = 'FOLLOW_BLACK'
 
# ---------------- Funkcje pomocnicze ----------------
def normalized_color_name(sensor):
    """Zwraca jednolita nazwę koloru z sensora (np. 'red','green') lub None."""
    try:
        code = sensor.color
        mapping = {
#            sensor.COLOR_BLACK: 'black',
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
            for cname in ['red','green']:
                if cname in name:
                    return cname
            return None
        except Exception:
            return None
 
def read_both_colors():
    """Zwraca (left_name, right_name) - mogą być None"""
    return normalized_color_name(left_color_sensor), normalized_color_name(right_color_sensor)
 
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
 
    delta_left  = limit_delta(correction,  SLEW_LIMIT)
    delta_right = limit_delta(-correction, SLEW_LIMIT)
 
    new_left  = clamp_speed(current_left_speed  + delta_left)
    new_right = clamp_speed(current_right_speed + delta_right)
 
    new_left  = min(2*BASE_SPEED, new_left)
    new_right = min(2*BASE_SPEED, new_right)
 
    return new_left, new_right, correction
 
def rotate_until_both_see(target_color, initial_side, timeout=ROTATE_TIMEOUT):
    """
    Obraca robota w stronę initial_side ('left' lub 'right') aż obydwa czujniki zobaczą target_color
    lub upłynie timeout. Zwraca True jeśli oba czujniki widzą kolor, False jeśli timeout.
    Ustawia/zeruje globalną flagę is_rotating.
    """
    global is_rotating
    is_rotating = True
    start = time.time()
 
    # kierunek obrotu: gdy left widzi -> obracamy w lewo (left wstecz, right do przodu)
    if initial_side == 'left':
        left_speed = -ROTATE_SPEED
        right_speed = ROTATE_SPEED
    else:
        left_speed = ROTATE_SPEED
        right_speed = -ROTATE_SPEED
 
    try:
        robot.on(SpeedPercent(left_speed), SpeedPercent(right_speed), brake=False)
    except Exception:
        try:
            left_motor.on(SpeedPercent(left_speed))
            right_motor.on(SpeedPercent(right_speed))
        except Exception:
            pass
 
    success = False
    while True:
        now = time.time()
        l, r = read_both_colors()
        if (l == target_color and initial_side == 'right') or (r == target_color and initial_side == 'left'):
            success = True
            break
        if now - start > timeout:
            success = False
            break
        sleep(0.01)
 
    robot.off()
    is_rotating = False
    return success
 
def close_gripper():
    """Zamknięcie chwytaka i ustawienie stanu."""
    global grip_state
    if not GRIPPER_AVAILABLE:
        print("Chwytak niedostępny (brak silnika na OUTPUT_C) — symuluję zamknięcie.")
        grip_state = 'closed'
        return
    try:
        gripper.on_for_degrees(SpeedPercent(GRIPPER_SPEED), GRIPPER_CLOSE_DEG, block=True)
        grip_state = 'closed'
        print("Chwytak: zamknięto (gripper motor).")
    except Exception as e:
        print("Błąd przy zamykaniu chwytaka:", e)
        grip_state = 'closed'
 
def open_gripper():
    """Otwarcie chwytaka i ustawienie stanu."""
    global grip_state
    if not GRIPPER_AVAILABLE:
        print("Chwytak niedostępny (brak silnika na OUTPUT_C) — symuluję otwarcie.")
        grip_state = 'open'
        return
    try:
        # używamy negatywnych stopni do otwarcia (dostosuj GRIPPER_OPEN_DEG jeśli potrzeba)
        gripper.on_for_degrees(SpeedPercent(GRIPPER_SPEED), GRIPPER_OPEN_DEG, block=True)
        grip_state = 'open'
        print("Chwytak: otwarto (gripper motor).")
    except Exception as e:
        print("Błąd przy otwieraniu chwytaka:", e)
        grip_state = 'open'
 
def rotate_180_degrees(speed_pct=30, fallback_seconds=1.5):
    """
    Obrót o ~180 stopni. Najpierw próbujemy MoveDifferential.turn_degrees,
    jeśli nie jest dostępne używamy fallbacku: obrót w miejscu przez określony czas.
    """
    try:
        print(">>> Rozpoczynam obrót o 180°.")
        robot.turn_degrees(180, speed_pct)  # Use speed_pct directly, not wrapped in SpeedPercent
    except Exception as e:
        print("Error during rotation: {}".format(e))
        print(">>> Używam fallbacku: obrót w miejscu przez określony czas.")
        robot.on(SpeedPercent(-speed_pct), SpeedPercent(speed_pct))  # Fallback: rotate in place
        time.sleep(fallback_seconds)  # Rotate for the fallback duration
        robot.off()  # Stop the motors after the fallback rotation
 
def backup_backward(speed_pct=BACKUP_SPEED, seconds=BACKUP_SECONDS):
    """Cofanie do tyłu przez określony czas (blokujące)."""
    print(">>> Cofam do tyłu przez {:.2f}s z prędkością {}%.".format(seconds, speed_pct))
    try:
        robot.on_for_seconds(SpeedPercent(-speed_pct), SpeedPercent(-speed_pct), seconds, brake=True, block=True)
    except Exception:
        # fallback: bez blokowania
        try:
            robot.on(SpeedPercent(-speed_pct), SpeedPercent(-speed_pct), brake=True)
            sleep(seconds)
        finally:
            robot.off()
    print(">>> Cofanie zakończone.")
 
# ---------------- Funkcje stanów ----------------
 
def state_follow_black(now, L, R, left_name, right_name):
    """
    Stan: śledzimy czarną. Szukamy nowych kolorów z listy
    Jeśli wykryjemy nowy kolor z preferencji -> ustawiamy rotate_target i przechodzimy do ROTATING.
    W tym stanie także wykonujemy normalne PID-owe sterowanie kół.
    """
    global current_target, seen_colors, last_check_time, leftcirclespeed, ricghtcirclespeed, pid, color_seen_time
 
    # Periodyczne sprawdzanie kolorów (debounce)
    if now - last_check_time >= CHECK_INTERVAL:
        last_check_time = now
        seen_color_now = choose_detected_color(left_name, right_name, COLORS_OF_INTEREST)
        if seen_color_now is not None and seen_color_now not in seen_colors:
            # określ stronę początkową
            initial_side = None
            if left_name == seen_color_now:
                initial_side = 'left'
            elif right_name == seen_color_now:
                initial_side = 'right'
 
            if initial_side is not None:
                print(">>> Nowy kolor {} wykryty przez {}. Przechodzę do ROTATING.".format(seen_color_now, initial_side))
 
                # ustaw target tymczasowo (przechodzi do ROTATING)
                current_target = seen_color_now
                return 'ROTATING'
 
    # standardowe PID sterowanie podczas śledzenia czarnej
    current_gain = COLOR_ERROR_GAINS.get('black', 1.0)
    leftcirclespeed, ricghtcirclespeed, correction = apply_pid_speed_control(
        leftcirclespeed, ricghtcirclespeed, (R - L), current_gain
    )
 
    if not is_rotating:
        robot.on(SpeedPercent(leftcirclespeed), SpeedPercent(ricghtcirclespeed), brake=False)
 
    # sprawdzanie: jeśli oba czujniki widzą ten sam kolor (dowolny), to możliwa akcja
    if (not is_rotating) and (left_name is not None) and (left_name == right_name):
        if now - last_action_time >= ACTION_COOLDOWN:
            return 'ACTION'
 
    return 'FOLLOW_BLACK'
 
def state_rotating(now, L, R, left_name, right_name):
    """
    Stan: obracamy aby wyrównać oba sensory na current_target.
    Jeżeli uda się wyrównać -> FOLLOW_COLOR, w przeciwnym razie -> FOLLOW_BLACK.
    """
    global current_target, seen_colors, leftcirclespeed, ricghtcirclespeed, pid
 
    target = current_target  # zapamiętaj target (może zostać zmieniony w międzyczasie)
    # ustal początkową stronę (jeżeli mamy - jeśli nie -> próbuj obrócić domyślnie w prawo)
    initial_side = None
    if left_name == target and right_name != target:
        initial_side = 'left'
    elif right_name == target and left_name != target:
        initial_side = 'right'
    else:
        # jeżeli oba lub żaden — spróbuj domyślnie 'right'
        initial_side = 'right'
 
    print(">>> ROTATING: próbuję wyrównać kolor '{}' od strony {}.".format(target, initial_side))
    success = rotate_until_both_see(target, initial_side, timeout=ROTATE_TIMEOUT)
    if success:
        # udało się — dodaj do seen, ustaw kolor jako aktywny i przejdź do FOLLOW_COLOR
        seen_colors.add(target)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        # odnotuj czas widzenia
        # ustaw color_seen_time by nie natychmiast uznać go za utracone
        # (funkcja wywołująca przekaże aktualny 'now')
        print(">>> ROTATING zakończone sukcesem — oba sensory widzą '{}'. Przechodzę do FOLLOW_COLOR.".format(target))
        return 'FOLLOW_COLOR'
    else:
        # timeout — wracamy do śledzenia czarnej
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        print(">>> ROTATING timeout — wracam do FOLLOW_BLACK.")
        return 'FOLLOW_BLACK'
 
def state_follow_color(now, L, R, left_name, right_name):
    """
    Stan: śledzimy current_target (np. 'blue').
    Jeśli zobaczymy 'black' -> wracamy do FOLLOW_BLACK.
    Jeśli oba sensory widzą ten sam kolor i cooldown -> ACTION.
    Jeśli stracimy kolor dłużej niż timeout -> FOLLOW_BLACK.
    W tym stanie także sterujemy PIDem (z innym gainem).
    """
    global current_target, color_seen_time, leftcirclespeed, ricghtcirclespeed, pid, last_action_time
 
    # jeśli widzimy czarny -> natychmiast powrót
    if left_name == 'black' or right_name == 'black':
        prev = current_target
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None
        print(">>> Wykryto CZARNY podczas śledzenia '{}'. Wracam do FOLLOW_BLACK.".format(prev))
        return 'FOLLOW_BLACK'
 
    # odświeżamy czas widzenia jeśli widzimy aktualny target
    if left_name == current_target or right_name == current_target:
        color_seen_time = now
 
    # jeśli straciliśmy kolor zbyt długo -> wracamy do czarnego
    if color_seen_time is not None and (now - color_seen_time) > COLOR_LOST_TIMEOUT:
        prev = current_target
        current_target = 'black'
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None
        print(">>> UTRACONO KOLOR '{}', wracam do FOLLOW_BLACK.".format(prev))
        return 'FOLLOW_BLACK'
 
    # PID ze specjalnym gainem (dla aktualnego koloru)
    current_gain = COLOR_ERROR_GAINS.get(current_target, 1.0)
    leftcirclespeed, ricghtcirclespeed, correction = apply_pid_speed_control(
        leftcirclespeed, ricghtcirclespeed, (R - L), current_gain
    )
 
    if not is_rotating:
        robot.on(SpeedPercent(leftcirclespeed), SpeedPercent(ricghtcirclespeed))
 
    # jeśli oba sensory widzą ten sam kolor (ten który śledzimy) -> akcja
    if (not is_rotating) and (left_name is not None) and (left_name == right_name) and (left_name == current_target):
        if now - last_action_time >= ACTION_COOLDOWN:
            return 'ACTION'
 
    return 'FOLLOW_COLOR'
 
def state_action(now, L, R, left_name, right_name):
    """
    Stan: wykonanie akcji chwytaka w zależności od stanu chwytaka.
    Po akcji przechodzimy z powrotem do FOLLOW_COLOR (lub FOLLOW_BLACK jeśli występuje).
    """
    global grip_state, last_action_time, leftcirclespeed, ricghtcirclespeed, pid, color_seen_time
 
    # zabezpieczenie: jeśli w trakcie rotacji -> anuluj akcję
    if is_rotating:
        print(">>> ACTION: anulowane, bo trwa rotacja.")
        return 'FOLLOW_COLOR' if current_target != 'black' else 'FOLLOW_BLACK'
 
    # zatrzymaj robota przed akcją
    robot.off()
    print(">>> ACTION: oba sensory widzą '{}'. Stan chwytaka = '{}'".format(left_name, grip_state))
 
    if grip_state == 'open':
        # zamknij chwytak i obróć
        print(">>> Zamykam chwytak.")
        close_gripper()
        sleep(0.2)
        print(">>> Obrót 180° po zamknięciu.")
        rotate_180_degrees(speed_pct=30, fallback_seconds=1.6)
        # reset i kontynuuj (wracamy do śledzenia aktualnego targetu)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        # odnotuj że widzieliśmy kolor
        color_seen_time = time.time()
    else:
        # otwórz chwytak i cofnij się
        print(">>> Otwieram chwytak (rozluźniam).")
        open_gripper()
        sleep(0.15)
        print(">>> Cofanie po otwarciu.")
        backup_backward(speed_pct=BACKUP_SPEED, seconds=BACKUP_SECONDS)
        pid.reset()
        leftcirclespeed = BASE_SPEED
        ricghtcirclespeed = BASE_SPEED
        color_seen_time = None
 
    last_action_time = time.time()
 
    # po akcji zwykle wracamy do FOLLOW_COLOR lub FOLLOW_BLACK (jeśli aktualny target to 'black')
    if current_target == 'black':
        return 'FOLLOW_BLACK'
    else:
        return 'FOLLOW_COLOR'
 
# ---------------- Główna pętla (FSM) ----------------
last_time = time.time()
try:
    print("Start. Początkowy target =", current_target, "| seen:", seen_colors, "| grip_state:", grip_state)
    while True:
        now = time.time()
        dt = now - last_time
        if dt <= 0:
            dt = 0.001
        last_time = now
        pid.dt = dt
 
 
        # Odczyty światła (dla PID) - wykorzystujemy różnicę natężenia
        L = left_color_sensor.reflected_light_intensity + 4
        R = right_color_sensor.reflected_light_intensity
        error = R - L
 
        # Odczyt kolorów (często, bo potrzebny do wykryć)
        left_name, right_name = read_both_colors()
 
        # Zanim przejdziemy do stanu wykonajmy uniwersalne sprawdzenie:
        # - Jeżeli jesteśmy w FOLLOW_BLACK i wykryto qualche kolor -> ROTATING (zrobione w funkcji stanu)
        # - Jeżeli oba czujniki widzą ten sam kolor i cooldown -> ACTION (obsługiwane w funkcjach stanów)
 
        # Dispatch do stanu
        if current_state == 'FOLLOW_BLACK':
            next_state = state_follow_black(now, L, R, left_name, right_name)
        elif current_state == 'ROTATING':
            next_state = state_rotating(now, L, R, left_name, right_name)
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
 
        # Logujemy status w mniejszej częstotliwości (tu co pętlę — możesz zmniejszyć)
        current_gain = COLOR_ERROR_GAINS.get(current_target, 1.0)
        if int(now * 10) % 5 == 0:  # co ok. 0.5 s
            print("[T:{}] L_ref={}, R_ref={}, L_col={}, R_col={}, error={:.2f}, gain={:.2f}, "
                "speeds L={}, R={}, seen={}, grip={}, state={}, dt={:.4f}s".format(
                    current_target.upper(), L, R, left_name, right_name, error, current_gain,
                    leftcirclespeed, ricghtcirclespeed, seen_colors, grip_state, current_state, dt
            ))
except KeyboardInterrupt:
    robot.off()
    print("Zatrzymano (KeyboardInterrupt).")
 