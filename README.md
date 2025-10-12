# Line Follower (EV3, Python, ev3dev2)

Easy and scalable set of scripts to drive LEGO Mindstorm EV3 robot booted from Ubuntu using Python + ev3dev2.
The repository includes multiple variants of increasing complexity, from drive left, right, or forward to full steering axis control with a PID-based turning angle and speed.

---

## 🚗 Robot Program Variants

### `LineFollowerPython.py`
Classic differential steering (two drive motors), “follow/turn” decisions based on color thresholds and simple speed/turn limits.  
Uses `MoveDifferential` and two color sensors on ports 1 and 2.  
Default parameters include:
- `speed = 40`
- `MIN_TURN = 2`
- `MAX_TURN = 60`
- black thresholds `[10, 30]`
- white thresholds `[80, 90]`

---

### `LineFollowerPythonColorReader.py`
Robot periodically logs brightness values (in `COL-REFLECT` mode), useful for calibrating black/white thresholds.  
Also based on `MoveDifferential`.

---

### `LineFollowerPythonWithPID.py`
Robot driving variant labeled “PID”, but still uses threshold logic. Serves as a base for implementing your own PID control for speed or turning.

---

### `LineFollowerFullSteering.py`
Configuration with a **steering axle** (drive motors on A/B - large motors, steering servo on C - medium motor).  
Implements **PID-based steering angle control** using a custom `simple_pid.PIDController`.  

Main idea:  
Compute the brightness difference between left and right sensors → use a PID controller to determine the steering servo’s target angle.  
The drive speed remains simple (e.g., 50% on straights, 30% in turns).

---

### `LineFollowerFullSteeringWithSpeed.py`
Extension of the above version that **attempts** to dynamically adjust speed depending on the steering angle.  
Marked as *TODO / work in progress* — the speed PID logic still needs refinement.

---

### `simple_pid.py`
Lightweight standalone PID controller with:
- **Anti-windup** (integral clamping)
- **Angle wrapping** (−π..π]
- **Derivative filtering**
- Optional “derivative on measurement”  
Used for steering control in the full-steering variants.

---

## 🧱 Hardware Requirements

- LEGO Mindstorms EV3 running ev3dev (Python 3)
- 2 × Large motors (drive) — **ports A & B**
- 1 × Medium motor (steering) — **port C** *(only for FullSteering versions)*
- 2 × Color sensors — **INPUT_1** (left), **INPUT_2** (right)

---

## ⚙️ Default Wiring

| Component          | Port       |
|--------------------|------------|
| Left drive motor   | OUTPUT_A   |
| Right drive motor  | OUTPUT_B   |
| Steering motor     | OUTPUT_C   |
| Left color sensor  | INPUT_1    |
| Right color sensor | INPUT_2    |

---

## 💾 Installation (on brick or Linux-based PC)

1. **Flash ev3dev** onto your EV3 brick.  
   - Download the latest image from [ev3dev.org](https://www.ev3dev.org/).  
   - Flash it to a microSD card and boot your EV3 from it.  
   - Connect to the brick via SSH or Visual Studio Code with the *ev3dev-browser* extension.

2. **Install Python and dependencies** (usually preinstalled):
   ```bash
   sudo apt update
   sudo apt install python3 python3-pip
   pip3 install --user python-ev3dev2

---

Structure
.
├── LineFollowerPython.py                  # Różnicowe sterowanie progowe (A/B + 2×Color)
├── LineFollowerPythonColorReader.py       # Logowanie jasności do kalibracji progów
├── LineFollowerPythonWithPID.py           # Nazwa „PID”, ale logika progowa
├── LineFollowerFullSteering.py            # Oś skrętna + PID kąta skrętu
├── LineFollowerFullSteeringWithSpeed.py   # Oś skrętna + (WIP) prędkość ~ skręt
├── simple_pid.py                          # Niezależny kontroler PID
└── README.md                              # (Ten plik – uzupełniona wersja)
