# 🤖 Otto Ninja — Mobile-Controlled Bipedal Robot

A dual-mode walking & rolling robot built on the **Wemos D1 Mini** (ESP8266), controlled wirelessly via the **RemoteXY** app on your smartphone. Otto Ninja can walk, roll, wave, spin in a circle, and even explore autonomously while avoiding obstacles.

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Required](#-hardware-required)
- [Wiring Diagram](#-wiring-diagram)
- [Software Setup](#-software-setup)
- [Installing the RemoteXY App](#-installing-the-remotexy-app)
- [Connecting Your Phone to Otto Ninja](#-connecting-your-phone-to-otto-ninja)
- [How to Control the Robot](#-how-to-control-the-robot)
- [Calibration](#-calibration)
- [Tuning & Customization](#-tuning--customization)
- [Troubleshooting](#-troubleshooting)

---

## ✨ Features

| Feature | How to Trigger |
|---|---|
| 🚶 Walk forward / backward | Joystick (Walk mode) |
| 🛞 Roll in any direction | Joystick (Roll mode) |
| 🤚 Wave | Button **A** |
| ⭕ Spin in a circle | Button **B** |
| 🔍 Autonomous obstacle-avoiding explore | Physical push button on D7 |
| 📱 Wireless control | RemoteXY app over Wi-Fi |

---

## 🛒 Hardware Required

| Component | Quantity | Notes |
|---|---|---|
| **Wemos D1 Mini** (ESP8266) | 1 | The main controller |
| SG90 / MG90 Servo Motor | 4 | 2 for legs, 2 for feet |
| HC-SR04 Ultrasonic Sensor | 1 | For obstacle detection |
| Push Button | 1 | Triggers autonomous explore mode |
| 5V Power Supply / Battery | 1 | Must handle 4 servos (≥ 2A recommended) |
| Jumper Wires | Several | |
| Otto Ninja 3D-printed frame | 1 | Standard Otto Ninja body |

---

## 🔌 Wiring Diagram

| Component | Signal Pin | D1 Mini Pin | GPIO |
|---|---|---|---|
| Left Foot Servo | Signal | **D1** | GPIO 5 |
| Left Leg Servo | Signal | **D2** | GPIO 4 |
| Right Foot Servo | Signal | **D3** | GPIO 0 |
| Right Leg Servo | Signal | **D4** | GPIO 2 |
| Push Button | Output | **D7** | GPIO 13 |
| HC-SR04 | TRIG | **D5** | GPIO 14 |
| HC-SR04 | ECHO | **D6** | GPIO 12 |

> ⚠️ **Power Tip:** Power the servos from a separate 5V supply (not from the D1 Mini's 3.3V pin). Share a common GND between the D1 Mini and the servo power supply.

---

## 💻 Software Setup

### Step 1 — Install Arduino IDE

Download and install the [Arduino IDE](https://www.arduino.cc/en/software).

### Step 2 — Add ESP8266 Board Support

1. Open Arduino IDE → **File → Preferences**
2. Paste this URL into *Additional Boards Manager URLs*:
   ```
   https://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
3. Go to **Tools → Board → Boards Manager**, search for `esp8266`, and install it.
4. Select **Tools → Board → Wemos D1 R2 & mini**

### Step 3 — Install Required Libraries

Go to **Sketch → Include Library → Manage Libraries** and install:

| Library | Search Term |
|---|---|
| RemoteXY | `RemoteXY` |
| ESP8266WiFi | Included with ESP8266 board package |
| Servo | `Servo` (built-in) |

### Step 4 — Upload the Code

1. Open `otto_ninja.ino` in Arduino IDE.
2. Connect your D1 Mini via USB.
3. Select the correct **Port** under **Tools → Port**.
4. Click **Upload** (→ arrow button).

---

## 📱 Installing the RemoteXY App

### Android

Download the APK directly:

**[⬇️ Download RemoteXY APK v4.15.15](https://apkpure.com/remotexy-arduino-control/com.shevauto.remotexy.free/download/4.15.15)**

> Since this is an APK file (not from the Play Store), you need to allow installation from unknown sources:
> - Go to **Settings → Security** (or **Apps → Special App Access**)
> - Enable **Install Unknown Apps** for your file manager or browser
> - Open the downloaded APK and tap Install

### iOS

Search for **"RemoteXY"** on the [App Store](https://apps.apple.com/) and install the free version.

---

## 📡 Connecting Your Phone to Otto Ninja

Otto Ninja creates its **own Wi-Fi hotspot**. You do not need a router.

1. **Power on** the robot (upload the code first).
2. On your phone, open **Wi-Fi Settings**.
3. Connect to the network:
   - **SSID (Network Name):** `OTTO NINJA2`
   - **Password:** `12345678`
4. Open the **RemoteXY app**.
5. Tap **+** → **Wi-Fi** → enter IP `192.168.4.1`, port `6377`.
6. The controller interface will load automatically.

> ✅ When connected, the joystick and buttons on the app will become active.

---

## 🎮 How to Control the Robot

### RemoteXY App Buttons

| Button | Action | Available In |
|---|---|---|
| **Y** | Switch to 🚶 **Walk mode** | Always |
| **X** | Switch to 🛞 **Roll mode** | Always |
| **A** | 🤚 **Wave** (stands on left leg, waves right leg twice) | Walk mode only |
| **B** | ⭕ **Circle** (stands on left leg, spins in a circle for 4 sec) | Walk mode only |

### Joystick

| Joystick Direction | Walk Mode | Roll Mode |
|---|---|---|
| Push **Up** | Walk forward | Roll forward |
| Push **Down** | Walk backward | Roll backward |
| Push **Left / Right** | Turn while walking | Roll sideways / steer |
| **Center (idle)** | Stand still | Stop rolling |

### Physical Push Button (D7)

| Press | Action |
|---|---|
| **First press** | 🔍 Start **Explore mode** — robot rolls forward and spins 180° when it detects an obstacle closer than 10 cm |
| **Second press** | ⛔ Stop Explore mode and return to standing position |

> 📌 While Explore mode is running, the joystick and app buttons are temporarily disabled.

---

## ⚙️ Calibration

If the robot doesn't stand upright, adjust these two values at the top of `otto_ninja.ino`:

```cpp
const int LA0 = 70;   // Left leg neutral angle  → increase to tilt left
const int RA0 = 110;  // Right leg neutral angle → decrease to tilt right
```

**How to calibrate:**
1. Upload the code and power on the robot.
2. Observe whether it leans left or right.
3. Adjust `LA0` and `RA0` by ±5 degrees at a time.
4. Re-upload and repeat until the robot stands straight.

All other leg and foot positions are calculated automatically from these two values.

---

## 🔧 Tuning & Customization

You can fine-tune the robot's behaviour by changing these `#define` values near the top of the code:

| Constant | Default | What It Does |
|---|---|---|
| `OBSTACLE_DISTANCE_CM` | `10` | Distance (cm) at which the robot turns away from obstacles |
| `EXPLORE_TURN_MS` | `550` | How long the robot spins for its 180° obstacle-avoidance turn |
| `ROLL_FWD_SPEED` | `40` | Speed of rolling forward (higher = faster) |
| `WAVE_REPEATS` | `2` | Number of times the robot waves |
| `CIRCLE_DURATION_MS` | `4000` | How long the circle spin lasts (in milliseconds) |
| `WAVE_LEG_LIFT` | `15` | How high the leg lifts during a wave |

---

## 🛠️ Troubleshooting

| Problem | Solution |
|---|---|
| Robot leans or falls over | Re-calibrate `LA0` / `RA0` (see Calibration section) |
| App doesn't connect | Make sure your phone is connected to the `OTTO NINJA2` Wi-Fi network, not your home Wi-Fi |
| Servos twitch but don't move properly | Check that servos are powered from a dedicated 5V supply, not from the D1 Mini |
| Explore mode doesn't turn | Increase `EXPLORE_TURN_MS` (e.g., try `700`) |
| Robot rolls too fast / slow | Adjust `ROLL_FWD_SPEED` |
| APK won't install | Enable "Install from Unknown Sources" in Android settings |
| Upload fails in Arduino IDE | Select correct COM port and make sure D1 Mini board is selected |

---

## 📁 Project Structure

```
otto_ninja/
├── otto_ninja.ino   # Main Arduino sketch
└── README.md        # This file
```

---

## 📜 License

This project is open-source and free to use, modify, and share for personal and educational purposes.

---

*Built with ❤️ using Wemos D1 Mini + RemoteXY + Otto Ninja frame.*
