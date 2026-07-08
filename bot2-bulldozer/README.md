# Chariot Bot 2 – minimally refactored PlatformIO project

The original bulldozer `main.cpp` was separated into small source and header files without changing its robot behavior, names, constants, MQTT topics, movement order, delays, target handling, pushing behavior, or telemetry calculations.

## Structure

- `src/main.cpp` – Arduino `setup()` and `loop()`
- `src/Communication.cpp` – Wi-Fi and MQTT connection/subscription
- `src/DriveSystem.cpp` – motors, encoders, ToF setup, position and wheel correction
- `src/ObstacleHandler.cpp` – MQTT obstacle callback, target movement, pushing and return-home behavior
- `src/Telemetry.cpp` – Bot 2 telemetry publishing
- `src/RobotState.cpp` – shared state definitions
- `include/*.h` – declarations and configuration

## PlatformIO

The project contains only the existing `main` environment:

```powershell
pio run -e main
pio run -e main -t upload
pio device monitor -b 115200
```

The source filter compiles all refactored `.cpp` files. A file named `tx_power_calibration.cpp` is explicitly ignored if it is still present in the `src` folder.
