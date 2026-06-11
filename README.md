# ME-507 Bicycle Power Meter

A custom bicycle crank power meter built with **nRF Connect SDK** and **Zephyr RTOS**. The project measures crank bending strain and motion using custom hardware, estimates crank state with an extended Kalman filter, computes bicycle power, and broadcasts the result over Bluetooth Low Energy using a Cycling Power Service style interface.

This repository contains the firmware, custom Zephyr board/device-tree support, sensor and BLE libraries, and hardware design files for a custom PCB-based bicycle power meter.

## Project Overview

This project is intended to run on Nordic nRF52 hardware using the nRF Connect SDK. It combines:

- Strain/load measurement through an external ADC
- IMU-based crank motion sensing
- Sensor fusion and state estimation using an embedded Kalman filter
- Power calculation from measured torque and angular velocity
- BLE transmission to compatible cycling/fitness devices
- Custom PCB hardware files for the physical power meter electronics

The firmware is organized as a multi-threaded Zephyr application, with separate source files for IMU sampling, ADC sampling, filtering, battery monitoring, BLE communication, and power measurement.

## Repository Structure

```text
.
├── Hardware/                  # Mechanical/PCB-related design outputs
├── boards/                    # Zephyr board overlays for the target hardware
├── config/                    # Additional project configuration files
├── dts/bindings/sensor/       # Custom Zephyr device-tree bindings
├── lib/                       # Project libraries and drivers
├── src/                       # Main application source code
├── CMakeLists.txt             # Zephyr application build file
├── prj.conf                   # Main Zephyr project configuration
├── debug_nus.conf             # Optional BLE NUS debug configuration (WIP)
└── west.yml                   # West manifest, including external module references
```

## Firmware Architecture

The application is built around Zephyr RTOS threads and inter-thread communication. The main functional blocks are:

- `imu_thread`: samples the IMU and provides crank motion data
- `adc_thread`: samples the strain/load measurement front end
- `filter_thread`: runs sensor fusion and filtering logic
- `battery_thread`: monitors battery/fuel-gauge state
- `ble_thread`: advertises and sends measurements over BLE
- `power_meas`: computes torque, angular velocity, cadence, and power-related quantities
- `app_ipc`: shared application messaging and synchronization helpers

The firmware is configured for RTT logging and BLE peripheral operation. Floating-point support is enabled for the filtering and power calculation code.

## Libraries

Several project-specific libraries live in `/lib`.

### `lib/ble`

Bluetooth Low Energy support for the power meter. This project is intended to expose measured cycling power data through a custom implementation based on the BLE Cycling Power Service style of operation.

An optional Nordic UART Service configuration is also present for debug builds, allowing debug or telemetry output over BLE NUS when enabled.

### `lib/eekf`

Embedded Extended Kalman Filter support used for IMU/state estimation. The filter helps fuse noisy sensor measurements into a more useful estimate of crank state for downstream power calculations.

### `lib/icm40609`

Driver/support code for the ICM-40609-D IMU used for crank motion sensing. The IMU provides accelerometer and gyroscope data used by the filtering and power calculation pipeline.

### `lib/icm20948`

Legacy support for the ICM-20948 IMU, which was used in earlier hardware revisions.

### `lib/max17048`

Driver/support code for the MAX17048 battery fuel gauge.

### `lib/ads1220`

ADS1220 ADC support for high-resolution analog measurement. The ADS1220 support in this project is based on the work from [`badjeff/ads1220-zephyr-module`](https://github.com/badjeff/ads1220-zephyr-module), which is also referenced in the project `west.yml`.

## Hardware

The `Hardware/` directory contains custom hardware design files for the physical bicycle power meter. These files are intended to support a custom PCB and mechanical integration into a crank-arm-mounted power meter.

Current hardware-related files include:

- Gerber CAM/manufacturing outputs
- Fusion 360 export
- STEP model output

## Bluetooth Output

The project is designed to send calculated cycling power data over BLE so that standard cycling computers, fitness watches, or BLE-capable host devices can receive live power data.

The primary BLE path is based on a Cycling Power Service style interface. A debug configuration for Nordic UART Service is also included for development and telemetry use (WIP).

## Sensor Fusion and Power Calculation

The power meter estimates crank motion and loading from multiple sensors:

1. The ADC measures strain/load-related signals from the crank hardware.
2. The IMU measures acceleration and angular velocity.
3. The Kalman filter estimates useful crank state from noisy sensor inputs.
4. The application computes power from torque and angular velocity.
5. BLE packets transmit the calculated result.

The Kalman filter implementation is an important part of the project because it provides a structured way to combine IMU measurements with the mechanical model of the bicycle crank.

## Building

This is an **nRF Connect SDK / Zephyr RTOS** application.

Typical workflow:

```bash
west init -l .
west update
west build -b nrf52dk_nrf52832 .
west flash
```

Depending on the target hardware, board name, overlay files, and build configuration may need to be adjusted.

For nRF Connect for VS Code, open the repository as an existing nRF Connect application and create a build configuration for the desired board.

## Configuration Notes

The main firmware configuration is in:

```text
prj.conf
```

This enables core project features such as:

- GPIO
- I2C
- SPI
- ADC
- Zephyr input subsystem support
- RTT logging
- floating-point printf support
- FPU support
- BLE peripheral mode
- custom BLE device name

The optional NUS debug configuration is in:

```text
debug_nus.conf
```

## Development Status

This project is under active development as a college course bicycle power meter. Some modules are experimental, some are hardware-specific, and some library code may represent earlier hardware revisions or alternate sensor options.

## Credits

- ADS1220 Zephyr support is based on [`badjeff/ads1220-zephyr-module`](https://github.com/badjeff/ads1220-zephyr-module).
- Built with Nordic nRF Connect SDK and Zephyr RTOS.
- Developed as part of the Cal Poly ME-507 bicycle power meter project.
