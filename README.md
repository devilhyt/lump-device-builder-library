# LUMP Device Builder Library

[![GitHub Repo stars](https://img.shields.io/github/stars/devilhyt/lump-device-builder-library?style=flat)](https://github.com/devilhyt/lump-device-builder-library/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/devilhyt/lump-device-builder-library?style=flat)](https://github.com/devilhyt/lump-device-builder-library/forks)
[![GitHub commit activity](https://img.shields.io/github/commit-activity/m/devilhyt/lump-device-builder-library?style=flat)](https://github.com/devilhyt/lump-device-builder-library/commits/main)
[![GitHub License](https://img.shields.io/github/license/devilhyt/lump-device-builder-library?style=flat)](./LICENSE)

This is an Arduino library that implements the LEGO UART Message Protocol (LUMP) for building custom devices.

## Introduction

Hi Makers! I'm HsiangYi from the [OFDL Robotics Lab](https://ofdl.tw/en/). We've open-sourced our internal library for building custom devices for LEGO hosts. For the open-source release, we've significantly refactored it and turned it into an Arduino library that's easy for both novices and professionals to use.

**Have fun building your own devices, and don't forget to share your creations with the community!**

## Todos

- [x] API Reference page
- [x] Advanced Topics page
- [x] Publish to Arduino Library Manager
- [x] Publish to PlatformIO Library Registry
- [ ] Troubleshooting page
- [ ] FAQ page

## Table of Contents

- [Features](#features)
- [Quickstart](#quickstart)
  - [Requirements](#requirements)
  - [Install the Library](#install-the-library)
    - [For Arduino IDE](#for-arduino-ide)
    - [For PlatformIO](#for-platformio)
    - [Manual Installation](#manual-installation)
  - [Wiring](#wiring)
    - [For SPIKE Hub](#for-spike-hub)
    - [For EV3](#for-ev3)
  - [Build a Simple Device — Analog / Digital Reader](#build-a-simple-device--analog--digital-reader)
    - [Step 1: Create a new Arduino sketch and include the library](#step-1-create-a-new-arduino-sketch-and-include-the-library)
    - [Step 2: Define the supported modes for the device](#step-2-define-the-supported-modes-for-the-device)
    - [Step 3: Instantiate the device](#step-3-instantiate-the-device)
    - [Step 4: Define the actions for each mode](#step-4-define-the-actions-for-each-mode)
    - [Step 5: Initialize the device and run it](#step-5-initialize-the-device-and-run-it)
    - [Step 6: Upload the sketch to the dev board](#step-6-upload-the-sketch-to-the-dev-board)
    - [Step 7: Test the device](#step-7-test-the-device)
- [Compatible Lego Hosts and Firmwares](#compatible-lego-hosts-and-firmwares)
- [Compatible Dev Boards](#compatible-dev-boards)
- [Limitations](#limitations)
- [Acknowledgements](#acknowledgements)
- [Disclaimers](#disclaimers)
- [License](#license)

## Features

- **LUMP Communication**
  - Communicates with LEGO hosts using the LEGO UART Message Protocol (LUMP).
    - See [Compatible Lego Hosts and Firmwares](#compatible-lego-hosts-and-firmwares).
  - Supports up to 16 modes for SPIKE Hub and 8 modes for EV3.
  - Configurable constant power on SPIKE Hub Pin 2, enabling external peripherals—such as servo motors or camera modules—to be powered at battery voltage.
    - See Advanced Topics - [Enable Constant Power on SPIKE Hub Pin 2](https://github.com/devilhyt/lump-device-builder-library/wiki/Advanced-Topics#enable-constant-power-on-spike-hub-pin-2).
  - Automatically detects host type for high-speed handshake, allowing SPIKE Hub to rapidly complete the handshake process.
- **Easy to Use**
  - Designed as an Arduino library, making it easy for both novices and professionals to use.
- **Non-Blocking Architecture**
  - Enabling programs to remain responsive while handling messages.
- **Easy Watchdog Timer Integration**
  - Just register the callback functions for watchdog timer initialization, feeding and deinitialization. The library handles the rest.
  - See Advanced Topics - [Watchdog Timer](https://github.com/devilhyt/lump-device-builder-library/wiki/Advanced-Topics#watchdog-timer).
- **Compatible with Multiple Dev Boards**
  - See [Compatible Dev Boards](#compatible-dev-boards).
- **Provides Basic Debugging Information**
  - Including device state tracking, decoded host messages, etc. 
  - See Advanced Topics - [Debug Mode](https://github.com/devilhyt/lump-device-builder-library/wiki/Advanced-Topics#debug-mode).

## Quickstart

### Requirements

To use the quickstart guide, you need:

- A LEGO host
  - See [Compatible LEGO Hosts and Firmwares](#compatible-lego-hosts-and-firmwares).
- A Dev board with UART interface.
  - See [Compatible Dev Boards](#compatible-dev-boards).
- An IDE
  - Arduino IDE 1.5.x+
  - PlatformIO
- Debugging tools
  - USB to TTL Serial Adapter (optional)
  - Logic Analyzer (optional, but highly recommended)

This guide uses a **SPIKE Hub with Pybricks** and an **ESP32-C3 SuperMini** as an example.

> [!TIP]
> Using **SPIKE Hub with Pybricks** is suggested for most users, as this combination is the simplest.
>
> Other combinations require additional steps. For example, using SPIKE Hub with SPIKE 3 requires emulating an existing LEGO sensor, while using the EV3 with EV3 Lab requires developing custom blocks.

> [!TIP]
> Choosing a dev board with at least two UART interfaces—one for programming and one for device communication—makes development easier.

### Install the Library

#### For Arduino IDE

1. In the menu bar, select `Tools` > `Manage Libraries…`.
2. Search for `LumpDeviceBuilder`.
3. Click install.

#### For PlatformIO

1. Go to `PlatformIO Home` > `Libraries` > `Registry`.
2. Search for `LumpDeviceBuilder`.
3. Click `Add to Project`.

#### Manual Installation

1. Download the [latest version](https://github.com/devilhyt/lump-device-builder-library/releases/latest) of the library from the GitHub Releases page and unzip it.
2. Move the library folder to the proper location.

### Wiring

> [!WARNING]
> Do not use multiple power sources simultaneously unless you are certain that the dev board has power management circuits.

#### For SPIKE Hub

- Development Wiring Diagram

  ```txt
  ┌──────────┐     ┌─────────────┐     ┌─────────────┐
  │  Burner  │     │  Dev Board  │     │  Host Port  │
  ├──────────┤     ├─────────────┤     ├─────────────┤
  │          │     │             │     │  1. M1      │      LPF2 Socket Pinout
  │          │     │             │     │  2. M2      │     ┌─────────────────┐
  │   GND    ├─────┤     GND     ├─────┤  3. GND     │     │   6 5 4 3 2 1   │
  │   VCC    ├─────┤     VIN     │     │  4. VCC     │     │ ┌─┴─┴─┴─┴─┴─┴─┐ │
  │          │     │     RX      ├─────┤  5. TX      │     └─┘             └─┘
  │          │     │     TX      ├─────┤  6. RX      │
  │   ...    ├─────┤     ...     │     └─────────────┘
  └──────────┘     └─────────────┘
  ```

- Production Wiring Diagram
  ```txt
  ┌─────────────┐     ┌─────────────┐
  │  Dev Board  │     │  Host Port  │
  ├─────────────┤     ├─────────────┤
  │             │     │  1. M1      │      LPF2 Socket Pinout
  │             │     │  2. M2      │     ┌─────────────────┐
  │     GND     ├─────┤  3. GND     │     │   6 5 4 3 2 1   │
  │     VIN     ├─────┤  4. VCC     │     │ ┌─┴─┴─┴─┴─┴─┴─┐ │
  │     RX      ├─────┤  5. TX      │     └─┘             └─┘
  │     TX      ├─────┤  6. RX      │
  └─────────────┘     └─────────────┘
  ```

#### For EV3

- Development Wiring Diagram

  ```txt
  ┌──────────┐     ┌─────────────┐     ┌─────────────┐
  │  Burner  │     │  Dev Board  │     │  Host Port  │      RJ12 Socket
  ├──────────┤     ├─────────────┤     ├─────────────┤      Pinout  ┌──┐
  │          │     │             │  ┌──┤  1. M1      │            ┌─┘  └─┐
  │          │     │             │  │  │  2. M2      │     ┌──────┘      │
  │   GND    ├─────┤     GND     ├──┴──┤  3. GND     │     │             │
  │   VCC    ├─────┤     VIN     │     │  4. VCC     │     │             │
  │          │     │     RX      ├─────┤  5. TX      │     │ 6 5 4 3 2 1 │
  │          │     │     TX      ├─────┤  6. RX      │     └─┴─┴─┴─┴─┴─┴─┘
  │   ...    ├─────┤     ...     │     └─────────────┘
  └──────────┘     └─────────────┘
  ```

- Production Wiring Diagram
  ```txt
  ┌─────────────┐     ┌─────────────┐
  │  Dev Board  │     │  Host Port  │      RJ12 Socket
  ├─────────────┤     ├─────────────┤      Pinout  ┌──┐
  │             │  ┌──┤  1. M1      │            ┌─┘  └─┐
  │             │  │  │  2. M2      │     ┌──────┘      │
  │     GND     ├──┴──┤  3. GND     │     │             │
  │     VIN     ├─────┤  4. VCC     │     │             │
  │     RX      ├─────┤  5. TX      │     │ 6 5 4 3 2 1 │
  │     TX      ├─────┤  6. RX      │     └─┴─┴─┴─┴─┴─┴─┘
  └─────────────┘     └─────────────┘
  ```

### Build a Simple Device — Analog / Digital Reader

#### Step 1: Create a new Arduino sketch and include the library

```cpp
#include <LumpDeviceBuilder.h>
```

#### Step 2: Define the supported modes for the device

For this device, two modes are implemented:

- mode 0: Analog Reader
- mode 1: Digital Reader

```cpp
const LumpMode modes[]{
  {"Analog", DATA16, 1, 4, 0, "raw", {0, 4095}, {0, 100}, {0, 4095}},
  {"Digital", DATA8, 1, 1, 0, "raw", {0, 1}, {0, 100}, {0, 1}}
};

uint8_t numModes = sizeof(modes) / sizeof(LumpMode);
```

> [!TIP]
> Declaring the mode table `const` lets the compiler place it in flash instead of RAM.
> On 32-bit MCUs each mode takes 56 bytes, so a 10-mode table saves 560 bytes of RAM.
> This matters on small parts such as STM32F0/C0 or CH32V003.
> A non-`const` table also works; it is simply kept in RAM.

Parameter descriptions:

```cpp
LumpMode(const char *name, uint8_t dataType, uint8_t numData,
         uint8_t figures, uint8_t decimals, const char *symbol,
         LumpValueSpan raw, LumpValueSpan pct, LumpValueSpan si,
         ...)
```

- `name`: Mode name.

  - Naming rules:

    - Must not be an empty string `""` or `nullptr`.
    - Must start with a letter (`A–Z`, `a–z`).

    If invalid, the name will be set to the string `"null"`.

  - Length limit (excluding null terminator):

    - If `power` parameter is `false`: `11` (default)
    - If `power` parameter is `true`: `5`

    Names exceeding the limit will be truncated.

  - The string is not copied; it is read during each handshake.
    It must remain valid for the lifetime of the device, e.g., a string literal or a global or `static` array.

- `dataType`: Data type.
  - Possible values: `DATA8`, `DATA16`, `DATA32`, `DATAF`.
- `numData`: Number of data.
  - The maximum depends on data type due to 32-byte payload limit:
    - `DATA8` (1 byte) : `[1..32]`
    - `DATA16` (2 bytes): `[1..16]`
    - `DATA32` (4 bytes): `[1..8]`
    - `DATAF` (4 bytes): `[1..8]`
- `figures`: Number of characters shown in view and data log (including the decimal point).
  - Valid range: `[0..15]`
- `decimals`: Number of decimals shown in view and data log.
  - Valid range: `[0..15]`
- `symbol`: Symbol of the measurement unit (default: `""`).
  - Rules:
    - Set to empty string `""` or `nullptr` if not required.
    - Length limit (excluding null terminator): `4`
    - Like `name`, the string is not copied and must remain valid for the lifetime of the device.
- `raw`: Raw value span (default: `false`).
  - When set to `false`, this value span will not be provided to the host. The host will use the default range `[0, 1023]`.
- `pct`: Percentage value span (default: `false`).
  - When set to `false`, this value span will not be provided to the host. The host will use the default range `[0, 100]`.
- `si`: Scaled value span (default: `false`).
  - When set to `false`, this value span will not be provided to the host. The host will use the default range `[0, 1023]`.

> [!NOTE]
> For a comprehensive list and detailed descriptions, please refer to the [`LumpMode`](https://github.com/devilhyt/lump-device-builder-library/wiki/API-Reference#lumpmode) API Reference.

#### Step 3: Instantiate the device

Select a serial interface for device communication and initialize the device with the defined modes.

```cpp
// Select a serial interface for device communication.
#define DEVICE_SERIAL Serial0
#define RX_PIN        20
#define TX_PIN        21

LumpDevice<HardwareSerial> device(&DEVICE_SERIAL, RX_PIN, TX_PIN, 68, 115200, modes, numModes);
```

Parameter descriptions:

```cpp
template <typename T>
LumpDevice(T *uart, uint8_t rxPin, uint8_t txPin,
           uint8_t type, uint32_t speed,
           const LumpMode *modes, uint8_t numModes,
           ...);
```

- `T`: Type of the serial interface (typically `hardwareSerial`).
- `uart`: Serial interface used for UART communication (e.g., `Serial0`, `Serial1`).
- `rxPin`: RX pin number of the serial interface.
- `txPin`: TX pin number of the serial interface.
- `type`: Device type.
- `speed`: Communication speed.
- `modes`: Device modes (must be an array of `LumpMode`).
- `numModes`: Number of modes.

  - The maximum depends on the host type:

    - SPIKE Hub: `[1..16]`
    - EV3: `[1..8]`

    Modes beyond the limit will be ignored.

> [!NOTE]
> For a comprehensive list and detailed descriptions, please refer to the [`LumpDevice`](https://github.com/devilhyt/lump-device-builder-library/wiki/API-Reference#lumpdevice) API Reference.

#### Step 4: Define the actions for each mode

The library uses a state machine to manage communication and reserves 2 states for developers to implement mode-specific actions.

Developers must implement the following states:

- `LumpDeviceState::InitMode`

  - Initializes the mode. 
    - e.g., resetting values, configuring hardware.

- `LumpDeviceState::Communicating`

  - Acquires data.
    - e.g., analog or digital reading.
  - Sends data to the host.
    - It is suggested to keep the data sending rate below 1000 Hz to prevent UART overrun errors.
    - For best practice, refer to [Event-Driven Data Transmission](https://github.com/devilhyt/lump-device-builder-library/wiki/Advanced-Topics#event-driven-data-transmission) in Advanced Topics.
  - Handles NACK from the host. 
    - Upon receiving a NACK, send data to the host immediately.

When the mode is changed, the device state transitions to `LumpDeviceState::InitMode`, and then to `LumpDeviceState::Communicating` after one iteration. These transitions are handled automatically by the library.

> [!WARNING]
> The entire program must be non-blocking when using this library. Avoid using blocking functions such as `delay()`. Use non-blocking techniques instead.

Let's design a function to run the device modes:

```cpp
// Pin definitions.
#define ANALOG_PIN  3
#define DIGITAL_PIN 4

void runDeviceModes() {
  // Get the device state and mode.
  auto state = device.state();
  auto mode  = device.mode();

  // Check the device state and perform actions accordingly.
  switch (state) {
    case LumpDeviceState::InitMode:
      // Initialize the mode.
      switch (mode) {
        case 0:
          pinMode(ANALOG_PIN, INPUT);
          break;
        case 1:
          pinMode(DIGITAL_PIN, INPUT);
          break;

        default:
          break;
      }
      break;

    case LumpDeviceState::Communicating: {
      static uint16_t value0     = 0;
      static uint8_t value1      = 0;
      static uint32_t period     = 5; // 200Hz
      static uint32_t prevMillis = 0;
      uint32_t currentMillis     = millis();

      // Send data to the host.
      if (device.hasNack() || (currentMillis - prevMillis > period)) {
        switch (mode) {
          case 0:
            value0 = analogRead(ANALOG_PIN);
            device.send(value0);
            break;

          case 1:
            value1 = digitalRead(DIGITAL_PIN);
            device.send(value1);
            break;

          default:
            break;
        }

        prevMillis = currentMillis;
      }
      break;
    }

    default:
      break;
  }
}
```

> [!NOTE]
> The value passed to `send()` must match the mode's `dataType` defined in `LumpMode`.
>
> For example, use `uint16_t` / `int16_t` for `DATA16`, and `uint8_t` / `int8_t` for `DATA8`.

#### Step 5: Initialize the device and run it

```cpp
void setup() {
  device.begin();
}

void loop() {
  device.run();
  runDeviceModes();
}
```

The full code is [here](https://github.com/devilhyt/lump-device-builder-library/blob/main/examples/AnalogDigitalReader/AnalogDigitalReader.ino).

#### Step 6: Upload the sketch to the dev board.

Ensure that the sketch is successfully uploaded to the dev board.

#### Step 7: Test the device.

Connect the dev board to SPIKE Hub Port A. Copy and paste the following code into the Pybricks IDE, then run it to verify device functionality:

```python
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port
from pybricks.tools import wait

device = PUPDevice(Port.A)
print(device.info())

while True:
    print(f"analog  (mode 0): {device.read(0)}")
    print(f"digital (mode 1): {device.read(1)}")
    wait(1000)
```

You should see the following output in the console:

```txt
{'id': 68, 'modes': (('Analog', 1, 1), ('Digital', 1, 0))}
analog  (mode 0): (4095,)
digital (mode 1): (1,)
```

**All steps are done!**

**You can start building your own devices.**

**See [Advanced Topics](https://github.com/devilhyt/lump-device-builder-library/wiki/Advanced-Topics) for in-depth guides.**

## Compatible LEGO Hosts and Firmwares

The firmware versions listed below were used for testing during the library's initial release.

<table>
  <thead>
    <tr>
      <th style="text-align: center;">Host</th>
      <th style="text-align: center;">Firmware</th>
      <th style="text-align: center;">Version</th>
    </tr>
  </thead>
  <tbody>
    <tr>
        <td rowspan="2">SPIKE Hub</td>
        <td>Pybricks</td>
        <td>3.6.1</td>
    </tr>
    <tr>
        <td>Hub OS</td>
        <td>1.6.62</td>
    </tr>
      <td>EV3</td>
      <td>EV3-G</td>
      <td>1.09E</td>
    </tr>
  </tbody>
</table>

## Compatible Dev Boards

The Dev boards listed below are ones we have used before and can run stably.

However, the library is designed to be compatible with any board that has a UART interface. If you have a board that is not listed here, please feel free to try it out.

<table>
  <thead>
    <tr>
      <th style="text-align: center;">Dev Board</th>
      <th style="text-align: center;">MCU</th>
      <th style="text-align: center;">Add'l Config</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>ESP32-C3 SuperMini</td>
      <td rowspan="3">ESP32</td>
      <td rowspan="3"/>
    </tr>
    <tr>
      <td>ESP32-DevKitC</td>
    </tr>
      <tr>
      <td>NodeMCU-32s</td>
    </tr>
    <tr>
      <td>NodeMCU DEVKIT V1.0</td>
      <td>ESP8266</td>
      <td/>
    </tr>
    <tr>
      <td>Raspberry Pi Pico</td>
      <td rowspan="3">RP2040</td>
      <td rowspan="3">
        <a href="https://github.com/devilhyt/lump-device-builder-library/wiki/Additional-Configuration-for-MCUs#rp2040">Yes</a>
      </td>
    </tr>
    <tr>
      <td>RP2040-Zero</td>
    </tr>
    <tr>
      <td>Seeed Studio XIAO-RP2040</td>
    </tr>
    <tr>
      <td>STM32 Blue Pill</td>
      <td>STM32F103C8T6</td>
      <td/>
    </tr>
    <tr>
      <td>Arduino UNO R3</td>
      <td rowspan="3">ATmega328/P</td>
      <td rowspan="3">
        <a href="https://github.com/devilhyt/lump-device-builder-library/wiki/Additional-Configuration-for-MCUs#atmega328/p">Yes</a>
      </td>
    </tr>
    <tr>
      <td>Arduino Nano</td>
    </tr>
    <tr>
      <td>Arduino Pro Mini</td>
    </tr>
        <tr>
      <td>MATRIX Mini R4</td>
      <td rowspan="2">Renesas RA4M1</td>
      <td rowspan="2">
        <a href="https://github.com/devilhyt/lump-device-builder-library/wiki/Additional-Configuration-for-MCUs#renesas-ra4m1">Yes</a>
      </td>
    </tr>
    <tr>
      <td>Arduino UNO R4</td>
    </tr>

  </tbody>
</table>

> [!IMPORTANT]
> Some Dev boards and MCUs require additional configuration.
>
> See [Additonal Configuration for MCUs](https://github.com/devilhyt/lump-device-builder-library/wiki/Additional-Configuration-for-MCUs) for details.

## Limitations

- The entire program must be non-blocking when using this library.
- No support for sending the [Combined Mode](https://github.com/pybricks/technical-info/blob/88a708c/uart-protocol.md#info_mode_combos) information during handshake.
- Not all MCUs support automatic host type detection. This feature must be manually disabled.

## Acknowledgements

Thanks to the Pybricks team for publishing a [detailed LUMP specification](https://github.com/pybricks/technical-info/blob/88a708c/uart-protocol.md). For our open-source release, we adopted the more comprehensive [LUMP header file](https://github.com/pybricks/pybricks-micropython/blob/7779f86/lib/lego/lego/lump.h) defined by Pybricks.

## Disclaimers

LEGO® is a trademark of the LEGO Group of companies which does not sponsor, authorize or endorse this project.

## License

This library is licensed under the LGPL 3.0 or later license.
