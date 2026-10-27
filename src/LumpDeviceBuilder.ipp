// SPDX-FileCopyrightText: 2023-2025 OFDL Robotics Lab
// SPDX-FileCopyrightText: 2023-2025 HsiangYi Tsai <devilhyt@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef LUMP_DEVICE_BUILDER_IPP
#define LUMP_DEVICE_BUILDER_IPP

#include "LumpDeviceBuilder.h"

namespace LumpDeviceBuilder {

  template <typename T>
  LumpDevice<T>::LumpDevice(
      T *uart,
      uint8_t rxPin,
      uint8_t txPin,
      uint8_t type,
      uint32_t speed,
      const LumpMode *modes,
      uint8_t numModes,
      uint8_t view,
      uint32_t fwVersion,
      uint32_t hwVersion
  )
      : uart{uart},
        rxPin{rxPin},
        txPin{txPin},
        type{type},
        speed{speed},
        modes{modes},
        view{view},
        fwVersion{fwVersion},
        hwVersion{hwVersion} {
    this->numModes = min(numModes, static_cast<uint8_t>(LUMP_MAX_EXT_MODE + 1));
  }

  template <typename T>
  void LumpDevice<T>::begin() {
    LUMP_DEBUG_BEGIN(LUMP_DEBUG_SPEED);

    /**
     * Allocates one zeroed buffer shared by all writable modes (mapOut != NONE).
     * Done here rather than in the constructor so it never depends on global initialization order.
     */
    uint16_t size = 0;
    for (uint8_t i = 0; i < numModes; ++i) {
      if (modes[i].isWritable()) {
        size += align4(modes[i].dataMsgSize);
      }
    }
    if (size != dataMsgBufSize || (size && !dataMsgBuf)) {
      free(dataMsgBuf);
      dataMsgBuf     = size ? static_cast<uint8_t *>(calloc(size, 1)) : nullptr;
      dataMsgBufSize = dataMsgBuf ? size : 0;
    }
    dataMsgFlags = 0;

    deviceState     = LumpDeviceState::InitWdt;
    prevDeviceState = LumpDeviceState::InitWdt;
    receiverState   = LumpReceiverState::ReadByte;
  }

  template <typename T>
  void LumpDevice<T>::end() {
    LUMP_DEBUG_PRINTLN("[Info] Device ended");

    if (deinitWdtCallback) {
      LUMP_DEBUG_PRINTLN("[WDT] WDT disabled");
      deinitWdtCallback();
    }

    LUMP_DEBUG_END();
  }

  template <typename T>
  void LumpDevice<T>::run() {
    currentMillis = millis();
    _run();
    processRxMsg();
  }

  template <typename T>
  void LumpDevice<T>::_run() {
    using namespace LumpDeviceBuilder::Internal;

    /* Device state machine */
    switch (deviceState) {
      /* Initialization phase */
      case LumpDeviceState::InitWdt:
        /* Initializes the watchdog timer. */
        LUMP_DEBUG_PRINTLN("[State] Init WDT");

        if (initWdtCallback) {
          LUMP_DEBUG_PRINTLN("[WDT] WDT enabled");
          deinitWdtCallback();
          initWdtCallback();
        }

        deviceState = LumpDeviceState::Reset;
        break;

      case LumpDeviceState::Reset:
        /* Resets the device. */
        LUMP_DEBUG_PRINTLN("------------------------");
        LUMP_DEBUG_PRINTLN("[State] Reset");

        feedWdt();
        deviceMode = 0;
        extMode    = 0;
        _hasNack   = false;
        isLpf2Host = false;
        clearCmdWriteData();
        dataMsgFlags = 0;
        if (dataMsgBuf) {
          memset(dataMsgBuf, 0, dataMsgBufSize);
        }

        deviceState = LumpDeviceState::InitAutoId;

        LUMP_DEBUG_PRINTLN("[Info] Starting handshake...");
        break;

      /* Handshake phase */
      case LumpDeviceState::InitAutoId:
        /**
         * Initializes the AutoID.
         *
         * Configures the UART to perform AutoID:
         * - Grounds the TX pin to indicate UART mode to the host.
         * - Enables RX to receive information from the host for automatic host type detection.
         *   Receipt of the `LUMP_CMD_SPEED` command indicates that the host type is LPF2.
         *
         * Note:
         *   To achieve this, the library follows a simple sequence:
         *   1. Initializes the UART.
         *   2. Grounds the TX pin.
         *
         *   However, this sequence may not work on all MCUs due to hardware differences.
         *   For example, on the ATmega328/P, register manipulation is required to ground the TX pin after UART initialization.
         *
         *   Therefore, on such MCUs, the automatic host type detection feature must be manually disabled by defining the
         *   `LUMP_HOST_DETECT_OFF` macro before including the library header to ensure proper operation.
         */
        LUMP_DEBUG_PRINTLN("[State] Init AutoID");

#ifdef LUMP_HOST_DETECT_OFF
        uart->end();
#else
        initUart(LUMP_UART_SPEED_LPF2);
#endif

        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);

        prevMillis  = currentMillis;
        deviceState = LumpDeviceState::WaitingAutoId;

        LUMP_DEBUG_PRINTLN("[State] Waiting for AutoID");
        break;

      case LumpDeviceState::WaitingAutoId:
        /**
         * Waits for the AutoID.
         *
         * - For LPF2 hosts: Waits until the `LUMP_CMD_SPEED` command is received,
         *                   then transition to `LumpDeviceState::InitUart`.
         *                   See `processRxMsg()` for details.
         * - For EV3 hosts: Waits for `LUMP_AUTO_ID_DELAY` milliseconds before proceeding.
         */
        if (currentMillis - prevMillis > LUMP_AUTO_ID_DELAY) {
          deviceState = LumpDeviceState::InitUart;
        }
        break;

      case LumpDeviceState::InitUart:
        /**
         * Initializes the UART.
         *
         * The UART speed depends on host type:
         * - For LPF2 hosts: `LUMP_UART_SPEED_LPF2`.
         * - For EV3 hosts: `LUMP_UART_SPEED_MIN`.
         */
        feedWdt();

        LUMP_DEBUG_PRINTLN("[Info] AutoID complete");
        LUMP_DEBUG_PRINT("[Info] Host type: ");
        LUMP_DEBUG_PRINTLN(isLpf2Host ? "LPF2" : "Non LPF2");
        LUMP_DEBUG_PRINT("[Info] Speed: ");
        LUMP_DEBUG_PRINTLN(isLpf2Host ? LUMP_UART_SPEED_LPF2 : LUMP_UART_SPEED_MIN);
        LUMP_DEBUG_PRINTLN("[State] Init UART");

        initUart(isLpf2Host ? LUMP_UART_SPEED_LPF2 : LUMP_UART_SPEED_MIN);

        prevMillis  = currentMillis;
        deviceState = LumpDeviceState::WaitingUartInit;

        LUMP_DEBUG_PRINTLN("[State] Waiting for UART init...");
        break;

      case LumpDeviceState::WaitingUartInit:
        /**
         * Waits for UART initialization.
         *
         * Waits `LUMP_UART_INIT_DELAY` milliseconds to ensure UART initialization is complete.
         * After initialization, sends an ACK to notify LPF2 hosts that the `LUMP_UART_SPEED_LPF2` speed
         * will be used for the handshake process.
         */
        if (currentMillis - prevMillis > LUMP_UART_INIT_DELAY) {
          LUMP_DEBUG_PRINTLN("[Info] UART init complete");

          if (isLpf2Host) {
            LUMP_DEBUG_PRINTLN("[Info] Sends ACK to LPF2 host");
            uartWrite(LUMP_SYS_ACK);
          }

          deviceState = LumpDeviceState::SendingType;
        }
        break;

      case LumpDeviceState::SendingType:
        /* Sends the device type. */
        LUMP_DEBUG_PRINTLN("[State] Sending type");

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_CMD, 1, LUMP_CMD_TYPE);
        txBuffer[1] = type;
        txBuffer[2] = calcChecksum(txBuffer, 2);
        uartWrite(txBuffer, 3);

        deviceState = LumpDeviceState::SendingModes;
        break;

      case LumpDeviceState::SendingModes: {
        /* Sends the numbers of modes and views. */
        LUMP_DEBUG_PRINTLN("[State] Sending modes");

        uint8_t maxView     = view - 1;
        uint8_t lpf2MaxMode = numModes - 1;
        uint8_t ev3MaxMode  = min(lpf2MaxMode, static_cast<uint8_t>(LUMP_MAX_MODE));

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_CMD, 4, LUMP_CMD_MODES);
        txBuffer[1] = ev3MaxMode;
        txBuffer[2] = (maxView > ev3MaxMode) ? ev3MaxMode : maxView;
        txBuffer[3] = lpf2MaxMode;
        txBuffer[4] = (maxView > lpf2MaxMode) ? lpf2MaxMode : maxView;
        txBuffer[5] = calcChecksum(txBuffer, 5);
        uartWrite(txBuffer, 6);

        deviceState = LumpDeviceState::SendingSpeed;
        break;
      }

      case LumpDeviceState::SendingSpeed:
        /* Sends the communication speed. */
        LUMP_DEBUG_PRINTLN("[State] Sending speed");

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_CMD, 4, LUMP_CMD_SPEED);
        memcpy(&txBuffer[1], &speed, 4);
        txBuffer[5] = calcChecksum(txBuffer, 5);
        uartWrite(txBuffer, 6);

        deviceState = LumpDeviceState::SendingVersion;
        break;

      case LumpDeviceState::SendingVersion: {
        /**
         * Sends the firmware and hardware version.
         *
         * Notes:
         * - Version information is required by SPIKE3 firmware.
         *   Without it, communication will not function correctly even if the handshake succeeds.
         */
        LUMP_DEBUG_PRINTLN("[State] Sending version");

        uint32_t fwVersionBcd = versionToBcd(fwVersion);
        uint32_t hwVersionBcd = versionToBcd(hwVersion);

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_CMD, 8, LUMP_CMD_VERSION);
        memcpy(&txBuffer[1], &fwVersionBcd, 4);
        memcpy(&txBuffer[5], &hwVersionBcd, 4);
        txBuffer[9] = calcChecksum(txBuffer, 9);
        uartWrite(txBuffer, 10);

        // Prepare to send mode information.
        modeIdx     = numModes - 1; // Start from the last mode.
        deviceState = LumpDeviceState::SendingName;
        break;
      }

      case LumpDeviceState::SendingName: {
        /* Sends the mode name and flags. */
        LUMP_DEBUG_PRINT("[INFO] Sends mode ");
        LUMP_DEBUG_PRINTLN(modeIdx);
        LUMP_DEBUG_PRINTLN("[State] Sending name");

        const LumpMode &m = modes[modeIdx];

        /**
         * Name validation (moved here from the LumpMode constructor so the mode table can be constant).
         * An invalid name (nullptr, empty, or not starting with a letter) is sent as "null".
         */
        static const char nullName[LUMP_MAX_SHORT_NAME_SIZE + 8]{"null"};
        const char *name = nullName;
        if (m.flagsInName) {
          if (m.name) {
            name = m.name;
          }
        } else if (m.name && isalpha(static_cast<unsigned char>(m.name[0]))) {
          name = m.name;
        }

        uint8_t nameLen = boundedStrlen(name, LUMP_MAX_NAME_SIZE); // null terminator is not required by default.

#ifndef LUMP_FIX_SHORT_NAME
        /**
         * Reproduces the byte-exact output of library versions that copied the name into a buffer
         * pre-filled with "null": names shorter than 4 characters keep the tail of "null"
         * (e.g. "HSV" is sent as "HSVl"). Define LUMP_FIX_SHORT_NAME to send the name as written.
         */
        char legacyName[5];
        if (!m.flagsInName && nameLen < 4) {
          memcpy(legacyName, nullName, sizeof(legacyName));
          memcpy(legacyName, name, nameLen);
          name    = legacyName;
          nameLen = 4;
        }
#endif

        uint8_t msgSize = queryNextPow2(nameLen);

        memset(txBuffer, 0, sizeof(txBuffer));

        if (m.flagsInName) {
          nameLen = LUMP_MAX_SHORT_NAME_SIZE + 7;
          msgSize = queryNextPow2(LUMP_MAX_SHORT_NAME_SIZE + 7); // 1 for short name's null terminator, 6 for flags.
        } else if (m.power) {
          nameLen = min(nameLen, static_cast<uint8_t>(LUMP_MAX_SHORT_NAME_SIZE));
          msgSize = queryNextPow2(LUMP_MAX_SHORT_NAME_SIZE + 7); // 1 for short name's null terminator, 6 for flags.

          txBuffer[LUMP_MAX_SHORT_NAME_SIZE + 3] = LUMP_MODE_FLAGS0_NEEDS_SUPPLY_PIN2;
          txBuffer[LUMP_MAX_SHORT_NAME_SIZE + 8] = 0x84; // SPIKE3 firmware requires these unknown flags.
        }

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_INFO, msgSize, modeIdx % (LUMP_MAX_MODE + 1));
        txBuffer[1] = LUMP_INFO_NAME | LUMP_INFO_MODE(modeIdx);
        memcpy(&txBuffer[2], name, nameLen);
        txBuffer[msgSize + 2] = calcChecksum(txBuffer, msgSize + 2);
        uartWrite(txBuffer, msgSize + 3);

        deviceState = LumpDeviceState::SendingValueSpans;
        break;
      }

      case LumpDeviceState::SendingValueSpans:
        /* Sends the value spans. */
        LUMP_DEBUG_PRINTLN("[State] Sending value spans");

        sendValueSpan(modes[modeIdx].raw, LUMP_INFO_RAW);
        sendValueSpan(modes[modeIdx].pct, LUMP_INFO_PCT);
        sendValueSpan(modes[modeIdx].si, LUMP_INFO_SI);

        deviceState = LumpDeviceState::SendingSymbol;
        break;

      case LumpDeviceState::SendingSymbol: {
        /* Sends the symbol. */
        LUMP_DEBUG_PRINTLN("[State] Sending symbol");

        const char *symbol = modes[modeIdx].symbol;
        if (symbol && symbol[0] != '\0') {
          size_t symbolLen = boundedStrlen(symbol, LUMP_MAX_UOM_SIZE); // null terminator is not required.
          size_t msgSize   = queryNextPow2(symbolLen);

          memset(txBuffer, 0, sizeof(txBuffer));
          txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_INFO, msgSize, modeIdx % (LUMP_MAX_MODE + 1));
          txBuffer[1] = LUMP_INFO_UNITS | LUMP_INFO_MODE(modeIdx);
          memcpy(&txBuffer[2], symbol, symbolLen);
          txBuffer[msgSize + 2] = calcChecksum(txBuffer, msgSize + 2);
          uartWrite(txBuffer, msgSize + 3);
        }

        deviceState = LumpDeviceState::SendingMapping;
        break;
      }

      case LumpDeviceState::SendingMapping:
        /**
         * Sends the mode mapping.
         *
         * Notes:
         * - Mode supports writing in Pybricks firmware if mapOut is nonzero.
         *   See: https://github.com/pybricks/technical-info/blob/88a708c/uart-protocol.md#info_mapping
         *        https://github.com/pybricks/pybricks-micropython/blob/7779f86/lib/pbio/src/port_lump.c#L471
         */
        LUMP_DEBUG_PRINTLN("[State] Sending mapping");

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_INFO, 2, modeIdx % (LUMP_MAX_MODE + 1));
        txBuffer[1] = LUMP_INFO_MAPPING | LUMP_INFO_MODE(modeIdx);
        txBuffer[2] = modes[modeIdx].mapIn;
        txBuffer[3] = modes[modeIdx].mapOut; // see note 1
        txBuffer[4] = calcChecksum(txBuffer, 4);
        uartWrite(txBuffer, 5);

        deviceState = LumpDeviceState::SendingFormat;
        break;

      case LumpDeviceState::SendingFormat:
        /**
         * Sends the data format.
         *
         * After this state:
         * - If there are remaining modes to send,
         *   transition to `LumpDeviceState::InterModePause` to prepare for the next mode.
         * - If all modes have been sent,
         *   transition to `LumpDeviceState::SendingAck` to finalize the handshake sequence.
         */
        LUMP_DEBUG_PRINTLN("[State] Sending format");

        txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_INFO, 4, modeIdx % (LUMP_MAX_MODE + 1));
        txBuffer[1] = LUMP_INFO_FORMAT | LUMP_INFO_MODE(modeIdx);
        txBuffer[2] = modes[modeIdx].numData;
        txBuffer[3] = modes[modeIdx].dataType;
        txBuffer[4] = modes[modeIdx].figures;
        txBuffer[5] = modes[modeIdx].decimals;
        txBuffer[6] = calcChecksum(txBuffer, 6);
        uartWrite(txBuffer, 7);

        feedWdt();
        if (modeIdx == 0) {
          deviceState = LumpDeviceState::SendingAck;
        } else {
          LUMP_DEBUG_PRINTLN("[State] Inter-mode pause");

          prevMillis  = currentMillis;
          deviceState = LumpDeviceState::InterModePause;
        }
        break;

      case LumpDeviceState::InterModePause:
        /**
         * Pauses for `LUMP_INTER_MODE_PAUSE` milliseconds between sending information for different modes
         * to allow the host to save the information.
         */
        if (currentMillis - prevMillis > LUMP_INTER_MODE_PAUSE) {
          --modeIdx;
          deviceState = LumpDeviceState::SendingName;
        }
        break;

      case LumpDeviceState::SendingAck:
        /* Sends an ACK to notify the host that all information has been sent and is ready for communication. */
        LUMP_DEBUG_PRINTLN("[State] Sending ACK");

        uart->flush(); // FIXME: Ensure non-blocking behavior.
        uartWrite(LUMP_SYS_ACK);

        prevMillis  = millis();
        deviceState = LumpDeviceState::WaitingAckReply;

        LUMP_DEBUG_PRINTLN("[State] Waiting for ACK reply...");
        break;

      case LumpDeviceState::WaitingAckReply:
        /**
         * Waits for the ACK reply.
         *
         * Waits until an ACK is received, then transitions to `LumpDeviceState::SwitchingUartSpeed`.
         * See `processRxMsg()` for details.
         *
         * If an ACK is not received within `LUMP_ACK_TIMEOUT` milliseconds, transitions to `LumpDeviceState::Reset`.
         */
        if (currentMillis - prevMillis > LUMP_ACK_TIMEOUT) {
          LUMP_DEBUG_PRINTLN("[Error] Handshake failed");
          deviceState = LumpDeviceState::Reset;
        }
        break;

      case LumpDeviceState::SwitchingUartSpeed:
        /* Switches UART to the communication speed. */
        LUMP_DEBUG_PRINTLN("[State] Switching UART speed");
        LUMP_DEBUG_PRINT("[INFO] Communication speed: ");
        LUMP_DEBUG_PRINTLN(speed);

        initUart(speed);

        deviceState = LumpDeviceState::InitMode;
        break;

      /* Communication phase */
      case LumpDeviceState::InitMode:
        /**
         * Initializes the mode after:
         * - Handshake completes.
         * - Mode changed. See `processRxMsg()` for details.
         *
         * Notes:
         * - Developers are responsible for implementing this state.
         *   See: https://github.com/devilhyt/lump-device-builder-library#quickstart.
         */
        LUMP_DEBUG_PRINT("[State] Init Mode: ");
        LUMP_DEBUG_PRINTLN(deviceMode);

        nackMillis  = currentMillis;
        deviceState = LumpDeviceState::Communicating;
        break;

      case LumpDeviceState::Communicating:
        /**
         * Communicates with the host.
         *
         * Notes:
         * - Developers are responsible for implementing this state.
         *   See: https://github.com/devilhyt/lump-device-builder-library#quickstart.
         */
        if (currentMillis - nackMillis > LUMP_NACK_TIMEOUT) {
          /* NACK timeout. Soft reset the device. */
          LUMP_DEBUG_PRINTLN("[Error] NACK timeout");
          LUMP_DEBUG_PRINTLN("[Info] Soft reset...");

          deviceState = LumpDeviceState::Reset;
        }
        break;

      case LumpDeviceState::SendingNack:
        /**
         * Sends a NACK to notify the host that the received message is invalid.
         */
        LUMP_DEBUG_PRINTLN("[State] Sending NACK");

        uartWrite(LUMP_SYS_NACK);

        deviceState = prevDeviceState;
        break;

      default:
        break;
    }
  }

  template <typename T>
  void LumpDevice<T>::processRxMsg() {
    using namespace LumpDeviceBuilder::Internal;

    switch (receiverState) {
      case LumpReceiverState::ReadByte: {
        /* Reads a byte. */
        if (!uart->available()) {
          break;
        }

        rxBuffer[rxIdx] = uart->read();

        if (rxIdx == 0) {
          receiverState = LumpReceiverState::ParseMsgType;
        } else if (rxIdx >= rxLen - 1) {
          receiverState = LumpReceiverState::VerityChecksum;
        }

        ++rxIdx;
        break;
      }

      case LumpReceiverState::ParseMsgType: {
        /* Parses the message type. */
        if (rxBuffer[0] == LUMP_SYS_SYNC || rxBuffer[0] == LUMP_SYS_NACK || rxBuffer[0] == LUMP_SYS_ACK) {
          /* System message */
          rxIdx         = 0;
          rxLen         = 1;
          receiverState = LumpReceiverState::ProcessMsg;
          break;
        }

        uint8_t msgSize = LUMP_MSG_SIZE(rxBuffer[0]);
        if (msgSize <= LUMP_MAX_MSG_SIZE) {
          /* Other types of message */
          rxLen = msgSize + 2; // +2 for command byte and check byte.
        } else {
          /* Invalid message size. Discard this message byte. */
          LUMP_DEBUG_PRINT_RX_BUFFER(rxBuffer, 1);
          LUMP_DEBUG_PRINT("| invalid size: ");
          LUMP_DEBUG_PRINTLN(msgSize);
          rxIdx = 0;
        }
        receiverState = LumpReceiverState::ReadByte;
        break;
      }

      case LumpReceiverState::VerityChecksum: {
        /* Verifies the checksum of the message. */
        uint8_t checksum = calcChecksum(rxBuffer, rxLen - 1);

        if (checksum == rxBuffer[rxLen - 1]) {
          receiverState = LumpReceiverState::ProcessMsg;
        } else {
          LUMP_DEBUG_PRINT_RX_BUFFER(rxBuffer, rxLen);
          LUMP_DEBUG_PRINT("| checksum error: ");
          LUMP_DEBUG_PRINTLN(checksum);

          prevDeviceState = deviceState;
          deviceState     = LumpDeviceState::SendingNack;
          receiverState   = LumpReceiverState::ReadByte;
        }
        rxIdx = 0;
        break;
      }

      case LumpReceiverState::ProcessMsg: {
        /* Processes the message. */
        LUMP_DEBUG_PRINT_RX_BUFFER(rxBuffer, rxLen);

        uint8_t msgType = rxBuffer[0] & LUMP_MSG_TYPE_MASK;
        uint8_t msgSize = LUMP_MSG_SIZE(rxBuffer[0]);
        uint8_t msgCmd  = rxBuffer[0] & LUMP_MSG_CMD_MASK; // cmd or mode

        switch (msgType) {
          case LUMP_MSG_TYPE_SYS:
            switch (msgCmd) {
              case LUMP_SYS_SYNC:
                LUMP_DEBUG_PRINTLN("| SYNC");
                break;
              case LUMP_SYS_NACK:
                LUMP_DEBUG_PRINTLN("| NACK");

                if (deviceState == LumpDeviceState::Communicating) {
                  feedWdt();
                  _hasNack   = true;
                  nackMillis = currentMillis;
                }
                break;
              case LUMP_SYS_ACK:
                LUMP_DEBUG_PRINTLN("| ACK");

                if (deviceState == LumpDeviceState::WaitingAckReply) {
                  LUMP_DEBUG_PRINTLN("[Info] Handshake success");
                  deviceState = LumpDeviceState::SwitchingUartSpeed;
                }
                break;
              default:
                LUMP_DEBUG_PRINTLN("| unknown");
                break;
            }
            break;
          case LUMP_MSG_TYPE_CMD:
            switch (msgCmd) {
              case LUMP_CMD_SPEED:
                if (deviceState == LumpDeviceState::WaitingAutoId) {
                  LUMP_DEBUG_PRINT("| speed: ");
                  LUMP_DEBUG_PRINTLN(reinterpret_cast<uint32_t *>(&rxBuffer[1])[0]);
                  LUMP_DEBUG_PRINTLN("[Info] LPF2 host detected");

                  isLpf2Host  = true;
                  deviceState = LumpDeviceState::InitUart;
                }
                break;
              case LUMP_CMD_SELECT:
                if (deviceState == LumpDeviceState::Communicating) {
                  deviceMode  = rxBuffer[1];
                  deviceState = LumpDeviceState::InitMode;

                  LUMP_DEBUG_PRINT("| select mode: ");
                  LUMP_DEBUG_PRINTLN(deviceMode);
                }
                break;
              case LUMP_CMD_WRITE:
                if (deviceState == LumpDeviceState::Communicating) {
                  if (msgSize <= sizeof(cmdWriteData)) {
                    cmdWriteDataSize = msgSize;
                    memcpy(cmdWriteData, &rxBuffer[1], msgSize);
                    _hasCmdWriteData = true;
                  }

                  LUMP_DEBUG_PRINT("| cmd write data, size: ");
                  LUMP_DEBUG_PRINT(msgSize);
                  LUMP_DEBUG_PRINTLN((msgSize <= sizeof(cmdWriteData)) ? "" : ", invalid");
                }
                break;
              case LUMP_CMD_EXT_MODE:
                if (deviceState == LumpDeviceState::Communicating) {
                  extMode = rxBuffer[1];

                  LUMP_DEBUG_PRINT("| ext mode: ");
                  LUMP_DEBUG_PRINTLN(extMode);
                }
                break;
              default:
                LUMP_DEBUG_PRINTLN("| unknown");
                break;
            }
            break;
          case LUMP_MSG_TYPE_DATA:
            if (deviceState == LumpDeviceState::Communicating) {
              uint8_t mode = msgCmd + extMode;
              uint8_t *buf = dataMsgPtr(mode);
              bool isValid = buf && msgSize >= modes[mode].dataMsgSize;

              if (isValid) {
                memcpy(buf, &rxBuffer[1], modes[mode].dataMsgSize);
                dataMsgFlags |= static_cast<uint16_t>(1u << mode);
              }

              LUMP_DEBUG_PRINT("| data msg, mode: ");
              LUMP_DEBUG_PRINT(mode);
              LUMP_DEBUG_PRINT(", size: ");
              LUMP_DEBUG_PRINT(msgSize);
              LUMP_DEBUG_PRINTLN(isValid ? "" : ", invalid");
            }
            break;
          default:
            LUMP_DEBUG_PRINTLN("| unknown");
            break;
        }
        receiverState = LumpReceiverState::ReadByte;
        break;
      }
      default:
        break;
    }
  }

  template <typename T>
  void LumpDevice<T>::feedWdt() {
    if (feedWdtCallback) {
      LUMP_DEBUG_PRINTLN("[WDT] Feeds");
      feedWdtCallback();
    }
  }

  template <typename T>
  void LumpDevice<T>::initUart(uint32_t speed) {
    uart->end();
    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, HIGH);
    uart->begin(speed);
  }

  template <typename T>
  void LumpDevice<T>::sendValueSpan(const LumpValueSpan &valueSpan, uint8_t infoType) {
    using namespace LumpDeviceBuilder::Internal;

    if (valueSpan.isExist && valueSpan.isValid) {
      txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_INFO, 8, modeIdx % (LUMP_MAX_MODE + 1));
      txBuffer[1] = infoType | LUMP_INFO_MODE(modeIdx);
      memcpy(&txBuffer[2], &(valueSpan.min), 4);
      memcpy(&txBuffer[6], &(valueSpan.max), 4);
      txBuffer[10] = calcChecksum(txBuffer, 10);
      uartWrite(txBuffer, 11);
    }
  }

  template <typename T>
  bool LumpDevice<T>::hasNack() {
    bool tmp = _hasNack;
    _hasNack = false;
    return tmp;
  }

  template <typename T>
  void LumpDevice<T>::clearCmdWriteData() {
    cmdWriteDataSize = 0;
    _hasCmdWriteData = false;
    memset(cmdWriteData, 0, sizeof(cmdWriteData));
  }

  template <typename T>
  bool LumpDevice<T>::hasCmdWriteData() {
    bool tmp         = _hasCmdWriteData;
    _hasCmdWriteData = false;
    return tmp;
  }

  template <typename T>
  uint8_t *LumpDevice<T>::dataMsgPtr(uint8_t mode) {
    if (!dataMsgBuf || mode >= numModes || !modes[mode].isWritable()) {
      return nullptr;
    }
    uint16_t offset = 0;
    for (uint8_t i = 0; i < mode; ++i) {
      if (modes[i].isWritable()) {
        offset += align4(modes[i].dataMsgSize);
      }
    }
    return dataMsgBuf + offset;
  }

  template <typename T>
  void LumpDevice<T>::clearDataMsg(uint8_t mode) {
    uint8_t *buf = dataMsgPtr(mode);
    if (buf) {
      dataMsgFlags &= static_cast<uint16_t>(~(1u << mode));
      memset(buf, 0, modes[mode].dataMsgSize);
    }
  }

  template <typename T>
  bool LumpDevice<T>::hasDataMsg(uint8_t mode) {
    if (mode < numModes) {
      bool tmp = dataMsgFlags & (1u << mode);
      dataMsgFlags &= static_cast<uint16_t>(~(1u << mode));
      return tmp;
    }
    return false;
  }

  template <typename T>
  template <typename U>
  U *LumpDevice<T>::readDataMsg(uint8_t mode) {
    return reinterpret_cast<U *>(dataMsgPtr(mode));
  }

  template <typename T>
  void LumpDevice<T>::sendDataMsg(void *payload, uint8_t len, uint8_t mode) {
    using namespace LumpDeviceBuilder::Internal;

    if (numModes > LUMP_MAX_MODE + 1) {
      txBuffer[0] = encMsgHeader(LUMP_MSG_TYPE_CMD, 1, LUMP_CMD_EXT_MODE);
      txBuffer[1] = (mode > LUMP_MAX_MODE) ? LUMP_EXT_MODE_8 : LUMP_EXT_MODE_0;
      txBuffer[2] = calcChecksum(txBuffer, 2);
      uartWrite(txBuffer, 3);
    }

    uint8_t msgSize = queryNextPow2(len);
    txBuffer[0]     = encMsgHeader(LUMP_MSG_TYPE_DATA, msgSize, mode % (LUMP_MAX_MODE + 1));
    memcpy(&txBuffer[1], payload, len);
    txBuffer[msgSize + 1] = calcChecksum(txBuffer, msgSize + 1);
    uartWrite(txBuffer, msgSize + 2);
  }

} // namespace LumpDeviceBuilder

#endif // LUMP_DEVICE_BUILDER_IPP