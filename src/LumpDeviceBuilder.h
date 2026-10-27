// SPDX-FileCopyrightText: 2023-2025 OFDL Robotics Lab
// SPDX-FileCopyrightText: 2023-2025 HsiangYi Tsai <devilhyt@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * LUMP Device Builder Library
 *
 * This is an Arduino library that implements the LEGO UART Message Protocol (LUMP) for building custom devices.
 */

#ifndef LUMP_DEVICE_BUILDER_H
#define LUMP_DEVICE_BUILDER_H

#include "LumpDeviceBuilderDebug.h"
#include "lump_ext.h"
#include <Arduino.h>

/* Compile-time helpers (must be visible before LumpMode). */
namespace LumpDeviceBuilder {
namespace Internal {

  /**
   * Queries size of the LUMP data type.
   *
   * @param dataType LUMP data type.
   * @return Size of the data type in bytes.
   */
  constexpr uint8_t sizeOfLumpDataType(uint8_t dataType) {
    return (dataType == LUMP_DATA_TYPE_DATA8)                                       ? 1
         : (dataType == LUMP_DATA_TYPE_DATA16)                                      ? 2
         : (dataType == LUMP_DATA_TYPE_DATA32 || dataType == LUMP_DATA_TYPE_DATAF) ? 4
                                                                                    : 0;
  }

  /**
   * Bounded string length (portable replacement for POSIX `strnlen`).
   */
  inline uint8_t boundedStrlen(const char *s, uint8_t maxLen) {
    uint8_t n = 0;
    while (n < maxLen && s[n] != '\0') {
      ++n;
    }
    return n;
  }

} // namespace Internal
} // namespace LumpDeviceBuilder

/* Namespace for the LUMP Device Builder Library. */
namespace LumpDeviceBuilder {

  /* Represents the state of a LUMP device. */
  enum class LumpDeviceState : uint8_t {
    /* Initialization phase */
    InitWdt, // Initializing the watchdog timer.
    Reset,   // Reseting the device.
    /* Handshake phase */
    InitAutoId,         // Initializing the AutoID.
    WaitingAutoId,      // Waiting for the AutoID.
    InitUart,           // Initializing the UART.
    WaitingUartInit,    // Waiting for the UART initialization.
    SendingType,        // Sending the device type.
    SendingModes,       // Sending the numbers of modes and views.
    SendingSpeed,       // Sending the communication speed.
    SendingVersion,     // Sending the firmware and hardware version.
    SendingName,        // Sending the mode name and flags.
    SendingValueSpans,  // Sending the value spans.
    SendingSymbol,      // Sending the symbol.
    SendingMapping,     // Sending the mode mapping.
    SendingFormat,      // Sending the data format.
    InterModePause,     // Inter-mode pause.
    SendingAck,         // Sending an ACK.
    WaitingAckReply,    // Waiting for the ACK reply.
    SwitchingUartSpeed, // Switching UART to the communication speed.
    /* Communication phase */
    InitMode,      // Initializing the mode.
    Communicating, // Communicating with the host.
    SendingNack,   // Sending a NACK.
  };

  /* Represents the state of the LUMP receiver. */
  enum class LumpReceiverState : uint8_t {
    ReadByte,       // Reads a byte.
    ParseMsgType,   // Parses the message type.
    VerityChecksum, // Verifies the checksum of the message.
    ProcessMsg      // Processes the message.
  };

  /* Represents a value span of a LUMP device mode. */
  class LumpValueSpan {
    public:
      /**
       * Creates a value span.
       *
       * @param min Minimum value of the span.
       * @param max Maximum value of the span.
       */
      constexpr LumpValueSpan(float min, float max) : min{min}, max{max}, isValid{min <= max}, isExist{true} {}

      /**
       * Creates an empty value span.
       *
       * @param isExist Whether the value span exists (default: `false`).
       * @note Used to allow the handshake process to skip sending value span information.
       */
      constexpr LumpValueSpan(bool isExist = false) : isExist{isExist} {}

      float min{0};
      float max{0};
      bool isValid{false};
      bool isExist{true};
  };

  /* Represents a mode of a LUMP device. */
  class LumpMode {
    public:
      /**
       * Creates a mode.
       *
       * @param name Mode name.
       *   Naming Rules:
       *     - Must not be an empty string `""` or `nullptr`.
       *     - Must start with a letter (`A–Z`, `a–z`).
       *     If invalid, the name will be set to the string literal "null".
       *   Length Limit (excluding null terminator):
       *     - If the `power` parameter is `false`: `11`
       *     - If the `power` parameter is `true`: `5`
       *     Names exceeding the limit will be truncated.
       * @param dataType Data type.
       *   Possible values: `DATA8`, `DATA16`, `DATA32`, `DATAF`.
       * @param numData Number of data.
       *   The maximum depends on `dataType` due to 32-byte payload limit:
       *   - `DATA8`  (1 byte) : `[1..32]`
       *   - `DATA16` (2 bytes): `[1..16]`
       *   - `DATA32` (4 bytes): `[1..8]`
       *   - `DATAF`  (4 bytes): `[1..8]`
       * @param figures Number of characters shown in the view and datalog (including the decimal point).
       *   Valid range: `[0..15]`
       * @param decimals Number of decimals shown in the view and datalog.
       *   Valid range: `[0..15]`
       * @param symbol Symbol of the measurement unit (default: `""`).
       *   Rules:
       *   1. Set to empty string `""` or `nullptr` if not required.
       *   2. Length limit (excluding null terminator): `4`
       * @param raw Raw value span (default: `false`).
       *   When set to `false`, this value span will not be provided to the host.
       *   The host will use the default range `[0, 1023]`.
       * @param pct Percentage value span (default: `false`).
       *   When set to `false`, this value span will not be provided to the host.
       *   The host will use the default range `[0, 100]`.
       * @param si Scaled value span (default: `false`).
       *   When set to `false`, this value span will not be provided to the host.
       *   The host will use the default range `[0, 1023]`.
       * @param mapIn Mode mapping for input side (default: `LUMP_INFO_MAPPING_NONE`).
       *   Possible values: `LUMP_INFO_MAPPING_NONE`, `LUMP_INFO_MAPPING_DIS`
       *                    `LUMP_INFO_MAPPING_REL`, `LUMP_INFO_MAPPING_ABS`.
       *   Optionally bitwise OR-ed with: `LUMP_INFO_MAPPING_SUPPORT_FUNCTIONAL_MAPPING_2`,
       *                                  `LUMP_INFO_MAPPING_SUPPORT_NULL`.
       * @param mapOut Mode mapping for output side (default: `LUMP_INFO_MAPPING_NONE`).
       *   Possible values: `LUMP_INFO_MAPPING_NONE`, `LUMP_INFO_MAPPING_DIS`
       *                    `LUMP_INFO_MAPPING_REL`, `LUMP_INFO_MAPPING_ABS`.
       *   Optionally bitwise OR-ed with: `LUMP_INFO_MAPPING_SUPPORT_FUNCTIONAL_MAPPING_2`,
       *                                  `LUMP_INFO_MAPPING_SUPPORT_NULL`.
       * @param power Whether to enable constant power on SPIKE Hub pin 2 (default: `false`).
       * @param flagsInName Whether the `name` parameter contains flags (default: `false`).
       *   If set to `true`, flags can be added to the `name` parameter.
       *   In this case, the `power` parameter will be ignored.
       *   The total length of the name and flags combined must be exactly 13
       *   (5 for name + 1 null terminator + 6 flags + 1 null terminator).
       *   See: https://github.com/pybricks/technical-info/blob/88a708c/uart-protocol.md#info_name.
       * @warning When the `power` parameter is set to `true` in any mode,
       *          constant power on SPIKE Hub pin 2 is enabled across all modes.
       */
      constexpr LumpMode(
          const char *name,
          uint8_t dataType,
          uint8_t numData,
          uint8_t figures,
          uint8_t decimals,
          const char *symbol = "",
          LumpValueSpan raw  = false,
          LumpValueSpan pct  = false,
          LumpValueSpan si   = false,
          uint8_t mapIn      = LUMP_INFO_MAPPING_NONE,
          uint8_t mapOut     = LUMP_INFO_MAPPING_NONE,
          bool power         = false,
          bool flagsInName   = false
      )
          : raw{raw},
            pct{pct},
            si{si},
            name{name},
            symbol{symbol},
            dataType{dataType},
            numData{numData},
            figures{figures},
            decimals{decimals},
            mapIn{mapIn},
            mapOut{mapOut},
            power{power},
            flagsInName{flagsInName},
            dataTypeSize{Internal::sizeOfLumpDataType(dataType)},
            dataMsgSize{static_cast<uint8_t>(numData * Internal::sizeOfLumpDataType(dataType))} {}

      /**
       * Whether the host can write data to this mode.
       */
      constexpr bool isWritable() const { return mapOut != LUMP_INFO_MAPPING_NONE; }

      /* Mode info (immutable; the whole table can live in flash when declared `const`) */
      LumpValueSpan raw;
      LumpValueSpan pct;
      LumpValueSpan si;
      const char *name;   // Validated / truncated when sent during the handshake.
      const char *symbol; // Truncated to LUMP_MAX_UOM_SIZE when sent during the handshake.
      uint8_t dataType;
      uint8_t numData;
      uint8_t figures;
      uint8_t decimals;
      uint8_t mapIn;
      uint8_t mapOut;
      bool power;
      bool flagsInName;
      uint8_t dataTypeSize;
      uint8_t dataMsgSize;
  };

  /**
   * LUMP device class.
   *
   * @tparam T Type of the serial interface (typically `hardwareSerial`).
   */
  template <typename T>
  class LumpDevice {
    public:
      virtual ~LumpDevice() { free(dataMsgBuf); }
      LumpDevice(const LumpDevice &)            = delete; // owns dataMsgBuf
      LumpDevice(LumpDevice &&)                 = delete;
      LumpDevice &operator=(const LumpDevice &) = delete;
      LumpDevice &operator=(LumpDevice &&)      = delete;

      /**
       * Creates a device.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param uart Serial interface used for UART communication (e.g., `Serial0`, `Serial1`).
       * @param rxPin RX pin number of the serial interface.
       * @param txPin TX pin number of the serial interface.
       * @param type Device type.
       * @param speed Communication speed.
       * @param modes Device modes (must be an array of `LumpMode`).
       * @param numModes Number of modes.
       *   For SPIKE Hub: `[1..16]`
       *   For EV3: `[1..8]`
       *   Modes beyond the limit will be ignored.
       * @param view Number of modes to show in view and data log (default: `LUMP_VIEW_ALL`).
       *   Valid range: `[1..16]`.
       *   Set to `LUMP_VIEW_ALL` to show all modes.
       * @param fwVersion Firmware version (default: `10000000`).
       *   Valid range: `[10000000..99999999]`.
       *   The value `10000000` represents v1.0.00.0000.
       * @param hwVersion Hardware version (default: `10000000`).
       *   Valid range: `[10000000..99999999]`.
       *   The value `10000000` represents v1.0.00.0000.
       */
      LumpDevice(
          T *uart,
          uint8_t rxPin,
          uint8_t txPin,
          uint8_t type,
          uint32_t speed,
          const LumpMode *modes,
          uint8_t numModes,
          uint8_t view       = LUMP_VIEW_ALL,
          uint32_t fwVersion = 10000000,
          uint32_t hwVersion = 10000000
      );

      /**
       * Starts the device.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void begin();

      /**
       * Finishes the device.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void end();

      /**
       * Sets the watchdog timer callback functions.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param initWdtCallback Callback function to initialize the watchdog timer.
       * @param feedWdtCallback Callback function to feed the watchdog timer.
       * @param deinitWdtCallback Callback function to deinitialize the watchdog timer.
       */
      inline void setWdtCallback(void (*initWdtCallback)(), void (*feedWdtCallback)(), void (*deinitWdtCallback)()) {
        this->initWdtCallback   = initWdtCallback;
        this->feedWdtCallback   = feedWdtCallback;
        this->deinitWdtCallback = deinitWdtCallback;
      }

      /**
       * Runs the device.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void run();

      /**
       * Gets the device state.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @return Device state.
       */
      inline LumpDeviceState state() { return deviceState; }

      /**
       * Gets the device mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @return Device mode.
       */
      inline uint8_t mode() { return deviceMode; }

      /**
       * Checks if the device is in communication phase.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @retval true The device is in communication phase.
       * @retval false Otherwise.
       */
      inline bool isCommunicating() { return deviceState >= LumpDeviceState::InitMode; }

      /**
       * Checks for a newly received NACK.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @retval true A newly received NACK is available.
       * @retval false Otherwise.
       * @note This function automatically clears the flag after checking.
       */
      bool hasNack();

      /**
       * Clears the command write data.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void clearCmdWriteData();

      /**
       * Checks for a newly received command write data.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @retval true A newly received command write data is available.
       * @retval false Otherwise.
       * @note This function automatically clears the flag after checking.
       */
      bool hasCmdWriteData();

      /**
       * Reads the command write data.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the command write data.
       * @return U* Pointer to the command write data array.
       */
      template <typename U>
      inline U *readCmdWriteData() {
        return reinterpret_cast<U *>(cmdWriteData);
      }

      /**
       * Clears the data message for the specified mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param mode Mode number.
       */
      void clearDataMsg(uint8_t mode);

      /**
       * Checks for a newly received data message for the specified mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param mode Mode number.
       * @retval true A newly received data message is available.
       * @retval false Otherwise.
       * @note This function automatically clears the flag after checking.
       */
      bool hasDataMsg(uint8_t mode);

      /**
       * Reads the data message of the specified mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @param mode Mode number.
       * @retval U* Pointer to the the data message array if the `mode` is valid and the data message is available.
       * @retval nullptr Otherwise.
       */
      template <typename U>
      U *readDataMsg(uint8_t mode);

      /**
       * Sends a data array of the specified type.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @param data Pointer to the data array.
       * @param num Number of data in the data array.
       */
      template <typename U>
      inline void send(U *data, uint8_t num) {
        sendDataMsg(reinterpret_cast<void *>(data), num * sizeof(U), deviceMode);
      }

      /**
       * Sends a data array of the specified type for a specific mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @param data Pointer to the data array.
       * @param num Number of data in the data array.
       * @param mode Mode number.
       */
      template <typename U>
      inline void send(U *data, uint8_t num, uint8_t mode) {
        sendDataMsg(reinterpret_cast<void *>(data), num * sizeof(U), mode);
      }

      /**
       * Sends a data array of the specified type.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @tparam N Size of the data array.
       * @param data Pointer to the data array.
       * @param num Number of data in the data array.
       */
      template <typename U, size_t N>
      inline void send(U (*data)[N], uint8_t num) {
        sendDataMsg(reinterpret_cast<void *>(data), num * sizeof(U), deviceMode);
      };

      /**
       * Sends a data array of the specified type for a specific mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @tparam N Size of the data array.
       * @param data Pointer to the data array.
       * @param num Number of data in the data array.
       * @param mode Mode number.
       */
      template <typename U, size_t N>
      inline void send(U (*data)[N], uint8_t num, uint8_t mode) {
        sendDataMsg(reinterpret_cast<void *>(data), num * sizeof(U), mode);
      };

      /**
       * Sends a data value of the specified type.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @param data A data value.
       */
      template <typename U>
      inline void send(U data) {
        sendDataMsg(reinterpret_cast<void *>(&data), sizeof(U), deviceMode);
      }

      /**
       * Sends a data value of the specified type for a specific mode.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @tparam U Type of the data.
       * @param data A data value.
       * @param mode Mode number.
       */
      template <typename U>
      inline void send(U data, uint8_t mode) {
        sendDataMsg(reinterpret_cast<void *>(&data), sizeof(U), mode);
      }

    protected:
      /**
       * Runs the device state machine.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void _run();

      /**
       * Processes the RX messages.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void processRxMsg();

      /**
       * Feeds the watchdog timer.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       */
      void feedWdt();

      /**
       * Initializes the UART.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param speed Speed.
       */
      void initUart(uint32_t speed);

      /**
       * Writes a message over UART.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param msg A message to write.
       * @param len Length of the message.
       */
      inline void uartWrite(uint8_t *msg, uint8_t len) { uart->write(msg, len); }

      /**
       * Writes a message over UART.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param msg Message to write.
       */
      inline void uartWrite(uint8_t msg) { uart->write(msg); }

      /**
       * Sends a value span.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param valueSpan Reference to the value span (LumpValueSpan).
       * @param valueType Type of value span.
       *   Possible values: `LUMP_INFO_RAW`, `LUMP_INFO_PCT`, `LUMP_INFO_SI`.
       */
      void sendValueSpan(const LumpValueSpan &valueSpan, uint8_t valueType);

      /**
       * Sends a data message to the host.
       *
       * @tparam T Type of the serial interface (typically `hardwareSerial`).
       * @param payload Pointer to the payload array.
       * @param len Number of data in the payload array.
       * @param mode Mode number.
       */
      void sendDataMsg(void *payload, uint8_t len, uint8_t mode);

      /**
       * Gets the storage of the data message of the specified mode.
       *
       * @param mode Mode number.
       * @retval uint8_t* Pointer into `dataMsgBuf` (4-byte aligned) if the mode is writable.
       * @retval nullptr Otherwise.
       */
      uint8_t *dataMsgPtr(uint8_t mode);

      /**
       * Rounds a size up to a multiple of 4 so every mode's buffer stays 4-byte aligned
       * (needed for int32/float access on Cortex-M0/M0+ and RV32 cores without misaligned access).
       */
      static constexpr uint16_t align4(uint16_t n) { return static_cast<uint16_t>((n + 3u) & ~3u); }

      /**
       * Initializes the watchdog timer.
       */
      void (*initWdtCallback)() = nullptr;

      /**
       * Feeds the watchdog timer.
       */
      void (*feedWdtCallback)() = nullptr;

      /**
       * Deinitializes the watchdog timer.
       */
      void (*deinitWdtCallback)() = nullptr;

      /* UART */
      T *uart;
      uint8_t rxPin;
      uint8_t txPin;

      /* Device info */
      uint8_t type;
      uint32_t speed;
      const LumpMode *modes;
      uint8_t numModes;
      uint8_t view;
      uint32_t fwVersion;
      uint32_t hwVersion;

      /* Host info */
      bool isLpf2Host;

      /* Device mode */
      uint8_t deviceMode{0};
      uint8_t extMode{0};
      int8_t modeIdx;

      /* State machine */
      LumpDeviceState deviceState{LumpDeviceState::InitWdt};
      LumpDeviceState prevDeviceState{LumpDeviceState::InitWdt};
      LumpReceiverState receiverState{LumpReceiverState::ReadByte};

      /* Timing */
      uint32_t currentMillis;
      uint32_t prevMillis;
      uint32_t nackMillis;

      /* TX */
      uint8_t txBuffer[LUMP_UART_BUFFER_SIZE]{};

      /* RX */
      uint8_t rxBuffer[LUMP_UART_BUFFER_SIZE]{};
      uint8_t rxLen{0};
      uint8_t rxIdx{0};
      bool _hasNack{false};

      /* Data messages received from the host (one shared buffer for all writable modes) */
      uint8_t *dataMsgBuf{nullptr};
      uint16_t dataMsgBufSize{0};
      uint16_t dataMsgFlags{0}; // bit n = new data message for mode n

      /* Command write message */
      uint8_t cmdWriteData[LUMP_MAX_MSG_SIZE]{};
      uint8_t cmdWriteDataSize{0};
      bool _hasCmdWriteData{false};
  };

} // namespace LumpDeviceBuilder

/* Internal namespace for the LUMP Device Builder Library. */
namespace LumpDeviceBuilder::Internal {

  /**
   * Converts a decimal version number to BCD format.
   *
   * @param version Version number.
   * @return Version number in BCD format.
   */
  uint32_t versionToBcd(uint32_t version);

  /**
   * Calculates the checksum of a message.
   *
   * @param msg Pointer to the message.
   * @param size Size of the message.
   * @return Checksum of the message.
   */
  uint8_t calcChecksum(uint8_t *msg, uint8_t size);

  /**
   * Queries log2 (up to 32).
   *
   * @param x Value to query.
   * @return Base-2 logarithm of `x`.
   */
  uint8_t queryLog2(uint8_t x);

  /**
   * Queries the next power of 2 (up to 32).
   *
   * @param x Value to query.
   * @return Smallest power of 2 greater than or equal to `x`.
   */
  uint8_t queryNextPow2(uint8_t x);

  /**
   * Encodes a message header.
   *
   * @param msgType Message type (`lump_msg_type_t`).
   * @param size Size of the payload.
   * @param cmd Command or mode number (`lump_cmd_t`).
   * @return Encoded message header.
   */
  inline uint8_t encMsgHeader(uint8_t msgType, uint8_t size, uint8_t cmd) {
    return msgType | (queryLog2(size) << LUMP_MSG_SIZE_SHIFT) | cmd;
  }

} // namespace LumpDeviceBuilder::Internal

#include "LumpDeviceBuilder.ipp"

using namespace LumpDeviceBuilder;

#endif /* LUMP_DEVICE_BUILDER_H */