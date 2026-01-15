/**********************************************************************/
/**
 * @brief  ICS Protocol Controller
 * @author naoa
 */
/**********************************************************************/
#pragma once
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Include files
 *----------------------------------------------------------------------
 */

#include <Arduino.h>

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Debug macro
 *----------------------------------------------------------------------
 */

#if 1
#define ICS_DP__(...)           (Serial.printf(__VA_ARGS__))
#else
#define ICS_DP__(...)
#endif

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Type definitions
 *----------------------------------------------------------------------
 */

// UART write function pointer: write single byte
typedef void (*ics_uart_write_func_t)(uint8_t);

// UART read function pointer: read single byte, returns -1 if no data
typedef int (*ics_uart_read_func_t)(void);

// UART available function pointer: returns number of available bytes
typedef int (*ics_uart_available_func_t)(void);

// UART flush function pointer: flush TX buffer
typedef void (*ics_uart_flush_func_t)(int);

// GPIO control function pointer: set pin mode and value
typedef void (*ics_insel_set_func_t)(int value);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

// ICS Main Commands
constexpr uint8_t ICS_MC_POS   = 0x80;  // Position command
constexpr uint8_t ICS_MC_READ  = 0xA0;  // Read command
constexpr uint8_t ICS_MC_WRITE = 0xC0;  // Write command
constexpr uint8_t ICS_MC_ID    = 0xE0;  // ID command

// ICS Sub Commands
constexpr uint8_t ICS_SC_EEPROM = 0x00;  // EEPROM
constexpr uint8_t ICS_SC_STRC   = 0x01;  // Stretch
constexpr uint8_t ICS_SC_SPD    = 0x02;  // Speed
constexpr uint8_t ICS_SC_CUR    = 0x03;  // Current
constexpr uint8_t ICS_SC_TMP    = 0x04;  // Temperature
constexpr uint8_t ICS_SC_TCH    = 0x05;  // Position

constexpr int ICS_EEPROM_SIZE   = 64;    // EEPROM size in bytes
constexpr int ICS_MAX_SERVO_ID  = 31;    // Maximum servo ID

// ICS EEPROM Structure Definitions (byte offsets)
constexpr int ICS_EEPROM_BACKUP_CHR       = 0;     // Backup character high (0x5A)
constexpr int ICS_EEPROM_STRETCH          = 2;     // Stretch gain high byte
constexpr int ICS_EEPROM_SPEED            = 4;     // Speed high byte
constexpr int ICS_EEPROM_PUNCH            = 6;     // Punch high byte
constexpr int ICS_EEPROM_DEADBAND         = 8;     // Deadband high byte
constexpr int ICS_EEPROM_DAMPING          = 10;    // Damping high byte
constexpr int ICS_EEPROM_PROTECTION       = 12;    // Protection timer high byte
constexpr int ICS_EEPROM_FLAG             = 14;    // Flag (combined)
constexpr int ICS_EEPROM_PULSE_LIMIT_UPP  = 16;    // Pulse limit upper (high byte upper)
constexpr int ICS_EEPROM_PULSE_LIMIT_LOW  = 20;    // Pulse limit lower (high byte upper)
// Reserved: 24-25
constexpr int ICS_EEPROM_BAUD_RATE        = 26;    // Baud rate high byte
constexpr int ICS_EEPROM_TEMP_LIMIT       = 28;    // Temperature limit high byte
constexpr int ICS_EEPROM_CURRENT_LIMIT    = 30;    // Current limit high byte
// Reserved: 32-49
constexpr int ICS_EEPROM_RESPONSE         = 50;    // Response high byte
constexpr int ICS_EEPROM_USER_OFFSET      = 52;    // User offset high byte
// Reserved: 54-55
constexpr int ICS_EEPROM_ID               = 56;    // ID high byte
constexpr int ICS_EEPROM_CHARACTERISTIC_STRETCH_1 = 58; // Characteristic stretch 1 high byte
constexpr int ICS_EEPROM_CHARACTERISTIC_STRETCH_2 = 60; // Characteristic stretch 2 high byte
constexpr int ICS_EEPROM_CHARACTERISTIC_STRETCH_3 = 62; // Characteristic stretch 3 high byte

// Flag Field Definitions
// Flag (ICS_EEPROM_FLAG)
constexpr int ICS_FLAG_ROT_MODE_BIT = 4; // HI bit 0 -> 4
constexpr int ICS_FLAG_SLAVE_BIT    = 7; // HI bit 3 -> 7
constexpr int ICS_FLAG_REVERSE_BIT  = 0; // LO bit 0 -> 0
constexpr int ICS_FLAG_FREE_BIT     = 1; // LO bit 1 -> 1
constexpr int ICS_FLAG_PWMINH_BIT   = 3; // LO bit 3 -> 3

// EEPROM Validation constants
constexpr uint8_t ICS_EEPROM_BACKUP_CHR_VAL = 0x5A;  // Backup character validation value

// ICS Parameter Value Ranges
constexpr uint8_t ICS_MIN_STRETCH       = 1;     // Minimum stretch value
constexpr uint8_t ICS_MAX_STRETCH       = 127;   // Maximum stretch value
constexpr uint8_t ICS_MIN_SPEED         = 1;     // Minimum speed value
constexpr uint8_t ICS_MAX_SPEED         = 127;   // Maximum speed value
constexpr uint8_t ICS_MIN_CURRENT_LIMIT = 1;     // Minimum current limit
constexpr uint8_t ICS_MAX_CURRENT_LIMIT = 63;    // Maximum current limit
constexpr uint8_t ICS_MIN_TEMP_LIMIT    = 1;     // Minimum temperature limit
constexpr uint8_t ICS_MAX_TEMP_LIMIT    = 127;   // Maximum temperature limit

// ICS Baud Rate Values
constexpr uint8_t ICS_BAUD_RATE_1_25M   = 0x00;  // 1.25 Mbps
constexpr uint8_t ICS_BAUD_RATE_625K    = 0x01;  // 625000 bps
constexpr uint8_t ICS_BAUD_RATE_115K2   = 0x0A;  // 115200 bps

// ICS Timing Parameters
constexpr int ICS_MODE_SWITCH_DELAY_US  = 1;      // Delay after SEL pin mode change (microseconds)
constexpr int ICS_TX_COMPLETE_DELAY_US  = 5000;   // Delay for TX completion (microseconds)
constexpr int ICS_RX_TIMEOUT_MS         = 5;      // RX response timeout (milliseconds)
constexpr int ICS_RX_TIMEOUT_EEPROM_MS  = 1000;   // RX response timeout for EEPROM read (milliseconds)

constexpr bool IS_ICS_VALID_ID(uint8_t id) { return id <= ICS_MAX_SERVO_ID; }

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Class definitions
 *----------------------------------------------------------------------
 */

class ICSCtrlEEPROM
{
public:
    uint8_t data[ICS_EEPROM_SIZE] = {0};

public:
    /**
     * @brief Validate EEPROM data
     * @return true if backup character is valid (0x5A), false otherwise
     */
    bool isValid(void) const
    {
        return get8bit(ICS_EEPROM_BACKUP_CHR) == ICS_EEPROM_BACKUP_CHR_VAL;
    }

    // EEPROM parameter accessors
    uint8_t getStretch(void) const { return get8bit(ICS_EEPROM_STRETCH); }
    void setStretch(uint8_t stretch) { set8bit(ICS_EEPROM_STRETCH, stretch); }

    uint8_t getSpeed(void) const { return get8bit(ICS_EEPROM_SPEED); }
    void setSpeed(uint8_t speed) { set8bit(ICS_EEPROM_SPEED, speed); }

    uint8_t getPunch(void) const { return get8bit(ICS_EEPROM_PUNCH); }
    void setPunch(uint8_t punch) { set8bit(ICS_EEPROM_PUNCH, punch); }

    uint8_t getDeadband(void) const { return get8bit(ICS_EEPROM_DEADBAND); }
    void setDeadband(uint8_t deadband) { set8bit(ICS_EEPROM_DEADBAND, deadband); }

    uint8_t getDamping(void) const { return get8bit(ICS_EEPROM_DAMPING); }
    void setDamping(uint8_t damping) { set8bit(ICS_EEPROM_DAMPING, damping); }

    uint8_t getProtectionTimer(void) const { return get8bit(ICS_EEPROM_PROTECTION); }
    void setProtectionTimer(uint8_t timer) { set8bit(ICS_EEPROM_PROTECTION, timer); }

    bool getFlagRotationMode(void) const { return (get8bit(ICS_EEPROM_FLAG) & (1 << ICS_FLAG_ROT_MODE_BIT)) != 0; }
    void setFlagRotationMode(bool value) {
        uint8_t val = get8bit(ICS_EEPROM_FLAG);
        if (value) val |= (1 << ICS_FLAG_ROT_MODE_BIT);
        else val &= ~(1 << ICS_FLAG_ROT_MODE_BIT);
        set8bit(ICS_EEPROM_FLAG, val);
    }

    bool getFlagSlave(void) const { return (get8bit(ICS_EEPROM_FLAG) & (1 << ICS_FLAG_SLAVE_BIT)) != 0; }
    void setFlagSlave(bool value) {
        uint8_t val = get8bit(ICS_EEPROM_FLAG);
        if (value) val |= (1 << ICS_FLAG_SLAVE_BIT);
        else val &= ~(1 << ICS_FLAG_SLAVE_BIT);
        set8bit(ICS_EEPROM_FLAG, val);
    }

    bool getFlagReverse(void) const { return (get8bit(ICS_EEPROM_FLAG) & (1 << ICS_FLAG_REVERSE_BIT)) != 0; }
    void setFlagReverse(bool value) {
        uint8_t val = get8bit(ICS_EEPROM_FLAG);
        if (value) val |= (1 << ICS_FLAG_REVERSE_BIT);
        else val &= ~(1 << ICS_FLAG_REVERSE_BIT);
        set8bit(ICS_EEPROM_FLAG, val);
    }

    bool getFlagFree(void) const { return (get8bit(ICS_EEPROM_FLAG) & (1 << ICS_FLAG_FREE_BIT)) != 0; }
    void setFlagFree(bool value) {
        uint8_t val = get8bit(ICS_EEPROM_FLAG);
        if (value) val |= (1 << ICS_FLAG_FREE_BIT);
        else val &= ~(1 << ICS_FLAG_FREE_BIT);
        set8bit(ICS_EEPROM_FLAG, val);
    }

    bool getFlagPWMINH(void) const { return (get8bit(ICS_EEPROM_FLAG) & (1 << ICS_FLAG_PWMINH_BIT)) != 0; }
    void setFlagPWMINH(bool value) {
        uint8_t val = get8bit(ICS_EEPROM_FLAG);
        if (value) val |= (1 << ICS_FLAG_PWMINH_BIT);
        else val &= ~(1 << ICS_FLAG_PWMINH_BIT);
        set8bit(ICS_EEPROM_FLAG, val);
    }

    uint16_t getPulseLimitUpper(void) const { return get16bit(ICS_EEPROM_PULSE_LIMIT_UPP); }
    void setPulseLimitUpper(uint16_t limit) { set16bit(ICS_EEPROM_PULSE_LIMIT_UPP, limit); }

    uint16_t getPulseLimitLower(void) const { return get16bit(ICS_EEPROM_PULSE_LIMIT_LOW); }
    void setPulseLimitLower(uint16_t limit) { set16bit(ICS_EEPROM_PULSE_LIMIT_LOW, limit); }

    uint8_t getBaudRate(void) const { return get8bit(ICS_EEPROM_BAUD_RATE); }
    void setBaudRate(uint8_t baudRate) { set8bit(ICS_EEPROM_BAUD_RATE, baudRate); }

    uint8_t getTemperatureLimit(void) const { return get8bit(ICS_EEPROM_TEMP_LIMIT); }
    void setTemperatureLimit(uint8_t limit) { set8bit(ICS_EEPROM_TEMP_LIMIT, limit); }

    uint8_t getCurrentLimit(void) const { return get8bit(ICS_EEPROM_CURRENT_LIMIT); }
    void setCurrentLimit(uint8_t limit) { set8bit(ICS_EEPROM_CURRENT_LIMIT, limit); }

    uint8_t getResponse(void) const { return get8bit(ICS_EEPROM_RESPONSE); }
    void setResponse(uint8_t response) { set8bit(ICS_EEPROM_RESPONSE, response); }

    int8_t getUserOffset(void) const { return (int8_t)get8bit(ICS_EEPROM_USER_OFFSET); }
    void setUserOffset(int8_t offset) { set8bit(ICS_EEPROM_USER_OFFSET, (uint8_t)offset); }

    uint8_t getID(void) const { return get8bit(ICS_EEPROM_ID); }
    void setID(uint8_t id) { set8bit(ICS_EEPROM_ID, id); }

    uint8_t getCharacteristicStretch1(void) const { return get8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_1); }
    void setCharacteristicStretch1(uint8_t stretch) { set8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_1, stretch); }

    uint8_t getCharacteristicStretch2(void) const { return get8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_2); }
    void setCharacteristicStretch2(uint8_t stretch) { set8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_2, stretch); }

    uint8_t getCharacteristicStretch3(void) const { return get8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_3); }
    void setCharacteristicStretch3(uint8_t stretch) { set8bit(ICS_EEPROM_CHARACTERISTIC_STRETCH_3, stretch); }

public:
    void dump(void) const
    {
        ICS_DP__("ICSCtrlEEPROM Dump:\n");
        for (int i = 0; i < ICS_EEPROM_SIZE; i++) {
            ICS_DP__(" %02X", data[i]);
            if ((i % 16) == 15) {
                ICS_DP__("\n");
            }
        }
        ICS_DP__("\n");
    }

private:
    // Helper functions for 4-bit pair encoding/decoding
    uint8_t get8bit(int index) const
    {
        uint8_t h = (data[index] & 0x0F);
        uint8_t l = (data[index + 1] & 0x0F);
        return (h << 4) | l;
    }

    void set8bit(int index, uint8_t value)
    {
        data[index] = (value >> 4) & 0x0F;
        data[index + 1] = value & 0x0F;
    }

    uint16_t get16bit(int index) const
    {
        uint8_t hh = (data[index] & 0x0F);
        uint8_t hl = (data[index + 1] & 0x0F);
        uint8_t lh = (data[index + 2] & 0x0F);
        uint8_t ll = (data[index + 3] & 0x0F);
        return ((uint16_t)(hh << 4) | hl) << 8 | ((lh << 4) | ll);
    }

    void set16bit(int index, uint16_t value)
    {
        data[index] = (value >> 12) & 0x0F;
        data[index + 1] = (value >> 8) & 0x0F;
        data[index + 2] = (value >> 4) & 0x0F;
        data[index + 3] = value & 0x0F;
    }

};

class ICSCtrl
{
public:
    /**
     * @brief Constructor
     * @note Call begin() after construction to initialize UART and GPIO functions.
     */
    ICSCtrl()
        : uartWrite_(nullptr),
          uartRead_(nullptr),
          uartAvailable_(nullptr),
          uartFlush_(nullptr),
          inselSet_(nullptr),
          inited_(false)
    {
    }

public:
    /**
     * @brief Initialize UART and GPIO functions
     * @param uartWrite UART write function pointer
     * @param uartRead UART read function pointer
     * @param uartAvailable UART available function pointer
     * @param uartFlush UART flush function pointer
     * @param inselSet GPIO control function pointer for SEL pin
     */
    void begin(
        ics_uart_write_func_t uartWrite,
        ics_uart_read_func_t uartRead,
        ics_uart_available_func_t uartAvailable,
        ics_uart_flush_func_t uartFlush,
        ics_insel_set_func_t inselSet
    )
    {
        // Validate all function pointers are provided
        if (!uartWrite || !uartRead || !uartAvailable || !uartFlush || !inselSet) {
            inited_ = false;
            return;
        }

        uartWrite_ = uartWrite;
        uartRead_ = uartRead;
        uartAvailable_ = uartAvailable;
        uartFlush_ = uartFlush;
        inselSet_ = inselSet;

        // Initialize SEL pin
        setRxMode(); // Default to RX mode

        // Mark as initialized
        inited_ = true;
    }

public:
    // Wrapper methods for convenient access

    uint16_t setPosition(uint8_t servoId, uint16_t position)
    {
        return cmdPosition(servoId, position);
    }

    uint16_t setFree(uint8_t servoId)
    {
        return cmdPosition(servoId, 0);
    }

    uint16_t getPosition(uint8_t servoId)
    {
        return cmdReadPosition(servoId);
    }

    uint16_t getStretch(uint8_t servoId)
    {
        return cmdRead(servoId, ICS_SC_STRC);
    }

    uint16_t getSpeed(uint8_t servoId)
    {
        return cmdRead(servoId, ICS_SC_SPD);
    }

    uint16_t getCurrent(uint8_t servoId)
    {
        return cmdRead(servoId, ICS_SC_CUR);
    }

    uint16_t getTemperature(uint8_t servoId)
    {
        return cmdRead(servoId, ICS_SC_TMP);
    }

    bool setStretch(uint8_t servoId, uint8_t stretch)
    {
        return (stretch >= ICS_MIN_STRETCH && stretch <= ICS_MAX_STRETCH) && 
               cmdWrite(servoId, ICS_SC_STRC, stretch);
    }

    bool setSpeed(uint8_t servoId, uint8_t speed)
    {
        return (speed >= ICS_MIN_SPEED && speed <= ICS_MAX_SPEED) && 
               cmdWrite(servoId, ICS_SC_SPD, speed);
    }

    bool setCurrentLimit(uint8_t servoId, uint8_t limit)
    {
        return (limit >= ICS_MIN_CURRENT_LIMIT && limit <= ICS_MAX_CURRENT_LIMIT) && 
               cmdWrite(servoId, ICS_SC_CUR, limit);
    }

    bool setTemperatureLimit(uint8_t servoId, uint8_t limit)
    {
        return (limit >= ICS_MIN_TEMP_LIMIT && limit <= ICS_MAX_TEMP_LIMIT) && 
               cmdWrite(servoId, ICS_SC_TMP, limit);
    }

    bool readEEPROM(uint8_t servoId, ICSCtrlEEPROM & eeprom)
    {
        return cmdReadEEPROM(servoId, eeprom.data);
    }

    bool writeEEPROM(uint8_t servoId, ICSCtrlEEPROM & eeprom)
    {
        if (!eeprom.isValid()) {
            ICS_DP__("ICSCtrl::writeEEPROM: Invalid EEPROM data\n");
            return false;
        }
        return cmdWriteEEPROM(servoId, eeprom.data);
    }

public:
    /**
     * @brief Send position command to servo
     * @param servoId Servo ID (0-31)
     * @param position Position value (0-16383)
     * @return Feedback position (0-16383) on success, 0xFFFF on error
     */
    uint16_t cmdPosition(uint8_t servoId, uint16_t position)
    {
        if (!IS_ICS_VALID_ID(servoId)) return 0xFFFF;
        if (position > 16383) return 0xFFFF;

        uint8_t cmd_byte = (ICS_MC_POS | servoId);
        uint8_t pos_h = ((position >> 7) & 0x007F);
        uint8_t pos_l = ((position >> 0) & 0x007F);
        uint8_t txCmd[3] = { cmd_byte, pos_h, pos_l };
        uint8_t rxBuf[3] = {0};  // Receive response: [R_CMD][TCH_H][TCH_L]

        if (!transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf))) {
            return 0xFFFF;
        }

        if (rxBuf[0] != (cmd_byte & 0x7F)) {
            return 0xFFFF;  // R_CMD mismatch
        }

        // Extract and return feedback position
        return ((uint16_t)rxBuf[1] << 7) | rxBuf[2];
    }

    /**
     * @brief Read sub-command data from servo
     * @param servoId Servo ID (0-31)
     * @param sc Sub-command (ICS_SC_*), excluding ICS_SC_EEPROM and ICS_SC_TCH
     * @return Read value (8-bit) on success, 0xFFFF on error or unsupported sub-command
     */
    uint16_t cmdRead(uint8_t servoId, uint8_t sc)
    {
        if (!IS_ICS_VALID_ID(servoId)) return 0xFFFF;
        
        // EEPROM and Position require separate methods
        if (sc == ICS_SC_EEPROM || sc == ICS_SC_TCH) {
            return 0xFFFF;
        }

        uint8_t cmd_byte = (ICS_MC_READ | servoId);        
        uint8_t txCmd[2] = { cmd_byte, sc };    
        uint8_t rxBuf[3] = {0};

        if (!transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) ||
            !verifyCommandResponse(rxBuf, cmd_byte) ||
            rxBuf[1] != sc) {
            return 0xFFFF;
        }

        return rxBuf[2];
    }

    /**
     * @brief Read position (TCH) from servo
     * @param servoId Servo ID (0-31)
     * @return Position value (0-16383) on success, 0xFFFF on error
     */
    uint16_t cmdReadPosition(uint8_t servoId)
    {
        if (!IS_ICS_VALID_ID(servoId)) return 0xFFFF;

        uint8_t cmd_byte = (ICS_MC_READ | servoId);        
        uint8_t txCmd[2] = { cmd_byte, ICS_SC_TCH };    
        uint8_t rxBuf[4] = {0};

        if (!transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) ||
            !verifyCommandResponse(rxBuf, cmd_byte) ||
            rxBuf[1] != ICS_SC_TCH) {
            return 0xFFFF;
        }

        return ((uint16_t)(rxBuf[2] & 0x3F) << 7) | rxBuf[3];
    }

    /**
     * @brief Read EEPROM data from servo
     * @param servoId Servo ID (0-31)
     * @param buffer Buffer to store EEPROM data (must be at least ICS_EEPROM_SIZE + 2 bytes)
     * @return true on success, false on error
     */
    bool cmdReadEEPROM(uint8_t servoId, uint8_t * buffer)
    {
        if (!IS_ICS_VALID_ID(servoId)) return false;

        uint8_t cmd_byte = (ICS_MC_READ | servoId);
        uint8_t txCmd[2] = { cmd_byte, ICS_SC_EEPROM };

        return transfer(txCmd, sizeof(txCmd), buffer, ICS_EEPROM_SIZE + 2, ICS_RX_TIMEOUT_EEPROM_MS) &&
               verifyCommandResponse(buffer, cmd_byte) &&
               buffer[1] == ICS_SC_EEPROM;
    }

    /**
     * @brief Write sub-command data to servo
     * @param servoId Servo ID (0-31)
     * @param sc Sub-command (ICS_SC_*)
     * @param value Value to write (8-bit)
     * @return true on success, false on error
     */
    bool cmdWrite(uint8_t servoId, uint8_t sc, uint8_t value)
    {
        if (!IS_ICS_VALID_ID(servoId)) return false;

        uint8_t cmd_byte = (ICS_MC_WRITE | servoId);
        uint8_t txCmd[3] = { cmd_byte, sc, value };
        uint8_t rxBuf[3] = {0};

        return transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) &&
               verifyCommandResponse(rxBuf, cmd_byte) &&
               rxBuf[1] == sc;
    }

    /**
     * @brief Write EEPROM data to servo
     * @param servoId Servo ID (0-31)
     * @param buffer Buffer with EEPROM data (must be exactly ICS_EEPROM_SIZE bytes)
     * @return true on success, false on error
     */
    bool cmdWriteEEPROM(uint8_t servoId, uint8_t * buffer)
    {
        if (!IS_ICS_VALID_ID(servoId)) return false;

        uint8_t cmd_byte = (ICS_MC_WRITE | servoId);
        uint8_t txCmd[2 + ICS_EEPROM_SIZE] = {0};
        txCmd[0] = cmd_byte;
        txCmd[1] = ICS_SC_EEPROM;
        memcpy(&txCmd[2], buffer, ICS_EEPROM_SIZE);
        uint8_t rxBuf[2] = {0};

        return transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) &&
               verifyCommandResponse(rxBuf, cmd_byte) &&
               rxBuf[1] == ICS_SC_EEPROM;
    }

    /**
     * @brief Read servo ID (broadcast read)
     * @note Uses special bus protocol to identify servo on the bus
     * @return Servo ID (0-31) on success, 0xFF on error or no response
     */
    uint8_t cmdIDRead(void)
    {
        uint8_t cmd_byte = (ICS_MC_ID | 0x1F);
        uint8_t txCmd[4] = { cmd_byte, 0, 0, 0 };
        uint8_t rxBuf[1] = {0};

        if (!transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) ||
            (rxBuf[0] & 0xE0) == 0xE0) {
            return 0xFF;
        }

        return rxBuf[0] & 0x1F;
    }

    /**
     * @brief Write new ID to servo
     * @note Uses special bus protocol to set servo ID
     * @param newId New ID to set (0-31)
     * @return true on success, false on error
     */
    bool cmdIDWrite(uint8_t newId)
    {
        if (!IS_ICS_VALID_ID(newId)) return false;

        uint8_t cmd_byte = (ICS_MC_ID | newId);
        uint8_t txCmd[4] = { cmd_byte, 0x01, 0x01, 0x01 };
        uint8_t rxBuf[1] = {0};

        return transfer(txCmd, sizeof(txCmd), rxBuf, sizeof(rxBuf)) &&
               (rxBuf[0] & 0xE0) != 0xE0;
    }

    /**
     * @brief Update process (call from loop)
     */
    void loop(void)
    {
        // Currently no continuous background processing needed
        // Data is handled in command methods
    }

private:
    /**
     * @brief Verify response command byte
     * @param rxBuf Receive buffer
     * @param cmd_byte Expected command byte
     * @return true if match, false otherwise
     */
    bool verifyCommandResponse(const uint8_t* rxBuf, uint8_t cmd_byte) const
    {
        return rxBuf[0] == (cmd_byte & 0x7F);
    }

    /**
     * @brief Set TX mode (SEL = HIGH)
     */
    void setTxMode(void)
    {
        inselSet_(LOW);
    }

    /**
     * @brief Set RX mode (SEL = LOW)
     */
    void setRxMode(void)
    {
        inselSet_(HIGH);
    }

private:
    ics_uart_write_func_t uartWrite_;
    ics_uart_read_func_t uartRead_;
    ics_uart_available_func_t uartAvailable_;
    ics_uart_flush_func_t uartFlush_;
    ics_insel_set_func_t inselSet_;
    bool inited_;

    /**
     * @brief Transfer data over ICS protocol (TX then RX)
     * @param txData Pointer to TX data buffer
     * @param txLen Number of bytes to send
     * @param rxData Pointer to RX data buffer
     * @param rxLen Number of bytes to receive
     * @param timeoutMs RX response timeout in milliseconds (default: ICS_RX_TIMEOUT_MS)
     * @return true if success
     */
    bool transfer(const uint8_t* txData, int txLen, uint8_t* rxData, int rxLen, unsigned long timeoutMs = ICS_RX_TIMEOUT_MS)
    {
        if (!inited_) {
            return false;
        }

        if (!txData || txLen <= 0 || !rxData || rxLen <= 0) {
            return false;
        }

        // Set TX mode
        setTxMode();

        // Send command
        for (int i = 0; i < txLen; i++) {
            uartWrite_(txData[i]);
        }

        // Ensure all data is sent
        uartFlush_(txLen);

        // Switch to RX mode
        setRxMode();

        // Wait for response with timeout
        unsigned long startTime = millis();
        int rxIdx = 0;

        while (millis() - startTime < timeoutMs) {
            if (uartAvailable_() > 0) {
                rxData[rxIdx] = (uint8_t)uartRead_();
                rxIdx++;
                
                if (rxIdx >= rxLen) {
                    break;
                }
            }
        }

        // Check if we received all expected bytes
        return (rxIdx == rxLen);
    }
};
