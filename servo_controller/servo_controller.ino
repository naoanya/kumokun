/**********************************************************************/
/**
 * @brief  Main Controller
 * @author naoa
 */
/**********************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Includes
 *----------------------------------------------------------------------
 */
#include <SoftwareSerial.h>

#include "led.hpp"
#include "interval_timer.hpp"
#include "ics_ctrl.hpp"

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

#define PIN_ICS_UART_TX_0                  (0)
#define PIN_ICS_UART_RX_0                  (1)
#define PIN_ICS_SEL_0                      (2)
#define PIN_ICS_UART_TX_1                  (4)
#define PIN_ICS_UART_RX_1                  (5)
#define PIN_ICS_SEL_1                      (6)
#define PIN_ICS_UART_TX_2                  (8)
#define PIN_ICS_UART_RX_2                  (9)
#define PIN_ICS_SEL_2                      (10)

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

#define ENABLE_SETUP_SERIAL_HOST_WAIT   (1)
#define SETUP_SERIAL_HOST_WAIT_MS       (1000) // startup wait after serial begin for host pc connection

#define USB_SERIAL                      Serial
#define UART_SERIAL                     Serial1

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

static void cmd_process(void);
static void main_process(void);

static void ics_0_uart_write(uint8_t data);
static int  ics_0_uart_read(void);
static int  ics_0_uart_available(void);
static void ics_0_uart_flush(int txLen);
static void ics_0_insel_set(int value);

static void ics_1_uart_write(uint8_t data);
static int  ics_1_uart_read(void);
static int  ics_1_uart_available(void);
static void ics_1_uart_flush(int txLen);
static void ics_1_insel_set(int value);

static void ics_2_uart_write(uint8_t data);
static int  ics_2_uart_read(void);
static int  ics_2_uart_available(void);
static void ics_2_uart_flush(int txLen);
static void ics_2_insel_set(int value);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Static Variables
 *----------------------------------------------------------------------
 */

static LED onBoardLed_(25, 500);
static IntervalTimer debugTimer_;
static ICSCtrl icsCtrl0_;
static ICSCtrl icsCtrl1_;
static ICSCtrl icsCtrl2_;
static ICSCtrlEEPROM icsCtrlEEPROM_;

static uint32_t fpschecker_ = 0;

static SoftwareSerial mySerial0(PIN_ICS_UART_RX_0, PIN_ICS_UART_TX_0);
static SoftwareSerial mySerial1(PIN_ICS_UART_RX_1, PIN_ICS_UART_TX_1);
static SoftwareSerial mySerial2(PIN_ICS_UART_RX_2, PIN_ICS_UART_TX_2);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Core0
 *----------------------------------------------------------------------
 */

void setup()
{
  //////////////////////////
  // Setup SysClock

  //set_sys_clock_khz(144000, true);

  //////////////////////////
  // Setup debug serial
  USB_SERIAL.begin(115200);
  #if ENABLE_SETUP_SERIAL_HOST_WAIT
  delay(SETUP_SERIAL_HOST_WAIT_MS);
  #endif
  USB_SERIAL.printf("START SETUP\n");
  
  //////////////////////////
  // Setup modules
  
  debugTimer_.setIntervalMs(1000);

  // Setup GPIO for ICS controller
  pinMode(PIN_ICS_SEL_0, OUTPUT);
  pinMode(PIN_ICS_SEL_1, OUTPUT);
  pinMode(PIN_ICS_SEL_2, OUTPUT);
  digitalWrite(PIN_ICS_SEL_0, HIGH);
  digitalWrite(PIN_ICS_SEL_1, HIGH);
  digitalWrite(PIN_ICS_SEL_2, HIGH);

  // Setup UART for ICS controller
  mySerial0.begin(115200, SERIAL_8E1);
  mySerial1.begin(115200, SERIAL_8E1);
  mySerial2.begin(115200, SERIAL_8E1);

  delay(10);

  // Setup ICS controller
  icsCtrl0_.begin(
      ics_0_uart_write,
      ics_0_uart_read,
      ics_0_uart_available,
      ics_0_uart_flush,
      ics_0_insel_set
  );
  icsCtrl1_.begin(
      ics_1_uart_write,
      ics_1_uart_read,
      ics_1_uart_available,
      ics_1_uart_flush,
      ics_1_insel_set
  );
  icsCtrl2_.begin(
      ics_2_uart_write,
      ics_2_uart_read,
      ics_2_uart_available,
      ics_2_uart_flush,
      ics_2_insel_set
  );

  USB_SERIAL.printf("ICS Controller initialized\n");
}

void loop()
{
  //////////////////////////
  // Main Process

  cmd_process();

  main_process();

  //////////////////////////
  // Debug

  #if 0
  if (debugTimer_.check()) {
    //USB_SERIAL.printf("Core Temp = %2.1f C\n", analogReadTemp());
    USB_SERIAL.printf("fps = %d\n", fpschecker_);
    fpschecker_ = 0;
  }
  #endif

  #if 1
  onBoardLed_.loop();
  #endif
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Core1
 *----------------------------------------------------------------------
 */

void setup1()
{
#if ENABLE_SETUP_SERIAL_HOST_WAIT
  delay(SETUP_SERIAL_HOST_WAIT_MS);
#endif
}

void loop1()
{

}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions
 *----------------------------------------------------------------------
 */

static char cmd_buffer_[1024];
static int  cmd_buf_i_ = 0;
static int  cmd_state_ = 0;

// Helper: parse id and id+value
static bool parseId(const char* cmd, int offset, int &id)
{
    return (sscanf(cmd + offset, "%d", &id) == 1);
}

static bool parseIdVal(const char* cmd, int offset, int &id, int &val)
{
    return (sscanf(cmd + offset, "%d %d", &id, &val) == 2);
}

// Generic GET handler for single-byte sub-commands
static void handleGetParam(const char* cmd, int offset, uint8_t sc, const char* jsonCmd)
{
    int id;
    if (!parseId(cmd, offset, id)) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"invalid_parameters\"}\n", jsonCmd);
        return;
    }

    ICSCtrl *ctrl = getCtrlFromId(id);
    if (!ctrl) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"invalid_parameters\"}\n", jsonCmd);
        return;
    }

    uint16_t tmp = ctrl->cmdRead(id, sc);
    if (tmp != 0xFFFF) {
        uint8_t val = (uint8_t)tmp;
        USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"%s\",\"id\":%d,\"val\":%d}\n", jsonCmd, id, val);
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"transmission_failed\"}\n", jsonCmd);
    }
}

// Generic SET handler with range check
static void handleSetParamRange(const char* cmd, int offset, uint8_t sc, int minVal, int maxVal, const char* jsonCmd)
{
    int id, val;
    if (!parseIdVal(cmd, offset, id, val)) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"invalid_parameters\"}\n", jsonCmd);
        return;
    }

    ICSCtrl *ctrl = getCtrlFromId(id);
    if (!ctrl) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"invalid_parameters\"}\n", jsonCmd);
        return;
    }

    if (val < minVal || val > maxVal) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"out_of_range\",\"min\":%d,\"max\":%d}\n", jsonCmd, minVal, maxVal);
        return;
    }

    if (!ctrl->cmdWrite(id, sc, (uint8_t)val)) {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"transmission_failed\"}\n", jsonCmd);
    }
}

static void handle_as(const char* cmd)
{
    int positions[18];
    int count = 0;
    const char* p = cmd + 2;
    char* end;

    for (int i = 0; i < 18; i++) {
        long val = strtol(p, &end, 10);
        if (p == end) break;
        positions[i] = (int)val;
        count++;
        p = end;
    }

    if (count == 18) {
        for (int i = 0; i < 18; i++) {
            int id = i + 1;
            ICSCtrl *ctrl = getCtrlFromId(id);
            if (ctrl) {
                uint16_t feedbackPos = ctrl->setPosition(id, positions[i]);
                if (feedbackPos != 0xFFFF) {
                    USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", id, feedbackPos);
                } else {
                    USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"transmission_failed\"}\n");
                }
            }
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"AS\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_ag(const char* cmd)
{
    for (int i = 1; i <= 18; i++) {
        ICSCtrl *ctrl = getCtrlFromId(i);
        if (ctrl) {
            uint16_t pos = ctrl->getPosition(i);
            if (pos != 0xFFFF) {
                USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", i, pos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"transmission_failed\"}\n");
            }
        }
    }
}

static void handle_af(const char* cmd)
{
    for (int i = 1; i <= 18; i++) {
        ICSCtrl *ctrl = getCtrlFromId(i);
        if (ctrl) {
            uint16_t feedbackPos = ctrl->setFree(i);
            if (feedbackPos != 0xFFFF) {
                USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", i, feedbackPos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"transmission_failed\"}\n");
            }
        }
    }
}

static void handle_xs(const char* cmd)
{
    int id, pos;
    if (sscanf(cmd + 2, "%d %d", &id, &pos) == 2) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t feedbackPos = ctrl->setPosition(id, pos);
            if (feedbackPos != 0xFFFF) {
                // Reduced JSON for XS command
                USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", id, feedbackPos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_xg(const char* cmd)
{
    int id;
    if (sscanf(cmd + 2, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t pos = ctrl->getPosition(id);
            if (pos != 0xFFFF) {
                // Reduced JSON for XG command
                USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", id, pos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_xf(const char* cmd)
{
    int id;
    if (sscanf(cmd + 2, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t feedbackPos = ctrl->setFree(id);
            if (feedbackPos != 0xFFFF) {
                // Reduced JSON for XF command
                USB_SERIAL.printf("{\"id\":%d,\"pos\":%d}\n", id, feedbackPos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_setpos(const char* cmd)
{
    int id, pos;
    if (sscanf(cmd + 6, "%d %d", &id, &pos) == 2) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t feedbackPos = ctrl->setPosition(id, pos);
            if (feedbackPos != 0xFFFF) {
                USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"SETPOS\",\"id\":%d,\"pos\":%d,\"feedback\":%d}\n", id, pos, feedbackPos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"SETPOS\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_getpos(const char* cmd)
{
    int id;
    if (sscanf(cmd + 6, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t pos = ctrl->getPosition(id);
            if (pos != 0xFFFF) {
                USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"GETPOS\",\"id\":%d,\"pos\":%d}\n", id, pos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"GETPOS\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_free(const char* cmd)
{
    int id;
    if (sscanf(cmd + 4, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            uint16_t feedbackPos = ctrl->setFree(id);
            if (feedbackPos != 0xFFFF) {
                USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"FREE\",\"id\":%d,\"feedback\":%d}\n", id, feedbackPos);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"FREE\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_read_eeprom(const char* cmd)
{
    int id;
    if (sscanf(cmd + 9, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            if (ctrl->readEEPROM(id, icsCtrlEEPROM_)) {
                USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"READEPROM\",\"id\":%d}\n", id);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"READEPROM\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"READEPROM\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"READEPROM\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_write_eeprom(const char* cmd)
{
    int id;
    if (sscanf(cmd + 10, "%d", &id) == 1) {
        ICSCtrl *ctrl = getCtrlFromId(id);
        if (ctrl) {
            if (ctrl->writeEEPROM(id, icsCtrlEEPROM_)) {
                // USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"WRITEEPROM\",\"id\":%d}\n", id);
            } else {
                USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"WRITEEPROM\",\"reason\":\"transmission_failed\"}\n");
            }
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"WRITEEPROM\",\"reason\":\"invalid_parameters\"}\n");
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"WRITEEPROM\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_eget(const char* cmd)
{
    char field[32] = {0};
    if (sscanf(cmd + 4, "%31s", field) == 1) {
        uint8_t uval8;
        uint16_t uval16;
        int8_t sval8;
        bool bval;
        
        if (strcmp(field, "stretch") == 0) {
            uval8 = icsCtrlEEPROM_.getStretch();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"stretch\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "speed") == 0) {
            uval8 = icsCtrlEEPROM_.getSpeed();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"speed\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "punch") == 0) {
            uval8 = icsCtrlEEPROM_.getPunch();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"punch\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "deadband") == 0) {
            uval8 = icsCtrlEEPROM_.getDeadband();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"deadband\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "damping") == 0) {
            uval8 = icsCtrlEEPROM_.getDamping();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"damping\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "protection") == 0) {
            uval8 = icsCtrlEEPROM_.getProtectionTimer();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"protection\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "rotmode") == 0) {
            bval = icsCtrlEEPROM_.getFlagRotationMode();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"rotmode\",\"value\":%d}\n", bval ? 1 : 0);
        } else if (strcmp(field, "slave") == 0) {
            bval = icsCtrlEEPROM_.getFlagSlave();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"slave\",\"value\":%d}\n", bval ? 1 : 0);
        } else if (strcmp(field, "reverse") == 0) {
            bval = icsCtrlEEPROM_.getFlagReverse();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"reverse\",\"value\":%d}\n", bval ? 1 : 0);
        } else if (strcmp(field, "free") == 0) {
            bval = icsCtrlEEPROM_.getFlagFree();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"free\",\"value\":%d}\n", bval ? 1 : 0);
        } else if (strcmp(field, "pwminh") == 0) {
            bval = icsCtrlEEPROM_.getFlagPWMINH();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"pwminh\",\"value\":%d}\n", bval ? 1 : 0);
        } else if (strcmp(field, "pullupper") == 0) {
            uval16 = icsCtrlEEPROM_.getPulseLimitUpper();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"pullupper\",\"value\":%d}\n", uval16);
        } else if (strcmp(field, "pulllower") == 0) {
            uval16 = icsCtrlEEPROM_.getPulseLimitLower();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"pulllower\",\"value\":%d}\n", uval16);
        } else if (strcmp(field, "baudrate") == 0) {
            uval8 = icsCtrlEEPROM_.getBaudRate();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"baudrate\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "templimit") == 0) {
            uval8 = icsCtrlEEPROM_.getTemperatureLimit();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"templimit\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "currentlimit") == 0) {
            uval8 = icsCtrlEEPROM_.getCurrentLimit();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"currentlimit\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "response") == 0) {
            uval8 = icsCtrlEEPROM_.getResponse();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"response\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "useroffset") == 0) {
            sval8 = icsCtrlEEPROM_.getUserOffset();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"useroffset\",\"value\":%d}\n", sval8);
        } else if (strcmp(field, "id") == 0) {
            uval8 = icsCtrlEEPROM_.getID();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"id\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "charstretch1") == 0) {
            uval8 = icsCtrlEEPROM_.getCharacteristicStretch1();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"charstretch1\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "charstretch2") == 0) {
            uval8 = icsCtrlEEPROM_.getCharacteristicStretch2();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"charstretch2\",\"value\":%d}\n", uval8);
        } else if (strcmp(field, "charstretch3") == 0) {
            uval8 = icsCtrlEEPROM_.getCharacteristicStretch3();
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"EGET\",\"field\":\"charstretch3\",\"value\":%d}\n", uval8);
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"EGET\",\"reason\":\"unknown_field\",\"field\":\"%s\"}\n", field);
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"EGET\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_eset(const char* cmd)
{
    char field[32] = {0};
    int ival;
    if (sscanf(cmd + 4, "%31s %d", field, &ival) == 2) {
        if (strcmp(field, "stretch") == 0) {
            icsCtrlEEPROM_.setStretch((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"stretch\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "speed") == 0) {
            icsCtrlEEPROM_.setSpeed((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"speed\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "punch") == 0) {
            icsCtrlEEPROM_.setPunch((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"punch\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "deadband") == 0) {
            icsCtrlEEPROM_.setDeadband((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"deadband\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "damping") == 0) {
            icsCtrlEEPROM_.setDamping((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"damping\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "protection") == 0) {
            icsCtrlEEPROM_.setProtectionTimer((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"protection\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "rotmode") == 0) {
            icsCtrlEEPROM_.setFlagRotationMode((bool)(ival != 0));
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"rotmode\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "slave") == 0) {
            icsCtrlEEPROM_.setFlagSlave((bool)(ival != 0));
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"slave\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "reverse") == 0) {
            icsCtrlEEPROM_.setFlagReverse((bool)(ival != 0));
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"reverse\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "free") == 0) {
            icsCtrlEEPROM_.setFlagFree((bool)(ival != 0));
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"free\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "pwminh") == 0) {
            icsCtrlEEPROM_.setFlagPWMINH((bool)(ival != 0));
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"pwminh\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "pullupper") == 0) {
            icsCtrlEEPROM_.setPulseLimitUpper((uint16_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"pullupper\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "pulllower") == 0) {
            icsCtrlEEPROM_.setPulseLimitLower((uint16_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"pulllower\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "baudrate") == 0) {
            icsCtrlEEPROM_.setBaudRate((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"baudrate\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "templimit") == 0) {
            icsCtrlEEPROM_.setTemperatureLimit((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"templimit\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "currentlimit") == 0) {
            icsCtrlEEPROM_.setCurrentLimit((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"currentlimit\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "response") == 0) {
            icsCtrlEEPROM_.setResponse((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"response\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "useroffset") == 0) {
            icsCtrlEEPROM_.setUserOffset((int8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"useroffset\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "id") == 0) {
            icsCtrlEEPROM_.setID((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"id\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "charstretch1") == 0) {
            icsCtrlEEPROM_.setCharacteristicStretch1((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"charstretch1\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "charstretch2") == 0) {
            icsCtrlEEPROM_.setCharacteristicStretch2((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"charstretch2\",\"value\":%d}\n", ival);
        } else if (strcmp(field, "charstretch3") == 0) {
            icsCtrlEEPROM_.setCharacteristicStretch3((uint8_t)ival);
            USB_SERIAL.printf("{\"status\":\"OK\",\"command\":\"ESET\",\"field\":\"charstretch3\",\"value\":%d}\n", ival);
        } else {
            USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"ESET\",\"reason\":\"unknown_field\",\"field\":\"%s\"}\n", field);
        }
    } else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"ESET\",\"reason\":\"invalid_parameters\"}\n");
    }
}

static void handle_dump(void)
{
    USB_SERIAL.printf("EEPROM Dump:\n");
    icsCtrlEEPROM_.dump();
}

static void handle_help(void)
{
    USB_SERIAL.printf("=== ICS Servo Controller Commands ===\n");
    USB_SERIAL.printf("Ports: icsCtrl0 -> IDs 1-6, icsCtrl1 -> IDs 7-12, icsCtrl2 -> IDs 13-18\n");
    USB_SERIAL.printf("Position Control:\n");
    USB_SERIAL.printf("  SETPOS <id> <pos>      - Set position (0-16383)\n");
    USB_SERIAL.printf("  GETPOS <id>            - Get current position\n");
    USB_SERIAL.printf("  FREE <id>              - Release servo (set position to 0)\n");
    USB_SERIAL.printf("Parameter Control:\n");
    USB_SERIAL.printf("  STRETCH <id> <val>     - Set stretch (range: %d-%d)\n", ICS_MIN_STRETCH, ICS_MAX_STRETCH);
    USB_SERIAL.printf("  GSTRETCH <id>          - Get stretch\n");
    USB_SERIAL.printf("  SPEED <id> <val>       - Set speed (range: %d-%d)\n", ICS_MIN_SPEED, ICS_MAX_SPEED);
    USB_SERIAL.printf("  GSPEED <id>            - Get speed\n");
    USB_SERIAL.printf("  CURRENT <id> <val>     - Set current limit (range: %d-%d)\n", ICS_MIN_CURRENT_LIMIT, ICS_MAX_CURRENT_LIMIT);
    USB_SERIAL.printf("  GCURRENT <id>          - Get current\n");
    USB_SERIAL.printf("  TEMP <id> <val>        - Set temperature limit (range: %d-%d)\n", ICS_MIN_TEMP_LIMIT, ICS_MAX_TEMP_LIMIT);
    USB_SERIAL.printf("  GTEMP <id>             - Get temperature\n");
    USB_SERIAL.printf("EEPROM Control:\n");
    USB_SERIAL.printf("  READEPROM <id>         - Read EEPROM from servo\n");
    USB_SERIAL.printf("  WRITEEPROM <id>        - Write EEPROM to servo\n");
    USB_SERIAL.printf("  EGET <field>           - Get EEPROM parameter\n");
    USB_SERIAL.printf("  ESET <field> <val>     - Set EEPROM parameter\n");
    USB_SERIAL.printf("Misc:\n");
    USB_SERIAL.printf("  DUMP                   - Display EEPROM data (hex dump)\n");
    USB_SERIAL.printf("  HELP                   - Show this help\n");
    USB_SERIAL.printf("  EHELP                  - Show EEPROM field names\n");
}

static void handle_ehelp(void)
{
    USB_SERIAL.printf("=== EEPROM Field Names ===\n");
    USB_SERIAL.printf("Parameters (8-bit):\n");
    USB_SERIAL.printf("  stretch, speed, punch, deadband, damping, protection\n");
    USB_SERIAL.printf("  templimit, currentlimit, response, useroffset, id\n");
    USB_SERIAL.printf("Parameters (16-bit):\n");
    USB_SERIAL.printf("  pullupper, pulllower\n");
    USB_SERIAL.printf("Characteristic Stretch:\n");
    USB_SERIAL.printf("  charstretch1, charstretch2, charstretch3\n");
    USB_SERIAL.printf("Flags (boolean, 0=OFF/1=ON):\n");
    USB_SERIAL.printf("  rotmode    - Rotation mode\n");
    USB_SERIAL.printf("  slave      - Slave mode\n");
    USB_SERIAL.printf("  reverse    - Reverse direction\n");
    USB_SERIAL.printf("  free       - Free mode (read-only)\n");
    USB_SERIAL.printf("  pwminh     - PWM inhibit (must be ON for serial)\n");
    USB_SERIAL.printf("Special:\n");
    USB_SERIAL.printf("  baudrate   - Baud rate (0x00=1.25M, 0x01=625K, 0x0A=115.2K)\n");
}

static void cmd_process(void)
{
    if (USB_SERIAL.available() == 0) return;
    
    char c = USB_SERIAL.read();
    
    // Handle newline / carriage return
    if (c == '\n' || c == '\r') {
        if (cmd_buf_i_ > 0) {
            cmd_buffer_[cmd_buf_i_] = '\0';
            cmd_execute(cmd_buffer_, cmd_buf_i_);
            cmd_buf_i_ = 0;
        }
        return;
    }
    
    // Store character in buffer
    if (cmd_buf_i_ < (int)sizeof(cmd_buffer_) - 1) {
        cmd_buffer_[cmd_buf_i_++] = c;
    }
}

/**
 * @brief Parse and execute command
 * @param cmd Command string
 * @param len Command length
 */
static void cmd_execute(const char* cmd, int len)
{
    if (len == 0) return;
    
    if (strncmp(cmd, "AS", 2) == 0) {
        handle_as(cmd);
    }
    else if (strncmp(cmd, "AG", 2) == 0) {
        handle_ag(cmd);
    }
    else if (strncmp(cmd, "AF", 2) == 0) {
        handle_af(cmd);
    }
    else if (strncmp(cmd, "XS", 2) == 0) {
        handle_xs(cmd);
    }
    else if (strncmp(cmd, "XG", 2) == 0) {
        handle_xg(cmd);
    }
    else if (strncmp(cmd, "XF", 2) == 0) {
        handle_xf(cmd);
    }
    else if (strncmp(cmd, "SETPOS", 6) == 0) {
        handle_setpos(cmd);
    }
    else if (strncmp(cmd, "GETPOS", 6) == 0) {
        handle_getpos(cmd);
    }
    else if (strncmp(cmd, "FREE", 4) == 0) {
        handle_free(cmd);
    }
    else if (strncmp(cmd, "STRETCH", 7) == 0) {
        handleSetParamRange(cmd, 7, ICS_SC_STRC, ICS_MIN_STRETCH, ICS_MAX_STRETCH, "STRETCH");
    }
    else if (strncmp(cmd, "SPEED", 5) == 0) {
        handleSetParamRange(cmd, 5, ICS_SC_SPD, ICS_MIN_SPEED, ICS_MAX_SPEED, "SPEED");
    }
    else if (strncmp(cmd, "CURRENT", 7) == 0) {
        handleSetParamRange(cmd, 7, ICS_SC_CUR, ICS_MIN_CURRENT_LIMIT, ICS_MAX_CURRENT_LIMIT, "CURRENT");
    }
    else if (strncmp(cmd, "TEMP", 4) == 0) {
        handleSetParamRange(cmd, 4, ICS_SC_TMP, ICS_MIN_TEMP_LIMIT, ICS_MAX_TEMP_LIMIT, "TEMP");
    }
    else if (strncmp(cmd, "GSTRETCH", 8) == 0) {
        handleGetParam(cmd, 8, ICS_SC_STRC, "GSTRETCH");
    }
    else if (strncmp(cmd, "GSPEED", 6) == 0) {
        handleGetParam(cmd, 6, ICS_SC_SPD, "GSPEED");
    }
    else if (strncmp(cmd, "GCURRENT", 8) == 0) {
        handleGetParam(cmd, 8, ICS_SC_CUR, "GCURRENT");
    }
    else if (strncmp(cmd, "GTEMP", 5) == 0) {
        handleGetParam(cmd, 5, ICS_SC_TMP, "GTEMP");
    }
    else if (strncmp(cmd, "READEPROM", 9) == 0) {
        handle_read_eeprom(cmd);
    }
    else if (strncmp(cmd, "WRITEEPROM", 10) == 0) {
        handle_write_eeprom(cmd);
    }
    else if (strncmp(cmd, "EGET", 4) == 0) {
        handle_eget(cmd);
    }
    else if (strncmp(cmd, "ESET", 4) == 0) {
        handle_eset(cmd);
    }
    else if (strncmp(cmd, "DUMP", 4) == 0) {
        handle_dump();
    }
    else if (strncmp(cmd, "HELP", 4) == 0) {
        handle_help();
    }
    else if (strncmp(cmd, "EHELP", 5) == 0) {
        handle_ehelp();
    }
    else {
        USB_SERIAL.printf("{\"status\":\"NG\",\"command\":\"%s\",\"reason\":\"unknown_command\"}\n", cmd);
    }
}

static ICSCtrl* getCtrlFromId(int id)
{
    if (id >= 1 && id <= 6) return &icsCtrl0_;
    if (id >= 7 && id <= 12) return &icsCtrl1_;
    if (id >= 13 && id <= 18) return &icsCtrl2_;
    return nullptr;
}

static void main_process(void)
{
    // Update ICS controller
    icsCtrl0_.loop();
    icsCtrl1_.loop();
    icsCtrl2_.loop();
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * UART Wrapper Functions for ICSCtrl
 *----------------------------------------------------------------------
 */

static void fixed_uart_delay(int txLen)
{
    constexpr uint32_t delay_us = ((1000000 * 11) / 115200);
    delayMicroseconds(delay_us * txLen + 10);
}

static void ics_0_uart_write(uint8_t data) { mySerial0.write(data); }
static int ics_0_uart_read(void) { if (mySerial0.available() > 0) { return mySerial0.read(); } return -1; }
static int ics_0_uart_available(void) { return mySerial0.available(); }
static void ics_0_uart_flush(int txLen) { /* mySerial0.flush(); */ fixed_uart_delay(txLen); }
static void ics_0_insel_set(int value) { digitalWrite(PIN_ICS_SEL_0, value); }

static void ics_1_uart_write(uint8_t data) { mySerial1.write(data); }
static int ics_1_uart_read(void) { if (mySerial1.available() > 0) { return mySerial1.read(); } return -1; }
static int ics_1_uart_available(void) { return mySerial1.available(); }
static void ics_1_uart_flush(int txLen) { /* mySerial1.flush(); */ fixed_uart_delay(txLen); }
static void ics_1_insel_set(int value) { digitalWrite(PIN_ICS_SEL_1, value); }

static void ics_2_uart_write(uint8_t data) { mySerial2.write(data); }
static int ics_2_uart_read(void) { if (mySerial2.available() > 0) { return mySerial2.read(); } return -1; }
static int ics_2_uart_available(void) { return mySerial2.available(); }
static void ics_2_uart_flush(int txLen) { /* mySerial2.flush(); */ fixed_uart_delay(txLen); }
static void ics_2_insel_set(int value) { digitalWrite(PIN_ICS_SEL_2, value); }
