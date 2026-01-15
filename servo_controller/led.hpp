/**********************************************************************/
/**
 * @brief  LED
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
 * Class definitions
 *----------------------------------------------------------------------
 */

class LED
{
public:

    LED(
        int pinLed,
        unsigned long blinkIntervalMs
    ) :
        pinLed_(pinLed),
        blinkIntervalMs_(blinkIntervalMs)
    {
        pinMode(pinLed_, OUTPUT);
    }

    void loop(void)
    {
        unsigned long curMillis = millis();
        if (curMillis - preMillis_ >= 500) {
            preMillis_ = curMillis;
            if (ledDir_) {
                digitalWrite(pinLed_, LOW);
                ledDir_ = false;
            } else {
                digitalWrite(pinLed_, HIGH);
                ledDir_ = true;
            }
        }
    }

private:
    int pinLed_;
    unsigned long blinkIntervalMs_ = 500;

private:
    bool ledDir_ = false;
    unsigned long preMillis_ = 0;
};
