/**********************************************************************/
/**
 * @brief  Interval Timer
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

class IntervalTimer
{
public:
    explicit IntervalTimer()
    {
        reset();
    }
    explicit IntervalTimer(unsigned long intervalMs)
    {
        reset();
        setIntervalMs(intervalMs);
    }
    
    void setIntervalMs(unsigned long intervalMs)
    {
        intervalMs_ = intervalMs;
    }

    void reset(void)
    {
        preMs_ = millis();
    }

    bool check(void)
    {
        unsigned long curMs_ = millis();
        if (curMs_ - preMs_ >= intervalMs_) {
            preMs_ = curMs_;
            return true;
        } else {
            return false;
        }
    }
    
private:
    unsigned long intervalMs_;
    unsigned long preMs_;
};
