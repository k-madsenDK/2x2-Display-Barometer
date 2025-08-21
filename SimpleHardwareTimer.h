/**
 * @file SimpleHardwareTimer.h
 * @brief Wrapper for Ticker hardware timer, for enkel brug.
 *
 * Brug:
 *   SimpleHardwareTimer timer;
 *   timer.setInterval(1000, myCallback);
 */
#pragma once
#include <Ticker.h>

class SimpleHardwareTimer {
public:
    /**
     * @brief Sæt hardware-interval og callback.
     * @param interval_ms Interval i millisekunder
     * @param cb Callback-funktion
     * SimpleHardwareTimer *fifoTimer = new SimpleHardwareTimer;
     * fifoTimer->setInterval(10, fifoTimerCallback);
     */
    void setInterval(uint32_t interval_ms, void (*cb)()) {
        ticker.attach_ms(interval_ms, cb);
    }
        /**
     * @brief Stopper hardware-timeren.
     */
    void stop() {
        ticker.detach();
    }
private:
    Ticker ticker;
};
