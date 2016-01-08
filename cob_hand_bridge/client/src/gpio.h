#ifndef HAND_BRIDGE_GPIO_H_
#define HAND_BRIDGE_GPIO_H_

#include "pigpio.h"

class GPIO {
    bool initialized;
    uint32_t pin_mask;
public:
    GPIO() : initialized(false) {}
    bool isInitialized() const { return initialized; }
    bool setInput(uint32_t pin) {
        return initialized && gpioSetMode(pin, PI_INPUT) == 0;
    }
    bool setOutput(uint32_t pin) {
        if(initialized && gpioSetMode(pin, PI_OUTPUT) == 0){
            pin_mask |= (1<<pin);
            return true;
        }
        return false;
    }
    bool init(){
        return initialized = gpioInitialise() >= 0;
    }
    bool clearPins(uint32_t pins){
        return initialized && (pins & pin_mask) == pins && (pins == 0 || gpioWrite_Bits_0_31_Clear(pins) == 0);
    }
    bool setPins(uint32_t pins){
        return initialized && (pins & pin_mask) == pins && (pins == 0 || gpioWrite_Bits_0_31_Set(pins) == 0);
    }
    uint32_t getState() {
        return initialized ? gpioRead_Bits_0_31(): 0;
    }
};
#endif // HAND_BRIDGE_GPIO_H_
