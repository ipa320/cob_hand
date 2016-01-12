#ifndef HAND_BRIDGE_GPIO_H_
#define HAND_BRIDGE_GPIO_H_

#include "pigpio.h"

class GPIO {
    bool initialized;
    uint32_t pin_mask0, pin_mask1;
    bool call(int (*func)(uint32_t), uint32_t mask, uint32_t pins){
        return initialized && (pins & mask) == pins && (pins == 0 || func(pins) == 0);
    }
public:
    GPIO() : initialized(false), pin_mask0(0), pin_mask1(0) {}
    bool isInitialized() const { return initialized; }
    bool setInput(uint32_t pin) {
        return initialized && gpioSetMode(pin, PI_INPUT) == 0;
    }
    bool setOutput(uint32_t pin) {
        if(initialized && gpioSetMode(pin, PI_OUTPUT) == 0){
            if(pin < 32) pin_mask0 |= (1<<pin);
            else pin_mask1 |= (1<<(pin-32));
            return true;
        }
        return false;
    }
    bool init(){
        return initialized = gpioInitialise() >= 0;
    }
    bool clearPins0(uint32_t pins){
        return call(gpioWrite_Bits_0_31_Clear, pin_mask0, pins);
    }
    bool setPins0(uint32_t pins){
        return call(gpioWrite_Bits_0_31_Set, pin_mask0, pins);
    }
    bool clearPins1(uint32_t pins){
        return call(gpioWrite_Bits_32_53_Clear, pin_mask1, pins);
    }
    bool setPins1(uint32_t pins){
        return call(gpioWrite_Bits_32_53_Set, pin_mask1, pins);
    }
    bool writePin(uint32_t pin, uint32_t level) {
        if(pin < 32 && ((1 << pin) & pin_mask0) != (1<<pin)) return false;
        if(pin >= 32 && ((1 << (pin-32)) & pin_mask1) != (1<<(pin-32))) return false;
        return gpioWrite(pin, level ? 1 : 0) == 0;
    }
    uint32_t getState0() {
        return initialized ? gpioRead_Bits_0_31(): 0;
    }
    uint32_t getState1() {
        return initialized ? gpioRead_Bits_32_53(): 0;
    }
};
#endif // HAND_BRIDGE_GPIO_H_
