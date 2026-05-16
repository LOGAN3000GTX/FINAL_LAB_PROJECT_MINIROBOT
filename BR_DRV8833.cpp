#include "BR_DRV8833.h"

BR_DRV8833::BR_DRV8833(int pinIN1, int pinIN2, int pinSleep) {
    _pinIN1 = pinIN1;
    _pinIN2 = pinIN2;
    _pinSleep = pinSleep;
    _isFlipped = false;
    _decayMode = FAST_DECAY; // Default to standard behavior
}

void BR_DRV8833::begin(uint32_t freq, uint8_t resolution) {
    _pwmFreq = freq;
    _pwmRes = resolution;
    _maxDuty = (1 << resolution) - 1; // e.g., 12-bit = 4095

    // Setup pins for ESP32 Core 3.0
    // ledcAttach(pin, freq, resolution) replaces ledcSetup + ledcAttachPin
    ledcAttach(_pinIN1, _pwmFreq, _pwmRes);
    ledcAttach(_pinIN2, _pwmFreq, _pwmRes);

    if (_pinSleep != -1) {
        pinMode(_pinSleep, OUTPUT);
        digitalWrite(_pinSleep, HIGH); // Enable driver by default
    }
}

void BR_DRV8833::drive(float speed) {
    // Constraint input to -100 to 100
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // Handle direction flipping
    if (_isFlipped) {
        speed = -speed;
    }

    // Determine direction
    bool forward = (speed > 0);
    
    // Convert 0-100 speed to PWM duty cycle
    float absSpeed = abs(speed);
    uint32_t duty = (uint32_t)((absSpeed / 100.0) * _maxDuty);

    if (speed == 0) {
        coast();
        return;
    }

    if (_decayMode == FAST_DECAY) {
        // FAST DECAY: PWM one pin, hold other LOW
        if (forward) {
            _setPWM(_pinIN1, duty);
            _setPWM(_pinIN2, 0);
        } else {
            _setPWM(_pinIN1, 0);
            _setPWM(_pinIN2, duty);
        }
    } else {
        // SLOW DECAY: PWM one pin, hold other HIGH
        // Logic is inverted: 0 duty = Full Speed, Max duty = Stop
        uint32_t invDuty = _maxDuty - duty;
        
        if (forward) {
            _setPWM(_pinIN1, _maxDuty); // High
            _setPWM(_pinIN2, invDuty);  // Pulsing Low
        } else {
            _setPWM(_pinIN1, invDuty);  // Pulsing Low
            _setPWM(_pinIN2, _maxDuty); // High
        }
    }
}

void BR_DRV8833::brake() {
    // To brake, set both inputs HIGH (Slow decay / Short brake)
    _setPWM(_pinIN1, _maxDuty);
    _setPWM(_pinIN2, _maxDuty);
}

void BR_DRV8833::coast() {
    // To coast, set both inputs LOW (Fast decay)
    _setPWM(_pinIN1, 0);
    _setPWM(_pinIN2, 0);
}

void BR_DRV8833::sleep() {
    if (_pinSleep != -1) {
        digitalWrite(_pinSleep, LOW);
    }
}

void BR_DRV8833::wake() {
    if (_pinSleep != -1) {
        digitalWrite(_pinSleep, HIGH);
    }
}

void BR_DRV8833::setDirectionFlip(bool flipped) {
    _isFlipped = flipped;
}

void BR_DRV8833::setDecayMode(DecayMode mode) {
    _decayMode = mode;
}

// Helper for ESP32 Core 3.0 writing
void BR_DRV8833::_setPWM(int pin, uint32_t duty) {
    ledcWrite(pin, duty);
}
