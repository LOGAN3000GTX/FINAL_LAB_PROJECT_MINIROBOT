/*
 * DRV8833 motor driver library created by Bench Robotics
 */
#ifndef BR_DRV8833_H
#define BR_DRV8833_H

#include <Arduino.h>

// Decay modes effect how the motor "brakes" between PWM pulses.
// FAST_DECAY: Coasts during off-cycle (Linear, easier to control).
// SLOW_DECAY: Brakes during off-cycle (Better low-speed torque).
enum DecayMode {
    FAST_DECAY,
    SLOW_DECAY
};

class BR_DRV8833 {
  public:
    /**
     * @param pinIN1  Control Pin 1
     * @param pinIN2  Control Pin 2
     * @param pinSleep (Optional) Sleep pin. Pass -1 if hardwired to VCC.
     */
    BR_DRV8833(int pinIN1, int pinIN2, int pinSleep = -1);

    /**
     * Initializes the PWM pins.
     * @param freq PWM Frequency (Default 20kHz for silent operation)
     * @param resolution PWM Resolution (Default 12-bit: 0-4095)
     */
    void begin(uint32_t freq = 20000, uint8_t resolution = 10);

    /**
     * Drive the motor.
     * @param speed Speed from -100 (Full Reverse) to 100 (Full Forward). 0 is Stop.
     */
    void drive(float speed);

    /**
     * Hard brake (Shorts motor windings).
     */
    void brake();

    /**
     * Soft stop (Disconnects motor, lets it coast).
     */
    void coast();

    /**
     * Low power mode. Motors will not move.
     */
    void sleep();
    void wake();

    /**
     * Flips the motor direction via software.
     * Useful if you wired the motor backward.
     */
    void setDirectionFlip(bool flipped);

    /**
     * Set the decay mode.
     * Default is FAST_DECAY (standard PWM). 
     * Use SLOW_DECAY for better torque at very low speeds.
     */
    void setDecayMode(DecayMode mode);

  private:
    int _pinIN1;
    int _pinIN2;
    int _pinSleep;
    
    bool _isFlipped;
    DecayMode _decayMode;
    
    // PWM internal settings
    uint32_t _pwmFreq;
    uint8_t _pwmRes;
    uint32_t _maxDuty; // Calculated based on resolution

    void _setPWM(int pin, uint32_t duty);
};

#endif
