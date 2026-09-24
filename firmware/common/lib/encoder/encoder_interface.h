// A common face for the real and the simulated wheel.
//
// `Encoder` is vendored upstream code (three platform variants of it, in
// encoder.h) and SimEncoder is ours, and the two were selected with
// `#define ENCODER SimEncoder` at compile time. That is the last sensor
// decision that forced a bench board and a real robot to carry different
// firmware.
//
// Rather than edit upstream, RealEncoder wraps it. The virtual call costs
// nothing that matters: it happens four times per control cycle at 50 Hz.
#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

class EncoderInterface
{
    public:
        virtual ~EncoderInterface() {}

        virtual float getRPM() = 0;

        // Only the simulated wheel needs this: it closes the loop in software
        // by integrating the commanded PWM. A real encoder is driven by the
        // physical wheel and has nothing to be fed, so the default does
        // nothing and the control loop can call it unconditionally instead of
        // being bracketed by #ifdef USE_SIM_WHEEL.
        virtual void feed(int pwm) { (void)pwm; }
};

#endif // ENCODER_INTERFACE_H
