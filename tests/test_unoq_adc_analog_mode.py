"""The UNO Q's analogRead puts its pin back in analog mode before every sample.

The STM32 converts a pin correctly only in analog mode. Zephyr's ADC driver sets that once
at boot, and the battery monitor's pinMode(pin, INPUT) -- the Arduino habit -- then left A0 a
digital input: a real robot's 10k/1k divider read 0.824 V at the pin where a meter showed
1.038, so a 3S pack at 11.41 V was reported as 9.06 V (2026-10-11).
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PINS = os.path.join(ROOT, "firmware", "zephyr", "unoq", "fw", "src", "pins.cpp")


def _analog_read():
    src = open(PINS, encoding="utf-8").read()
    start = src.index("int analogRead(int pin)")
    return src[start:src.index("\n}\n", start)]


def test_every_read_restores_analog_mode_first():
    body = _analog_read()
    restore = body.index("GPIO_DISCONNECTED")
    assert restore < body.index("adc_read_dt"), "the pin must be analog before the sample"
    assert re.search(r"gpio_pin_configure_dt\(&pins\[pin\],\s*GPIO_DISCONNECTED\)", body)


def test_a_timer_pin_keeps_its_pwm():
    # A2/A3 carry motor PWM (pwmIndex); turning them analog would stop the motor.
    assert "pwmIndex(pin) < 0" in _analog_read()
