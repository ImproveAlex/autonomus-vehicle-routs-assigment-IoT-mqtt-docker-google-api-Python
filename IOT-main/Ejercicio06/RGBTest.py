import time
import RPi.GPIO as GPIO
from Session06 import setup_luces, yellow, red, white, turnOff

"""
NOTA PARA EL PROFESOR:

Solo funciona una de las luces RGB, fisicamente una vino dañada.
Se probó mover la luz RGB que funciona a ambos lados de la configuración
y funciona correctamente, es por ello que el código es valido y la
configuracion también. En el Video se evidencia esto.
"""

def main():
    # Set up the GPIO pins and PWM objects
    setup_luces()

    # Perform the RGB test sequence
    yellow(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
    yellow(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    time.sleep(3)
    red(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
    red(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    time.sleep(3)
    white(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
    white(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    time.sleep(3)
    turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
    turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)