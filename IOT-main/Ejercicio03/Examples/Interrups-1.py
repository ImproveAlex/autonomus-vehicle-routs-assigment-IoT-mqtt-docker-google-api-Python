import RPi.GPIO as GPIO
import time

#variable con el valor en BCM del pin
button_pin = 23

GPIO.setmode(GPIO.BCM)

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

button_state = GPIO.HIGH

try:
    while True:
        GPIO.wait_for_edge(button_pin, GPIO.BOTH)
        if not GPIO.input(button_pin):
            print("You have pressed the Button!")
        else:
            print("You have released the Button!")

except KeyboardInterrupt:
    GPIO.cleanup()