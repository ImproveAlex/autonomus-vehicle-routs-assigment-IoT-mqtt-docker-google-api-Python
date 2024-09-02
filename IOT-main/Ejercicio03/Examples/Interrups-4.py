import RPi.GPIO as GPIO
import time
import signal
import sys

#variable con el valor en BCM del pin
button_pin = 23
led_pin = 20

GPIO.setmode(GPIO.BCM)

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_event_callback(channel):
    
    # Verificar el estado actual del pin del botón
    if not GPIO.input(button_pin):
        print("You've pressed the Button!")
        GPIO.output(led_pin, GPIO.HIGH)
    else:
        print("You've released the Button!")
        GPIO.output(led_pin, GPIO.LOW)

try:
    if __name__ == '__main__':
        GPIO.add_event_detect(button_pin, GPIO.BOTH,
                                callback=button_event_callback, bouncetime=25)

        signal.signal(signal.SIGINT, signal_handler)
        signal.pause()

except KeyboardInterrupt:
    GPIO.cleanup()
