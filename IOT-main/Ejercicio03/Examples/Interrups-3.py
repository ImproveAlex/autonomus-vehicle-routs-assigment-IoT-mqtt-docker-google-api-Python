import RPi.GPIO as GPIO
import time
import signal
import sys

#variable con el valor en BCM del pin
button_pin = 23

GPIO.setmode(GPIO.BCM)

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_event_callback(channel):
    
    # Verificar el estado actual del pin del botón
    if not GPIO.input(button_pin):
        print("You've pressed the Button!")
    else:
        print("You've released the Button!")

if __name__ == '__main__':
    GPIO.add_event_detect(button_pin, GPIO.BOTH,
                            callback=button_event_callback, bouncetime=50)

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
