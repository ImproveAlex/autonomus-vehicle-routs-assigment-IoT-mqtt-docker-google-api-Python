import RPi.GPIO as GPIO
import time
import sys

#variable con el valor en BCM del pin
button_pin = 23

GPIO.setmode(GPIO.BCM)

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Estado predeterminado del GPIO
button_state = GPIO.HIGH

# Variable booleana para saber si el boton ha sido presionado o no
pressed = False

try:
    while True:
        # Leer el estado del botón
        button_input = GPIO.input(button_pin)

        # Si el botón está presionado y no estaba previamente presionado, imprimir el mensaje
        if button_input == GPIO.LOW and not pressed:
            print("Button Pressed!")
            pressed = True 

        # Si el botón no está presionado, cambiar el estado de presionado a False
        elif button_input == GPIO.HIGH:
            pressed = False

        # Esperar un breve tiempo antes de volver a leer el estado del botón
        time.sleep(0.1)

except Exception as e:
    print(e)
    GPIO.cleanup()
    sys.exit(0)
