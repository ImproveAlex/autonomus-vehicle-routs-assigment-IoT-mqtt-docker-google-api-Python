import RPi.GPIO as GPIO
import time

#Usaremos el pin directamente
pin_sensor_luz = 7 

# Configurar el modo GPIO en BOARD no en BCM
GPIO.setmode(GPIO.BOARD)

# Funcion que devuelve el valor de count cuando este llegue a alto
def rc_time(pin):
    count = 0

    # Configurar el pin como salida y establecerlo en bajo
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.1)

    # Cambiar el pin a entrada
    GPIO.setup(pin, GPIO.IN)

    # Contar hasta que el pin se vuelva alto
    while GPIO.input(pin) == GPIO.LOW:
        count += 1

    return count

try:
    while True:
        # Leer el valor de la resistencia
        luz = rc_time(pin_sensor_luz)
        print("Valor de luz:", luz)
        time.sleep(0.1)  # Reposo durante 100 ms

except KeyboardInterrupt:
    GPIO.cleanup()
