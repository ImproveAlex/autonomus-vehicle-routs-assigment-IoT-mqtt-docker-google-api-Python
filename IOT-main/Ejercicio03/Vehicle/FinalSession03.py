import RPi.GPIO as GPIO
import time
import signal
import sys

# Variable con el valor en BCM del pin
pin_sensor_luz = 4
pinTrigger = 18
button_pin = 23
pinEcho = 24

# Variable para rastrear el estado del botón
pressed = False

GPIO.setmode(GPIO.BCM)

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Establecer el Trigger como salida y el Echo como entrada
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

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

# Función principal para calcular la distancia
def calcular_distancia():

    # Enviar una señal de arranque al Trigger
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)
    
    # Inicializar tiempo inicial y final
    start_time = time.time()
    end_time = time.time()
    
    # Esperar a que el pin Echo se active
    while GPIO.input(pinEcho) == 0:
        start_time = time.time()
    
    # Esperar a que el pin Echo se desactive
    while GPIO.input(pinEcho) == 1:
        end_time = time.time()
    
    # Calcular la duración total (tiempo de vuelo)
    duration = end_time - start_time
    
    # Calcular la distancia con la fórmula: Distancia = Velocidad * Tiempo / 2 (tiempo de ida y vuelta)
    distance = duration * 34300 / 2  # La velocidad del sonido en cm/s es aproximadamente 34300 cm/s
    
    return distance

def signal_handler(sig, frame):
    # Limpiar los pines GPIO y salir del programa
    GPIO.cleanup()
    sys.exit(0)

def check_button_state(channel):
    global pressed
    # Cambiar el estado del bucle cuando se detecta un evento de botón
    pressed = not pressed

try:
    if __name__ == '__main__':
        # Detectar eventos de flanco de subida en el pin del botón y llamar a la función check_button_state
        GPIO.add_event_detect(button_pin, GPIO.RISING,
                                callback=check_button_state, bouncetime=25)
        
        # Configurar el manejador de señales para limpiar los pines GPIO al interrumpir el programa
        signal.signal(signal.SIGINT, signal_handler)
        
        while True:
            if pressed:
                # Llamar a las funciones en un bucle si el bucle está en ejecución
                distancia = calcular_distancia()
                print("Distancia:", distancia, " cm")
                luz = rc_time(pin_sensor_luz)
                print("Valor de luz:", luz)
                time.sleep(0.1)
            else:
                # Esperar para evitar uso de CPU
                time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()