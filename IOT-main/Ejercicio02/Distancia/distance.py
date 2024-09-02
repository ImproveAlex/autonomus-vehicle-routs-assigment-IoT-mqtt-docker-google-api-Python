import RPi.GPIO as GPIO
import signal
import time

#Pines GPIo a usar
pinTrigger = 18
pinEcho = 24

# Metodo para limpiar los pines
def close(signal, frame):
    GPIO.cleanup()
    exit(0)

# Asignar el método de cierre al evento de señal SIGINT
signal.signal(signal.SIGINT, close)

# Establecemos el Trigger como out y el Echo como in
GPIO.setmode(GPIO.BCM)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# Función Principal para calcular la distancia
def calcular_distancia():

    # Enviar al trigger una señal de arranque
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)
    
    # Inicializamos tiempo inicial y final
    start_time = time.time()
    end_time = time.time()
    
    # Esperamos a que el pin Echo se active
    while GPIO.input(pinEcho) == 0:
        start_time = time.time()
    
    # Esperamos a que el pin Echo se desactive
    while GPIO.input(pinEcho) == 1:
        end_time = time.time()
    
    # Calculamos la duracion todal que es la final menos la inical
    duration = end_time - start_time
    
    # Calcular la distancia con la formula X =  V * T dividido entre 2 ya que el tiempo es el de ir y venir 
    distance = duration * 34300 / 2  # La velocidad del sonido en cm/s es aproximadamente 34300 cm/s
    
    return distance

try:
    # Bucle para tomar muestras de medición
    while True:
        distancia = calcular_distancia()
        print("Distancia:", distancia, " cm")
        time.sleep(0.01)  # Esperar 0.01 segundos antes de tomar la siguiente muestra

# Limpiar los pines GPIO al finalizar el programa
except KeyboardInterrupt:
    GPIO.cleanup()
