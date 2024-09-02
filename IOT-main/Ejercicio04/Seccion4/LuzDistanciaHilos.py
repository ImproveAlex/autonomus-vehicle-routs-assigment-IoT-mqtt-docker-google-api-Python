import RPi.GPIO as GPIO
import time
import signal
import sys
import threading

"""
NOTA PARA EL PROFESOR:

Se ha desarollado un unico programa que ejecuta en 
diferentes threads: el sensor ultrasonido, el LDR y el motor continuo
"""

GPIO.setmode(GPIO.BCM)

###########################
########### LDR ###########
###########################
pin_sensor_luz = 4

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

    print("Valor de luz: ", count, "\n")

    return count

###########################
####### Motor CC ##########
###########################

motorA_pin = 5
motorB_pin = 6
motorE_pin = 13

GPIO.setup(motorA_pin, GPIO.OUT)
GPIO.setup(motorB_pin, GPIO.OUT)
GPIO.setup(motorE_pin, GPIO.OUT)

# Objeto PWM para controlar la velocidad del motor
dc_motor_object = GPIO.PWM(motorE_pin, 100)
current_speed = 0

def start_motor(speed):
    GPIO.output(motorA_pin, True)
    GPIO.output(motorB_pin, False)
    dc_motor_object.start(speed)

# Función para detener el motor
def stop_motor(case=None):
    global current_speed
    if case is not None:
        current_speed = 0
    dc_motor_object.stop()
    GPIO.output(motorA_pin, False)
    GPIO.output(motorB_pin, False)

# Función para el hilo del motor
def move_dc_motor():
    global current_speed
    dc_motor_object.ChangeDutyCycle(current_speed)
    print("Cambiando la velocidad del motor a:", current_speed )

###########################
## Sensor de Ultrasonido ##
###########################

pinTrigger = 18
pinEcho = 24

#Establecemos el Trigger como out y el Echo como in
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

    print("Distancia:", distance, " cm ")
    
    return distance

# Mantener el programa en ejecución
try:
    start_motor(current_speed) 
    while True:
        if current_speed != 100:
            current_speed += 5
            threading.Thread(target=move_dc_motor, daemon=True).start()
        
        elif current_speed == 100:
            stop_motor()


        threading.Thread(target=calcular_distancia, daemon=True).start()

        threading.Thread(target=rc_time, args=(pin_sensor_luz,), daemon=True).start()

        time.sleep(1)  # Reposo durante 100 ms
            
except KeyboardInterrupt:
    stop_motor(1)
    GPIO.cleanup()
    sys.exit(0)