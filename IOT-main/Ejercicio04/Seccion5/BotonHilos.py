import RPi.GPIO as GPIO
import time
import signal
import sys
import threading


GPIO.setmode(GPIO.BCM)

# Crear un candado para proteger la variable should_run
lock = threading.Lock()

# Variable global para saber si el coche debe arrancar o no
should_run = False

###########################
########## Botón ##########
###########################

button_pin = 23 

# Configurar el pin del botón como una entrada digital con resistencia de extracción ascendente
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Estado predeterminado del GPIO
button_state = GPIO.HIGH

# Variable booleana para saber si el boton ha sido presionado o no
def monitor_button(button_pin):
    global should_run
    while True:
        button_input = GPIO.input(button_pin)

        if button_input == GPIO.LOW:
            lock.acquire()
            should_run = not should_run
            lock.release()
            print("Button Pressed!, We are running:", should_run, "\n")

        time.sleep(0.15)

###########################
########### LDR ###########
###########################
pin_sensor_luz = 4

def rc_time(pin):
    global should_run
    if should_run:
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

    else:
        time.sleep(0.1)  # Espera si el vehículo está apagado

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

# Función para detener el motor
def stop_motor():
    global current_speed
    current_speed = 0
    dc_motor_object.stop()
    GPIO.output(motorA_pin, False)
    GPIO.output(motorB_pin, False)

# Función para encender el motor
def start_motor(speed):
    GPIO.output(motorA_pin, True)
    GPIO.output(motorB_pin, False)
    dc_motor_object.start(speed)

# Función para el hilo del motor
def move_dc_motor():
    global current_speed, should_run
    if should_run:
        dc_motor_object.ChangeDutyCycle(current_speed)
        current_speed += 5
        print("Cambiando la velocidad del motor a:", current_speed )
        if current_speed == 100:
            current_speed = 0
            stop_motor()
    else:
        stop_motor()
        time.sleep(0.1)

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
    global should_run
    if should_run:
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

    else:
            time.sleep(0.1)

# Mantener el programa en ejecución
threading.Thread(target=monitor_button, args=(button_pin,), daemon=True).start()
try:
    while True:
        #En el caso que debamos correr entonces ejecutamos los threads, los cuales 
        #a su vez tambien van a estar evaluando el valor de should_run para parar
        if should_run:
            start_motor(current_speed)

            #Iniciamos todos los threads con su variables correspondientes
            threading.Thread(target=move_dc_motor, daemon=True).start()
            threading.Thread(target=calcular_distancia, daemon=True).start()
            threading.Thread(target=rc_time, args=(pin_sensor_luz,), daemon=True).start()

            time.sleep(1)  # Reposo durante 1s para major legibilidad en la linea de comandos
        else:
            stop_motor()    #Forzamos la detencion del motor de modo manual ya que pueden haber casos en los que se nos escapa
            time.sleep(0.1)  # Wait if the vehicle is turned off

except KeyboardInterrupt:
    stop_motor()
    GPIO.cleanup()
    sys.exit(0)