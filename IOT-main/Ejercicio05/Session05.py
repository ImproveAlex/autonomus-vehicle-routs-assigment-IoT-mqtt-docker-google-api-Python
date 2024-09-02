import RPi.GPIO as GPIO
import time
import signal
import sys
import threading
import json



GPIO.setmode(GPIO.BCM)

# Crear un candado para proteger la variable should_run
lock = threading.Lock()

# Variable global para saber si el coche debe arrancar o no
should_run = False

# Variable global para saber si hay commandos
vehicleControlCommands = None

###########################
####### Servo Motor #######
###########################

servo_pin = 16
GPIO.setup(servo_pin, GPIO.OUT)
servo_object = GPIO.PWM(servo_pin, 50)
servo_object.start(0)

def setAngle(angle):
    global servo_object, should_run
    if should_run:
        angle = max(0, min (180, angle))
        angle_min = 0
        angle_max = 180
        duty_cycle_min = 2.5
        duty_cycle_max = 12.5
        angle_as_percent =  ((angle - angle_min) / (angle_max - angle_min)) * (duty_cycle_max - duty_cycle_min) + duty_cycle_min
        servo_object.ChangeDutyCycle(angle_as_percent)
    else:
        servo_object.ChangeDutyCycle(7.5)




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
            if should_run == True:  
                print("Button Pressed!, We are running:", should_run, "\n")
            if should_run == False:
                print("Button Pressed!, We stop and set angle to: 90, and the motor has stoped\n")

        time.sleep(0.15)

###########################
########### LDR ###########
###########################
pin_sensor_luz = 4

def rc_time(pin):
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

# Función para detener el motor
def stop_motor():
    dc_motor_object.stop()
    GPIO.output(motorA_pin, False)
    GPIO.output(motorB_pin, False)

# Función para encender el motor
def start_motor():
    GPIO.output(motorA_pin, True)
    GPIO.output(motorB_pin, False)
    dc_motor_object.start(0)

# Función para el hilo del motor
def move_dc_motor(speed):
    if should_run:
        dc_motor_object.ChangeDutyCycle(speed)
        print("Cambiando la velocidad del motor a:", speed)
        if speed == 100:
            stop_motor()

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
    if should_run:
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

# Función para leer un json con commandos
def load_commands():
    global vehicleControlCommands
    # Abrimos el archivo JSON que contiene los comandos
    file = open('./commands.json')
    # Cargamos los comandos desde el archivo y los almacenamos en vehicleControlCommands
    vehicleControlCommands = json.load(file)

def ExcecuteCommand():
    while len(vehicleControlCommands) > 0:
        # Iniciamos todos los threads con su variables correspondientes
        while should_run and len(vehicleControlCommands) > 0:
            # Extraemos y eliminamos el primer comando de la lista de comandos
            command = vehicleControlCommands.pop(0)
            print("Setting angle to:", command['SteeringAngle'])
            threading.Thread(target=setAngle, args=(command['SteeringAngle'],), daemon=True).start()
            start_motor()
            threading.Thread(target=move_dc_motor, args=(command['Speed'],), daemon=True).start()
            threading.Thread(target=calcular_distancia, daemon=True).start()
            threading.Thread(target=rc_time,args=(pin_sensor_luz,), daemon=True).start()
            # Esperamos un tiempo dado por el comando
            time.sleep(command['Time'])
        setAngle(90)
        stop_motor()
            
    return

# Mantener el programa en ejecución
threading.Thread(target=monitor_button, args=(button_pin,), daemon=True).start()
try:
    # Cargamos los comandos
    load_commands()
    start_motor()
    # Iniciamos el Unico hilo que ejecuta los comandos
    threading.Thread(target=ExcecuteCommand, daemon=True).start()
    while True:
        #bucle para mantener el programa en ejecucion
        time.sleep(0.1)


except KeyboardInterrupt:
    stop_motor()
    setAngle(90)
    GPIO.cleanup()
    sys.exit(0)