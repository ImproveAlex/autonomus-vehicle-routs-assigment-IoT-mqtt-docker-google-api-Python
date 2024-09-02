import RPi.GPIO as GPIO
import time
import signal
import sys
import threading
import json
"""
NOTAS PARA EL PROFESOR:

-Se ha mofificado pines ya que en la configuracion original
 habian coincidencias con respecto a los ejercicios

-El valor de la luz para encender es de 20.000 unidades ya que
aun con una linterna directo al LDR los valores no bajaban de 10.000

-Cada sensor/actuador tiene su propio hilo, incluido el leer commandos

-Solo funciona un Led fisico, por lo que el programa este bien programado y el
circuto bien conectado uno de los leds rgb esta quemado, asi que  he tenido
que mover el led fisico de derecho a izquierda y comprobar su funcionamiento,
que es el deseado.
"""



GPIO.setmode(GPIO.BCM)

# Crear un candado para proteger la variable should_run
lock = threading.Lock()

# Variable global para saber si el coche debe arrancar o no
should_run = False

# Variable global para saber si el coche antes estaba arrancado y se detuvo por un obtaculo
stoped_by_distance = False 

# Variable global para saber si hay commandos
vehicleControlCommands = None

# Variable global para saber el valor de la luz
current_light = 0

# Variable global para saber el angulo del volante 
current_angle = 90

# Variable gloval para saber la velocidad el motor CC
current_speed = 0

# Variable gloval para llevar un registro de la velocidad anterior
last_speed = 0

# Variable gloval para saber la distancia actual
current_distance = 0




###########################
######## Luces RGB ########
###########################

def setup_luces():
    global rear_right_red, rear_right_green, rear_right_blue
    global rear_left_red, rear_left_green, rear_left_blue
    global pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right
    global pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left

    rear_right_red = 17
    GPIO.setup(rear_right_red, GPIO.OUT)
    GPIO.output(rear_right_red, True)
    pwm_red_rear_right = GPIO.PWM(rear_right_red, 100)
    pwm_red_rear_right.start(0)
    pwm_red_rear_right.ChangeDutyCycle(0)

    rear_right_green = 22
    GPIO.setup(rear_right_green, GPIO.OUT)
    GPIO.output(rear_right_green, True)
    pwm_green_rear_right = GPIO.PWM(rear_right_green, 100)
    pwm_green_rear_right.start(0)
    pwm_green_rear_right.ChangeDutyCycle(0)

    rear_right_blue = 27
    GPIO.setup(rear_right_blue, GPIO.OUT)
    GPIO.output(rear_right_blue, True)
    pwm_blue_rear_right = GPIO.PWM(rear_right_blue, 100)
    pwm_blue_rear_right.start(0)
    pwm_blue_rear_right.ChangeDutyCycle(0)

    rear_left_red = 14
    GPIO.setup(rear_left_red, GPIO.OUT)
    GPIO.output(rear_left_red, True)
    pwm_red_rear_left = GPIO.PWM(rear_left_red, 100)
    pwm_red_rear_left.start(0)
    pwm_red_rear_left.ChangeDutyCycle(0)

    rear_left_green = 12
    GPIO.setup(rear_left_green, GPIO.OUT)
    GPIO.output(rear_left_green, True)
    pwm_green_rear_left = GPIO.PWM(rear_left_green, 100)
    pwm_green_rear_left.start(0)
    pwm_green_rear_left.ChangeDutyCycle(0)

    rear_left_blue = 15
    GPIO.setup(rear_left_blue, GPIO.OUT)
    GPIO.output(rear_left_blue, True)
    pwm_blue_rear_left = GPIO.PWM(rear_left_blue, 100)
    pwm_blue_rear_left.start(0)
    pwm_blue_rear_left.ChangeDutyCycle(0)


def pinOn(pwm_object, freq=100) :
    pwm_object.ChangeDutyCycle(freq)

def pinOff(pwm_object) :
    pwm_object.ChangeDutyCycle(0)


def red(redPin, greenPin, bluePin):
    pinOn(redPin)
    pinOff(greenPin)
    pinOff(bluePin)

def white(redPin, greenPin, bluePin):
    pinOn (redPin)
    pinOn (greenPin)
    pinOn (bluePin)

def yellow(redPin, greenPin, bluePin):
    pinOff(bluePin)
    pinOn(redPin)
    pinOn(greenPin)

def turnOff(redPin, greenPin, bluePin):
    pinOff(redPin)
    pinOff(greenPin)
    pinOff(bluePin)


def lightControler():
    global rear_right_red, rear_right_green, rear_right_blue
    global rear_left_red, rear_left_green, rear_left_blue
    global pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right
    global pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left
    if should_run:
        while current_angle > 100:
            yellow(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
            time.sleep(0.1667)
            turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
            time.sleep(0.1667)
            turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
            
            
            
        while current_angle < 80:
            yellow(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
            time.sleep(0.1667)
            turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
            time.sleep(0.1667)
            turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    
        if should_run:    
            if current_speed < last_speed:
                pwm_red_rear_left.ChangeDutyCycle(100)
                pwm_red_rear_right.ChangeDutyCycle(100)
                red(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
                red(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
                return

            if current_light > 20000:
                pwm_red_rear_left.ChangeDutyCycle(50)
                pwm_red_rear_right.ChangeDutyCycle(50)
                red(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
                red(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
                return

            else:
                turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
                turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
        else:
                turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
                turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    else:
        turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
        turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)




###########################
####### Servo Motor #######
###########################

servo_pin = 16
GPIO.setup(servo_pin, GPIO.OUT)
servo_object = GPIO.PWM(servo_pin, 50)
servo_object.start(0)

def setAngle(angle):
    global servo_object, should_run, current_angle
    if should_run:
        angle = max(0, min (180, angle))
        angle_min = 0
        angle_max = 180
        duty_cycle_min = 2.5
        duty_cycle_max = 12.5
        angle_as_percent =  ((angle - angle_min) / (angle_max - angle_min)) * (duty_cycle_max - duty_cycle_min) + duty_cycle_min
        servo_object.ChangeDutyCycle(angle_as_percent)
        lock.acquire()
        current_angle = angle
        lock.release()
    else:
        lock.acquire()
        current_angle = 90
        lock.release()
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
                setAngle(90)
                move_dc_motor(current_speed)
                turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
                turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
                print("Button Pressed!, We stop and set angle to: 90, and the motor has stoped\n")

        time.sleep(0.15)

###########################
########### LDR ###########
###########################
pin_sensor_luz = 4

def rc_time(pin):
    global current_light
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
        lock.acquire()
        current_light = count
        lock.release()

    
        


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
    global current_speed, last_speed
    if should_run:
        dc_motor_object.ChangeDutyCycle(speed)
        print("Cambiando la velocidad del motor a:", speed)
        lock.acquire()
        last_speed = current_speed
        current_speed = speed
        lock.release()
        if speed == 100:
            lock.acquire()
            last_speed = current_speed
            current_speed = speed
            lock.release()
            stop_motor()
    else:
        lock.acquire()
        last_speed = current_speed
        current_speed = speed
        lock.release()
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
    global current_distance, should_run, stoped_by_distance
    # Enviar al trigger una señal de arranque
    while True:
        while should_run or current_distance < 20:
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

            current_distance = distance
            if current_distance < 20:
                lock.acquire()
                should_run = False
                stoped_by_distance = True
                lock.release()
                print("OBJETO DELANTE NO NOS MOVEMOS ")
                print("Distancia:",current_distance, " cm\n ")
                time.sleep(1)

            elif current_distance > 20 and not should_run:
                print("Distancia:", current_distance, " cm ")
                lock.acquire()
                should_run = True
                lock.release()



        

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
            threading.Thread(target=rc_time,args=(pin_sensor_luz,), daemon=True).start()
            threading.Thread(target=calcular_distancia, daemon=True).start()
            threading.Thread(target=lightControler, daemon=True).start()
            # Esperamos un tiempo dado por el comando

            # Convertir command['Time'] a un entero
            time_to_sleep = command['Time']

            # Esperamos un tiempo dado por el comando
            while time_to_sleep > 0:
                time_to_sleep -= 0.5
                if current_distance > 20:
                    time.sleep(0.5)
        setAngle(90)
        stop_motor()
            
    return

# Mantener el programa en ejecución
threading.Thread(target=monitor_button, args=(button_pin,), daemon=True).start()
try:
    # Cargamos los comandos
    load_commands()
    start_motor()
    setup_luces()
    # Iniciamos el Unico hilo que ejecuta los comandos
    threading.Thread(target=ExcecuteCommand, daemon=True).start()
    while True:
        #bucle para mantener el programa en ejecucion
        time.sleep(0.1)


except KeyboardInterrupt:
    stop_motor()
    setAngle(90)
    turnOff(pwm_red_rear_right, pwm_green_rear_right, pwm_blue_rear_right)
    turnOff(pwm_red_rear_left, pwm_green_rear_left, pwm_blue_rear_left)
    GPIO.cleanup()
    sys.exit(0)
