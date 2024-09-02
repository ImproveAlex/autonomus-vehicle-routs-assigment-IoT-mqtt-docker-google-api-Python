import RPi.GPIO as GPIO
import time
import signal
import sys
import threading

"""
NOTA PARA EL PROFESOR:

En el main se aumenta periodicamente la velocidad del coche, si este llega a 100 se para el programa
"""

# Definición de pines
motorA_pin = 5
motorB_pin = 6
motorE_pin = 13

# Inicializar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(motorA_pin, GPIO.OUT)
GPIO.setup(motorB_pin, GPIO.OUT)
GPIO.setup(motorE_pin, GPIO.OUT)

# Objeto PWM para controlar la velocidad del motor
dc_motor_object = GPIO.PWM(motorE_pin, 100)
current_speed = 0

# Función para iniciar el motor
def start_motor(speed):
    GPIO.output(motorA_pin, True)
    GPIO.output(motorB_pin, False)
    dc_motor_object.start(speed)

# Función para detener el motor
def stop_motor():
    global current_speed
    current_speed = 0
    dc_motor_object.stop()
    GPIO.output(motorA_pin, False)
    GPIO.output(motorB_pin, False)

# Función para el hilo del motor
def move_dc_motor():
    global current_speed
    dc_motor_object.ChangeDutyCycle(current_speed)
    print("Cambiando la velocidad del motor a:", current_speed )



# Mantener el programa en ejecución
try:
    start_motor(current_speed) 
    while True:
        current_speed += 5
        threading.Thread(target=move_dc_motor, daemon=True).start()
        time.sleep(1)   # Mantener el motor en movimiento
        # Si el hilo es interrumpido, detener el motor
        if not threading.current_thread() or current_speed == 100:
            stop_motor()
            GPIO.cleanup()
            break
            
except KeyboardInterrupt:
    stop_motor()
    GPIO.cleanup()
    sys.exit(0)
