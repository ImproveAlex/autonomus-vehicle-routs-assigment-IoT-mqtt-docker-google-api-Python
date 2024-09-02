import time
import threading
import json
import math
import requests
import random
import os
import subprocess
from datetime import datetime
import paho.mqtt.client as mqtt

currentRouteDetailedSteps = []
vehicleControlCommands = []
pending_routes = []
google_maps_api_key = "AIzaSyAmv3GBC28NTPKN4Jx5HzA0nt6ZsD6SlKI"

current_steering = 90  
current_speed = 0
last_speed = 0 
current_position = {"latitude": 0, "longitude": 0}
current_light_intensity = 0.0
current_obstacle_distance = 0.0
current_leds = [{"Color": "White", "Intensity": 0.0, "Blinking": False},
                {"Color": "White", "Intensity": 0.0, "Blinking": False},
                {"Color": "Red", "Intensity": 0.0, "Blinking": False},
                {"Color": "Red", "Intensity": 0.0, "Blinking": False}]

lock = threading.Lock()


def routes_loader(route):
    global pending_routes

    route_set = json.loads(route)
    print(len(pending_routes))
    lock.acquire()
    pending_routes.append(route_set)
    lock.release()
    print(len(pending_routes))
    print("Se ha añadido una nueva ruta pendiente:", route_set)
    routes_manager(route_set["Origin"], route_set["Destination"])

def routes_manager(origin_address, destination_address):
    global currentRouteDetailedSteps
    global vehicleControlCommands

    # print("Asignando una ruta al vehículo")
    url = "https://maps.googleapis.com/maps/api/directions/json?origin=" + origin_address + "&destination=" + \
          destination_address + "&key=" + google_maps_api_key
    #print("URL: {}".format(url))
    payload = {}
    headers = {}
    response = requests.request("GET", url, headers=headers, data=payload)
    steps = response.json()["routes"][0]["legs"][0]["steps"]
    get_detailed_steps(steps)
    get_commands()
    #print("He acabado de asignar los comandos al vehículo")

# Método para obtener pasos detallados
def get_detailed_steps(steps):
    global currentRouteDetailedSteps

    for step in steps:
        # Determinar la velocidad en escala de 100
        stepSpeed = (step["distance"]["value"] / 1000) / (step["duration"]["value"] / 3600)
        # Determinar la distancia del paso
        stepDistance = step["distance"]["value"]
        # Determinar el tiempo del paso
        stepTime = step["duration"]["value"]
        # Determinar la maniobra que se tiene que ejecutar con el volante
        try:
            stepManeuver = step["maneuver"]
        except KeyError:
            stepManeuver = "Straight"
        # Determinar los waypoints que se corresponden con ese paso
        substeps = decode_polyline(step["polyline"]["points"])
        
        for index in range(len(substeps) - 1):  # Iterar sobre los subpasos
            p1 = {"latitude": substeps[index][0], "longitude": substeps[index][1]}
            p2 = {"latitude": substeps[index + 1][0], "longitude": substeps[index + 1][1]}
            
            # Calcular la distancia entre p1 y p2
            points_distance = calculate_distance(p1, p2)

            if points_distance > 0.001:
                subStepDuration = stepDistance / stepTime
                new_detailed_step = {
                    "Origin": p1,
                    "Destination": p2,
                    "Speed": stepSpeed,
                    "Time": subStepDuration,
                    "Distance": points_distance,
                    "Maneuver": stepManeuver
                }
                # Agregar el paso detallado a la lista de pasos detallados
                currentRouteDetailedSteps.append(new_detailed_step)
    
    # Imprimir la cantidad de pasos detallados en la ruta
    print("La ruta tiene {} pasos detallados".format(len(currentRouteDetailedSteps)))


def calculate_distance(p1, p2):
    # Obtener las coordenadas de p1
    p1Latitude = p1["latitude"]
    p1Longitude = p1["longitude"]
    
    # Obtener las coordenadas de p2
    p2Latitude = p2["latitude"]
    p2Longitude = p2["longitude"]
    
    # Radio de la Tierra en kilómetros y millas
    earth_radius = {"km": 6371.0087714, "mile": 3959}
    
    # Calcular la distancia utilizando la fórmula del haversine
    result = earth_radius["km"] * math.acos(math.cos(math.radians(p1Latitude)) * math.cos(math.radians(p2Latitude)) * math.cos(math.radians(p2Longitude) - math.radians(p1Longitude)) + math.sin(math.radians(p1Latitude)) * math.sin(math.radians(p2Latitude)))
    
    # Devolver el resultado
    return result


def decode_polyline(polyline_str):
    index, lat, lng = 0, 0, 0
    coordinates = []

    # Las coordenadas tienen longitud variable cuando están codificadas,
    # así que simplemente llevamos un registro de si hemos llegado al final de la cadena.
    # En cada iteración del bucle while, se decodifica una única coordenada.
    while index < len(polyline_str):
        changes = {'latitude': 0, 'longitude': 0}

        # Recolectamos los cambios de Lat/Lon, los almacenamos en un diccionario para aplicarlos luego
        for unit in ['latitude', 'longitude']:
            shift, result = 0, 0
            while True:
                byte = ord(polyline_str[index]) - 63
                index += 1
                result |= (byte & 0x1F) << shift
                shift += 5
                if not byte >= 0x20:
                    break

            if (result & 1):
                changes[unit] = ~(result >> 1)
            else:
                changes[unit] = (result >> 1)

        lat += changes['latitude']
        lng += changes['longitude']
        coordinates.append((lat / 100000.0, lng / 100000.0))

    return coordinates

def vehicle_controller():
     global vehicleControlCommands
     global pending_routes
     global event_message

     while True:
        while len(pending_routes) > 0:
            print("Estoy aqui viendo si entre en pending routs")
            while vehicleControlCommands:
                print("hay comandos asi que entro por aca")
                lock.acquire()
                command = vehicleControlCommands.pop(0) 
                step = currentRouteDetailedSteps.pop(0)
                lock.release()
                executeCommand(command, step) 
            if len(pending_routes) > 0:
                lock.acquire()
                event_message = "Route Completed"
                lock.release()
                pending_routes.pop(0)

def get_commands():
    global currentRouteDetailedSteps
    global vehicleControlCommands
    index = 0
    for detailedStep in currentRouteDetailedSteps:
        index += 1
        #print("Generando el comando {} para el paso {}".format(index, detailedStep))

        steering_angle = 90.0

        maneuver_upper = detailedStep["Maneuver"].upper()

        if maneuver_upper in ["STRAIGHT", "RAMP-LEFT", "RAMP-RIGHT", "MERGE", "MANEUVER-UNSPECIFIED"]:
            steering_angle = 90.0
        elif maneuver_upper == "TURN-LEFT":
            steering_angle = 45.0
        elif maneuver_upper == "UTURN-LEFT":
            steering_angle = 0.0
        elif maneuver_upper == "TURN-SHARP-LEFT":
            steering_angle = 15.0
        elif maneuver_upper == "TURN-SLIGHT-LEFT":
            steering_angle = 60.0
        elif maneuver_upper == "TURN-RIGHT":
            steering_angle = 135.0
        elif maneuver_upper == "UTURN-RIGHT":
            steering_angle = 180.0
        elif maneuver_upper == "TURN-SHARP-RIGHT":
            steering_angle = 105.0
        elif maneuver_upper == "TURN-SLIGHT-RIGHT":
            steering_angle = 150.0

        new_command = {
            "SteeringAngle": steering_angle,
            "Speed": detailedStep["Speed"],
            "Time": detailedStep["Time"]
        }

        vehicleControlCommands.append(new_command)

def executeCommand(command, step):
    global current_steering
    global current_speed
    global current_position
    global current_light_intensity
    global last_speed
    
    # Acquire the lock before updating shared variables
    lock.acquire()
    try:
        current_steering = command["SteeringAngle"]
        current_speed = command["Speed"]
        last_speed = current_speed
        current_position = step["Destination"]
        print("The value of current position is: ",current_position)

        #print("\n------------------------IMPRIMIENDO EL ESTADO EL VEHICULO------------------------")
        #print("El Angulo es: {}, Velocidad: {}, Posición:  {}, Distancia al obj más cercano: {}, Intensidad luz: {}".format(current_steering,current_speed,current_position, current_obstacle_distance, current_light_intensity))
        #print("Los valores de las 4 luces son: ", current_leds)
    finally:
        # Release the lock after updating shared variables
        lock.release()
    
    # Sleep for the specified duration
    time.sleep(command["Time"])

def vehicle_stop():
    global vehicleControlCommands
    global currentRouteDetailedSteps
    global current_steering
    global current_speed
    global last_speed
    global current_leds
    global current_obstacle_distance
    global current_light_intensity
    
    vehicleControlCommands = []
    currentRouteDetailedSteps = []
    current_steering = 90.0
    current_speed = 0
    last_speed = 0
    current_leds_str = '[{"Color": "White", "Intensity": 0.0, "Blinking": "False"},' \
        '{"Color": "White", "Intensity": 0.0, "Blinking": "False"}, ' \
        '{"Color": "Red", "Intensity": 0.0, "Blinking": "False"}, ' \
        '{"Color": "Red", "Intensity": 0.0, "Blinking": "False"}]'
    current_leds = json.loads(current_leds_str)
    current_obstacle_distance = 0.0
    current_light_intensity = 0.0

def environment_simulator():
    global current_light_intensity
    global current_obstacle_distance
    global current_speed
    global vehicleControlCommands
    global lock

    
    while True:
        # Simulate the light
        lock.acquire()
        try:
            if current_light_intensity <= 0.0:
                current_light_intensity = random.uniform(0.0, 3000.0)
            else:
                current_light_intensity += random.randint(-300, 300)
        finally:
            lock.release()

        # Simulate the obstacle distance
        lock.acquire()
        try:
            if current_obstacle_distance <= 0.0:
                current_obstacle_distance = random.uniform(0.0, 50.0)
            else:
                current_obstacle_distance += random.randint(-5, 5)
        finally:
            lock.release()

        # Determine if the vehicle should brake
        lock.acquire()
        try:
            if current_obstacle_distance < 10:
                current_speed = 0
            elif current_speed == 0:
                # If the vehicle is stopped and the distance is safe, set the speed based on the current command
                if vehicleControlCommands:
                    command = vehicleControlCommands[0]
                    current_speed = command["Speed"]
        finally:
            lock.release()

        time.sleep(0.2)


def led_controller():
    global current_leds
    global current_steering
    global current_speed
    global current_light_intensity
    global lock
    
    # Actualizar los valores en current_leds basados en las condiciones actuales
    while True:
        # Acquire the lock before accessing shared variables
        lock.acquire()
        if current_steering > 100:
            current_leds[0]["Color"] = "Yellow"
            current_leds[0]["Intensity"] = 100.0
            current_leds[0]["Blinking"] = True
            current_leds[1]["Color"] = "White"
            current_leds[1]["Intensity"] = 100.0
            current_leds[1]["Blinking"] = False
            current_leds[2]["Color"] = "Yellow"
            current_leds[2]["Intensity"] = 100.0
            current_leds[2]["Blinking"] = True
            current_leds[3]["Color"] = "Red"
            current_leds[3]["Intensity"] = 100.0
            current_leds[3]["Blinking"] = False
        elif current_steering < 80:
            current_leds[0]["Color"] = "White"
            current_leds[0]["Intensity"] = 100.0
            current_leds[0]["Blinking"] = False
            current_leds[1]["Color"] = "Yellow"
            current_leds[1]["Intensity"] = 100.0
            current_leds[1]["Blinking"] = True
            current_leds[2]["Color"] = "Red"
            current_leds[2]["Intensity"] = 100.0
            current_leds[2]["Blinking"] = False
            current_leds[3]["Color"] = "Yellow"
            current_leds[3]["Intensity"] = 100.0
            current_leds[3]["Blinking"] = True
        elif current_light_intensity > 2000:
            current_leds[0]["Color"] = "White"
            current_leds[0]["Intensity"] = 100.0
            current_leds[0]["Blinking"] = False
            current_leds[1]["Color"] = "White"
            current_leds[1]["Intensity"] = 100.0
            current_leds[1]["Blinking"] = False
            current_leds[2]["Color"] = "Red"
            current_leds[2]["Intensity"] = 50.0
            current_leds[2]["Blinking"] = False
            current_leds[3]["Color"] = "Red"
            current_leds[3]["Intensity"] = 50.0
            current_leds[3]["Blinking"] = False
        elif current_speed < last_speed:
            current_leds[0]["Color"] = "White"
            current_leds[0]["Intensity"] = 100.0
            current_leds[0]["Blinking"] = False
            current_leds[1]["Color"] = "White"
            current_leds[1]["Intensity"] = 100.0
            current_leds[1]["Blinking"] = False
            current_leds[2]["Color"] = "Red"
            current_leds[2]["Intensity"] = 50.0
            current_leds[2]["Blinking"] = False
            current_leds[3]["Color"] = "Red"
            current_leds[3]["Intensity"] = 50.0
            current_leds[3]["Blinking"] = False
        else:
            current_leds[0]["Color"] = "White"
            current_leds[0]["Intensity"] = 0.0
            current_leds[0]["Blinking"] = False
            current_leds[1]["Color"] = "White"
            current_leds[1]["Intensity"] = 0.0
            current_leds[1]["Blinking"] = False
            current_leds[2]["Color"] = "Red"
            current_leds[2]["Intensity"] = 0.0
            current_leds[2]["Blinking"] = False
            current_leds[3]["Color"] = "Red"
            current_leds[3]["Intensity"] = 0.0
            current_leds[3]["Blinking"] = False
        
        # Release the lock after updating shared variables
        lock.release()
        time.sleep(0.2)

# Definir el topic de solicitud de matrícula y otros topics relevantes
vehicle_plate = ""
event_message = ""
PLATE_REQUEST_TOPIC = "/fic/vehicles/telemetry/state/request_plate"
STATE_TOPIC = "/fic/vehicles/telemetry/state/telemetry"
CONFIG_TOPIC = "/fic/vehicles/+/state/config"
ROUTES_TOPIC = "/fic/vehicles/+/state/routes"

# Método on_connect
def on_connect(client, userdata, flags, rc, properties):
    print("Conexión establecida con el broker MQTT")
    if rc == 0:
        print("Conexión exitosa al broker MQTT")
        # Suscribirse a los canales de telemetría del vehículo
        client.subscribe(CONFIG_TOPIC)
        print("Suscribiéndose al tema de telemetría del vehículo: {}".format(CONFIG_TOPIC))
        client.subscribe(ROUTES_TOPIC)
        print("Suscribiéndose al tema de telemetría del vehículo: {}".format(ROUTES_TOPIC))
        # Publicar un mensaje para solicitar la placa del vehículo
        client.publish(PLATE_REQUEST_TOPIC, payload=get_host_name(), qos=1, retain=False)
        print("Publicando en el tema PLATE_REQUEST_TOPIC: {}, Carga útil: {}".format(PLATE_REQUEST_TOPIC, get_host_name()))

    else:
        print("Error al conectar al broker MQTT")

def on_message(client, userdata, msg):
    global vehicle_plate  # Acceder a la variable global
    
    # Imprimir el mensaje recibido desde el broker
    print("Mensaje recibido en el topic:", msg.topic)
    print("Contenido del mensaje:", msg.payload.decode())
    
    # Comprobar si el topic del mensaje incluye "config"
    topic = (msg.topic).split('/')
    if topic[-1] == "config":
        # Procesar la configuración recibida
        config_received = msg.payload.decode()
        json_config_received = json.loads(config_received)
        
        # Actualizar la matrícula del vehículo si está disponible en la configuración
        if "Plate" in json_config_received and json_config_received["Plate"] != "Not Available":
            vehicle_plate = json_config_received["Plate"]

    elif topic[-1] == "routes":
        # Realizar la asignación de la ruta al vehículo
        required_route = msg.payload.decode()
        routes_loader(required_route)

def get_host_name():
    bashCommandName = 'echo $HOSTNAME'
    host = subprocess.check_output(['bash','-c', bashCommandName]).decode("utf-8").strip()
    return host

def publish_event(client):
    # Crear el evento a enviar
    event_to_send = {
        "Plate": vehicle_plate,
        "Event": event_message,
        "Timestamp": datetime.timestamp(datetime.now())
    }
    # Publicar el evento en el tópico correspondiente
    host = get_host_name()
    EVENTS_TOPIC = f"/fic/vehicles/{host}/state/routes_completed"
    client.publish(EVENTS_TOPIC, payload=str(event_to_send), qos=1, retain=False)

def mqtt_communications():
    # Crear cliente MQTT
    global event_message

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(username="fic_server", password="fic_password")
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Conectar al broker MQTT
    MQTT_SERVER = os.getenv("MQTT_SERVER_ADDRESS")
    MQTT_PORT = int(os.getenv("MQTT_SERVER_PORT"))
    client.connect(MQTT_SERVER, MQTT_PORT, 60)
    
    # Iniciar el loop de MQTT
    client.loop_start()
    while vehicle_plate == "":
        time.sleep(1)
    
    # Publicar telemetría del vehículo periódicamente
    while True:
        # Obtener la telemetría del vehículo
        if event_message != "":
            publish_event(client)
            lock.acquire()
            event_message = ""
            lock.release()

        while len(pending_routes) > 0:
            vehicle_status = {
                "id": get_host_name(),
                "vehicle_plate": vehicle_plate,
                "telemetry": {
                    "current_steering": current_steering,
                    "current_speed": current_speed,
                    "current_position": current_position,
                    "current_leds": json.dumps(current_leds),
                    "current_light_intensity": current_light_intensity,
                    "current_obstacle_distance": current_obstacle_distance,
                    "time_stamp": datetime.now().isoformat()
                }
            }
            
            # Convertir a formato JSON
            json_telemetry = json.dumps(vehicle_status)
            
            # Publicar telemetría en el tópico adecuado
            client.publish(STATE_TOPIC, payload=json_telemetry, qos=1, retain=False)
            
            # Esperar un tiempo antes de publicar la próxima telemetría
            time.sleep(10)
    client.loop_stop()



if __name__ == '__main__':
    try:
        t1 = threading.Thread(target=mqtt_communications, daemon=True)
        t1.start()
        t2 = threading.Thread(target=environment_simulator, daemon=True)
        t2.start()
        t3 = threading.Thread(target=vehicle_controller, daemon=True)
        t3.start()
        t4 = threading.Thread(target=led_controller, daemon=True)
        t4.start()
        t1.join()
        t2.join()
        t3.join()
        t4.join()
    except Exception as e:
        print(e)
        vehicle_stop()