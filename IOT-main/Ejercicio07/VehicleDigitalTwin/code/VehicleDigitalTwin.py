import time
import threading
import json
import math
import requests
import random

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
    pending_routes.append(route)
    print("Se ha añadido una nueva ruta pendiente:", route)

def routes_manager(origin_address="Toronto", destination_address="Montreal"):
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

     print("La lista de comandos tiene {} comandos asociados".format(len(vehicleControlCommands)))
     while len(pending_routes) > 0:
        while vehicleControlCommands:
            lock.acquire()
            command = vehicleControlCommands.pop(0) 
            step = currentRouteDetailedSteps.pop(0)
            lock.release()
            executeCommand(command, step) 
        if len(pending_routes) > 0:
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
        print("\n------------------------IMPRIMIENDO EL ESTADO DEL VEHICULO------------------------")
        print("El Angulo es: {}, Velocidad: {}, Posición:  {}, Distancia al obj más cercano: {}, Intensidad luz: {}".format(current_steering,current_speed,current_position, current_obstacle_distance, current_light_intensity))
        print("Los valores de las 4 luces son: ", current_leds)
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


if __name__ == '__main__':
    try:
        my_route = '{"Origin:"Toronto", "Destination:"Montreal"}'
        routes_loader(my_route)
        routes_manager(my_route)
        t2 = threading.Thread(target=environment_simulator, daemon=True)
        t2.start()
        t3 = threading.Thread(target=vehicle_controller, daemon=True)
        t3.start()
        t4 = threading.Thread(target=led_controller, daemon=True)
        t4.start()
        t2.join()
        t3.join()
        t4.join()
    except Exception as e:
        print(e)
        vehicle_stop()