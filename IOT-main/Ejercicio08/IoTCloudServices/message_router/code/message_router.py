import os
import json
import paho.mqtt.client as mqtt

JSON_FILE_PATH = "telemetry_data.json"

# Método on_connect
def on_connect(client, userdata, flags, rc, properties):
    print("Conexión establecida con el broker MQTT")
    if rc == 0:
        STATE_TOPIC = "/fic/vehicles/+/state/+"
        client.subscribe(STATE_TOPIC)
        print("Suscrito al tema:", STATE_TOPIC)
    else:
        print("Error al conectar al broker MQTT")

# Método on_message
def on_message(client, userdata, msg):
    print("Mensaje recibido en el tema:", msg.topic)
    print("Contenido del mensaje:", msg.payload.decode())
    
    # Comprobando si el topic incluye 'request_plate'
    topic = msg.topic.split('/')
    if topic[-1] == "request_plate":
        requested_id = connected_vehicles.get(msg.payload.decode())
        if requested_id is not None:
            plate_json = '{"Plate":' + connected_vehicles[requested_id] + '}'
            client.publish("/fic/vehicles/" + msg.payload.decode() + "/state/config", payload=plate_json, qos=1, retain=False)
        else:
            if len(connected_vehicles) < len(available_plates):
                vehicle_plate = available_plates[len(connected_vehicles)]
                connected_vehicles[msg.payload.decode()] = vehicle_plate
                plate_json = f'{{"Plate": "{vehicle_plate}"}}'
                client.publish("/fic/vehicles/" + msg.payload.decode() + "/state/config", payload=plate_json, qos=1, retain=False)

            else:
                print("La flota de vehículos ya está totalmente asignada")
                client.publish("/fic/vehicles/" + msg.payload.decode() + "/state/config", payload='{"Plate":"Not Available"}', qos=1, retain=False)
    elif topic[-1] == "telemetry":
        # Actualizar un archivo json local con la nueva telemetría recibida
        update_local_json(msg.payload.decode())

def update_local_json(telemetry_data):
    # Leer el archivo JSON existente o crear uno nuevo si no existe
    if os.path.exists(JSON_FILE_PATH):
        with open(JSON_FILE_PATH, 'r') as json_file:
            telemetry = json.load(json_file)
    else:
        telemetry = {}

    # Actualizar el archivo JSON con la nueva telemetría
    new_data = json.loads(telemetry_data)
    telemetry.update(new_data)

    # Escribir los datos actualizados en el archivo JSON
    with open(JSON_FILE_PATH, 'w') as json_file:
        json.dump(telemetry, json_file, indent=4)

# Configuración del cliente MQTT
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(username="fic_server", password="fic_password")
client.on_connect = on_connect
client.on_message = on_message

# Conexión al broker MQTT
MQTT_SERVER = os.getenv("MQTT_SERVER_ADDRESS")
MQTT_PORT = int(os.getenv("MQTT_SERVER_PORT"))
#print(MQTT_PORT== 1883)
client.connect(MQTT_SERVER, MQTT_PORT, 60)

# Variables globales
connected_vehicles = {}
available_plates = ["0001BBB", "0002BBB", "0003BBB", "0004BBB", "0005BBB", "0006BBB", "0007BBB", "0008BBB", "0009BBB", "0010BBB"]

# Mantener el cliente MQTT en ejecución
client.loop_forever()