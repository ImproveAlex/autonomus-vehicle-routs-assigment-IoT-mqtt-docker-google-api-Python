# Usa la imagen base de Debian Buster
FROM debian:buster

# Actualiza e instala Mosquitto y sus clientes
RUN apt update && apt upgrade -y \
    && apt install -y mosquitto mosquitto-clients

# Crea el archivo de contraseña y agrega un usuario y contraseña
RUN touch /etc/mosquitto/passwd \
    && mosquitto_passwd -b /etc/mosquitto/passwd fic_server fic_password

# Cambia los permisos del archivo de configuración
RUN chmod 644 /etc/mosquitto/mosquitto.conf

# Copia el archivo de configuración personalizado
COPY code/mosquitto.conf /etc/mosquitto/mosquitto.conf

# Define el comando para lanzar Mosquitto
CMD ["/usr/sbin/mosquitto", "-c", "/etc/mosquitto/mosquitto.conf"]
