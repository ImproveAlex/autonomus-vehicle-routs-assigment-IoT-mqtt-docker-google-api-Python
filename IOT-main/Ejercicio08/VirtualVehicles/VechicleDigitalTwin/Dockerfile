# Utiliza la imagen base de Python 3.11.1
FROM python:3.11.1

# Copia el código fuente a la carpeta /usr/src/app en el contenedor
COPY ./code /usr/src/app

# Establece /usr/src/app como el directorio de trabajo
WORKDIR /usr/src/app

# Instala los paquetes necesarios
RUN pip install requests paho-mqtt

# Ejecuta el código de VehicleDigitalTwin.py
CMD ["python","-u", "VehicleDigitalTwin.py"]