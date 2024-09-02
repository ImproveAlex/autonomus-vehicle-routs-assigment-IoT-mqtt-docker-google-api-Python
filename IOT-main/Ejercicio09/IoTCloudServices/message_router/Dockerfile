# Usa la imagen base de Python
FROM python:3.11.1

# Establece el directorio de trabajo en /usr/src/app
WORKDIR /usr/src/app

# Copia el código Python al contenedor
COPY ./code/message_router.py .

# Instala las dependencias necesarias
RUN pip install paho-mqtt

# Espera 10 segundos antes de ejecutar el script Python
CMD ["sh", "-c", "sleep 5 && python -u message_router.py"]
