# Robotica Movil 2023 - Trabajo Practico 3


## Archivos

Existe un script para descargar automáticamente los archivos de calibración y correr la conversión a Rosbag2:

```
# Se debe encontrar en el root del repositorio

# Primero creamos un entorno virtual para python3
sudo apt install python3-venv && \
    python3 -m venv venv && \
    . venv/bin/activate && \
    pip3 install -r requirements.txt

# Luego ejecutamos la descarga de los archivos de calibración
./download_calibration.sh

# Finalmente ejecutamos el script de conversión a rosbag2
python3 create_calibration_rosbag2.py && \
    deactivate
```

Los datasets que no conicernen a calibración se encuentran en formato ROSBAG2 directamente desde la página oficial:  
https://docs.openvins.com/gs-datasets.html#gs-data-euroc  
y se deben descargar y descomprimir dentro de la carpeta `EuRoC/`


## Docker
El archivo `Dockerfile` provee comandos para levantar un contenedor Docker.
Para compilar el contenedor se tiene que tener Docker instalado ([instrucciones](https://docs.docker.com/engine/install/ubuntu/)) y correr el siguiente comando:
```bash
docker build --build-arg USER_ID=$(id -u ${USER}) --build-arg GROUP_ID=$(id -g ${USER}) --rm -t "ros2:humble_with_gazebo" .
```
(donde `--rm` remueve contenedores intermedios luego de terminar la compilacián, y `-t` le da un nombre a nuestro contenedor).

Luego se puede levantar el contenedor:
- en forma headless (sin gráficos)
```bash
docker run --rm -it --net=host --volume="`pwd`:/root/dev_ws:rw" ros2:humble_with_gazebo
```
- o con interfaz gráfica:
```bash
xhost +local:root
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo
xhost -local:root
```

Luego de levantado el contenedor podemos conectarnos con multiples terminales de la siguiente forma:
```bash
docker exec -it `docker ps | awk '/ros2:humble_with_gazebo/ { print $1 }'` bash
```

Para salir de una terminal se puede terminar la misma con C^D o con el comando `exit`.

_(se agradece a Roman Comelli por sus contribuciones respecto a la creación del contenedor)_


## Ejecución

To-Do
