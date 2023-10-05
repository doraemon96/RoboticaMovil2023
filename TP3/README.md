# Robotica Movil 2023 - Trabajo Practico 3

## Docker
El archivo `Dockerfile` provee comandos para crear una imagen en Docker.
Para compilar la imagen se tiene que tener Docker instalado ([instrucciones](https://docs.docker.com/engine/install/ubuntu/)) y correr el siguiente comando:
```bash
docker build --build-arg USER_ID=$(id -u ${USER}) --build-arg GROUP_ID=$(id -g ${USER}) --rm -t "ros2:humble_with_gazebo" .
```
(donde `--rm` remueve contenedores intermedios luego de terminar la compilación, y `-t` le da un nombre a nuestra imagen).

Luego, se puede levantar el contenedor:
- en forma headless (sin gráficos)
```bash
docker run --rm -it --net=host --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo
```
- o con interfaz gráfica:
```bash
xhost +local:root &&\
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo &&\
xhost -local:root 
```

Luego de levantado el contenedor podemos conectarnos con múltiples terminales de la siguiente forma:
```bash
docker exec -it `docker ps | awk '/ros2:humble_with_gazebo/ { print $1 }'` bash
```

Para salir de una terminal se puede terminar la misma con C^D o con el comando `exit`.

_(se agradece a Roman Comelli por sus contribuciones respecto a la creación del contenedor)_


## Calibración

Existe un script para descargar automáticamente los archivos de calibración y correr la conversión a Rosbag2:

```
# Se debe encontrar en el root del repositorio

# Primero creamos un entorno virtual para python3
sudo apt update && \
    sudo apt install -y python3-venv && \
    python3 -m venv venv && \
    . venv/bin/activate && \
    pip3 install -r requirements.txt

# Luego ejecutamos la descarga de los archivos de calibración
./download_calibration.sh

# Finalmente ejecutamos el script de conversión a rosbag2
python3 create_calibration_rosbag2.py && \
    deactivate
```

Para calibrar, corremos la rutina de calibración desde una terminal en el contenedor:
```bash
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 7x6 --square 0.108 --ros-args -r right:=/cam0 -r left:=/cam1 -r right_camera:=/cam0 -r left_camera:=/cam1
```
y en otra terminal del contenedor reproducimos nuestro rosbag
```bash
ros2 bag play --disable-keyboard-controls ./EuRoC/cam_checkerboard/cam_checkerboard_rosbag2
```
Deberíamos terminar con más de 100 samples para calibrar. Luego, pulsamos "Calibrate". 
Tener en cuenta que la calibración puede durar varios minutos y la ventana puede dejar de responder (recomendamos tener paciencia).

Finalmente, clickeamos "Save" para guardar los datos, y ejecutamos el comando:
```bash
cd EuRoC/cam_checkerboard &&\
mv /tmp/calibrationdata.tar.gz . &&\
tar -xf calibrationdata.tar.gz right.yaml left.yaml
```
para mover los archivos generados y extraer lo necesario.