# Workspace/Build
# Struttura del Workspace e Istruzioni di Build del Pacchetto

## Installazione librerie PYTHON 
assicurarsi che il gestore di pacchetti Python (pip3) sia installato sul sistema
```sudo apt update
sudo apt install python3-pip
```
installazione delle librerie python
```
pip3 install numpy sounddevice scipy matplotlib
```
Strumenti di compilazione Ros2 
```sudo apt install python3-colcon-common-extensions python3-rosdep```

## Creazione della cartella di lavoro
Preparazione area di lavoro 
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws 
```
Sorgere l'ambiente ROS2 
```
source /opt/ros/humble/setup.bash
```
Creare pacchetto ROS2 
creazione del pacchetto Python chiamato audio_streamer all'interno della cartella src/
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python audio_streamer
```
Creare le cartelle necessarie 
cartella: launch: serve per il lancio del programma 
cartella: config: dove inseriremo il file YAML che contiene tutte le informazioni della Scarlett necessarie per il funzionamento del programma
```
mkdir -p ~/ros2_ws/src/audio_streamer/launch
mkdir -p ~/ros2_ws/src/audio_streamer/config
```
Creare due file python audio_publisher.py e audio_subscriber.py all'interno della directory
```
~/ros2_ws/src/audio_streamer/audio_streamer/
```
Creare il file params.yaml nella directory 
```
~/ros2_ws/src/audio_streamer/config/
```
Creare il file streaming_launch.py nella directory
```
~/ros2_ws/src/audio_streamer/launch/
```
Configuareazione dei manifesti 
Modifica package.xml aggiungendo 
```
<depend>rclpy</depend> 
<depend>audio_common_msgs</depend>
 ```
Setup Eseguibili 
Modifica setup.py per aggiungere gli entry points dei due nodi
 ``` 
    entry_points={
        'console_scripts': [
            'audio_pub = audio_streamer.audio_publisher:main',
            'audio_sub = audio_streamer.audio_subscriber:main',        ],
    },
 ```
## Lancio del programma

Compilazione e lancio (con i parametri definiti in params.yaml) utilizzando il file di launch:
```
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch audio_streamer streaming_launch.py
```
