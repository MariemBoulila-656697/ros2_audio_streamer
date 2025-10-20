# Hardware/ROS2 
# ROS 2 Multicanale: Setup Hardware e Installazione ROS

### Descrizione 
il progetto implementa un sistema ROS2 per l'acquisizione la riproduzione e registrazione di flussi audio multicanali da un dispositivo hardware esterno **Focusrite Scarlett 18i16 4th Gen**.

## Connessione
+ alimentazione esterna della Scerlett
    + collegare l'alimentatore fornito alla porta **15V** sul pannello posteriore
+ accensione della Scarelett
+ collegare la Scarlett tramite la porta USB al proprio computer
+ collegare i microfoni a condensatore a uno qualsiasi degli slot di ingresso disponibili
+ attivare l'alimentazione Phantom Power **+48V** premendo il pulsante sul pannello frontale 
## Architettura ROS2 
+ verificare l'installazione di ROS2 tramite un comondo base
```ros2 run turtlesim turtlesim_node```
+ se si apre la finiestra con la tartaruga si può saltare il passo successivo 
## Installazione ROS2 
Fase 1: Configurazione dei Repository: questa fase prepara il sistema ad accettare pacchetti da fonti ROS
senza chiave GPG il gestore di pacchetti **apt** non saprebbe dove trovare i file di installazione 
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
```
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
aggiungere la reposetry alla lista APT 
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Fase 2 : Installazione dei Pacchetti ROS
```
sudo apt update
```
```
sudo apt upgrade 
sudo apt install ros-humble-ros-base
```
installazione dei pacchetti per messaggi audio
```
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/audio_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Fase 3: Configurazione dell'ambiente 
Questo carica le variabili d'ambiente di ROS 2 nel terminale corrente

```
source /opt/ros/humble/setup.bash 
```
per non dover ripetere il comando ad ogni nuova finestra del terminale
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

