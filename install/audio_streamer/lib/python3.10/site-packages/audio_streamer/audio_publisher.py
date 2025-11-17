#qui abbiamo che ogni microfono pubblica su un topic diverso e non sullo stesso
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData, AudioInfo
import sounddevice as sd
import numpy as np


class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        # --- Dichiarazione Parametri ---
        self.declare_parameter('device_index', 1)
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('channels', 4)  # Assumiamo 4 canali
        self.declare_parameter('chunk_duration', 0.1)
        self.declare_parameter('device_name', "Scarlett 18i16")

        #self.declare_parameter('publish_audio_topic', "audio_stream")
        #self.declare_parameter('publish_info_topic', "audio_info")
        # Non dichiariamo più i topic come parametri singoli, saranno generati
        self.declare_parameter('base_stream_topic', "audio_stream")
        self.declare_parameter('base_info_topic', "audio_info")

        self.device_index = self.get_parameter('device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.device_name = self.get_parameter('device_name').value
        self.base_stream_topic = self.get_parameter('base_stream_topic').value
        self.base_info_topic = self.get_parameter('base_info_topic').value
        self.get_logger().info("1")
        # --- Publishers Dinamici (UN PUBLISHER PER OGNI CANALE/INFO) ---
        self.audio_stream_publishers = []
        self.audio_info_publishers = []
        self.get_logger().info("1")
        for i in range(self.channels):
            mic_id = f"mic{i + 1}"

            # Topic per lo Stream (Dati Audio)
            stream_topic = f"{self.base_stream_topic}/{mic_id}"
            stream_pub = self.create_publisher(AudioData, stream_topic, 10)
            self.audio_stream_publishers.append(stream_pub)

            # Topic per le Info (Metadati)
            info_topic = f"{self.base_info_topic}/{mic_id}"
            info_pub = self.create_publisher(AudioInfo, info_topic, 10)
            self.audio_info_publishers.append(info_pub)

            self.get_logger().info(f"🎤 Canale {i + 1} (Mic {i + 1}) -> Stream: {stream_topic} | Info: {info_topic}")
        self.get_logger().info("3")
        # --- Stream Audio ---
        #self.chunk_size = int(self.sample_rate * self.chunk_duration)
        self.stream = sd.InputStream(samplerate=self.sample_rate,
                                     channels=self.channels,
                                     dtype='float32',
                                     device=self.device_index,
                                     callback=self.audio_stream_callback,
                                     blocksize=int(self.sample_rate * self.chunk_duration)
                                     )
        self.get_logger().info("4")
        self.stream.start()

        # Timer per pubblicazione
        #self.timer = self.create_timer(self.chunk_duration, self.timer_callback)
        self.get_logger().info(f"🎙️ Acquisizione avviata con {self.channels} canali.")
        self.get_logger().info("5")
    def audio_stream_callback(self, indata, frames, time, status):
        """Funzione chiamata da sounddevice ogni volta che un blocco è pronto.""" 
        if status: 
        # Qui gestiamo l'eventuale warning di overflow dal driver 
            self.get_logger().warn(f"Buffer audio status warning: {status}") 
            
        # La logica di demultiplazione rimane identica 
        for i in range(self.channels):
            # indata è il blocco di dati, frames è il numero di campioni 
            channel_data = indata[:, i] 
            
            # Pubblicazione Audio Stream (AudioData) 
            audio_msg = AudioData() 
            audio_msg.data = channel_data.tobytes() 
            self.audio_stream_publishers[i].publish(audio_msg) 
            
            # Pubblicazione Audio Info (AudioInfo) 
            info_msg = AudioInfo() 
            info_msg.sample_rate = self.sample_rate 
            info_msg.channels = 1 
            info_msg.sample_format = "float32" 
            info_msg.coding_format = "PCM" 
            self.audio_info_publishers[i].publish(info_msg) 



def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Publisher interrotto. Spegnimento stream audio.")
    finally:
        node.stream.stop()  # FERMA LO STREAM sounddevice
        node.stream.close()  # CHIUDE LO STREAM
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
