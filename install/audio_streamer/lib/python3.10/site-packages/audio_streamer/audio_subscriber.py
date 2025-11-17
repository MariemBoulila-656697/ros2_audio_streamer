import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData, AudioInfo
import numpy as np
from scipy.io import wavfile
import os


class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')

        # --- Dichiarazione Parametri ---
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('channels', 4)  # Deve corrispondere al Publisher
        self.declare_parameter('base_stream_topic', "audio_stream")
        self.declare_parameter('base_info_topic', "audio_info")
        self.declare_parameter('output_directory', "recorded_audio")
        self.declare_parameter('output_base_name', "output_mic")
        self.declare_parameter('save_to_disk', True)
	
	#prende i parametri dal file yaml se non le trova usa quelli de default 
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.base_stream_topic = self.get_parameter('base_stream_topic').value
        self.base_info_topic = self.get_parameter('base_info_topic').value
        self.output_directory = self.get_parameter('output_directory').value
        self.output_base_name = self.get_parameter('output_base_name').value
        self.save_to_disk = self.get_parameter('save_to_disk').value

        # --- Strutture Dati per la Registrazione ---
        # Lista di buffer per immagazzinare i dati (bytes) di ogni singolo microfono
        self.mic_buffers = [bytearray() for _ in range(self.channels)]

        # --- Sottoscrizioni Dinamiche ---
        self.stream_subscribers = []
        self.info_subscribers = []

        for i in range(self.channels):
            mic_id = f"mic{i + 1}"

            # Topic Stream (Dati Audio)
            stream_topic = f"{self.base_stream_topic}/{mic_id}"
            stream_sub = self.create_subscription(
                AudioData,
                stream_topic,
                # Uso di lambda per passare l'indice 'i' al callback
                lambda msg, idx=i: self.audio_callback(msg, idx),
                10
            )
            self.stream_subscribers.append(stream_sub)

            # Topic Info (Metadati - solo per ricevere la prima info)
            info_topic = f"{self.base_info_topic}/{mic_id}"
            info_sub = self.create_subscription(
                AudioInfo,
                info_topic,
                lambda msg, idx=i: self.info_callback(msg, idx),
                10
            )
            self.info_subscribers.append(info_sub)

            self.get_logger().info(f" In ascolto su Stream: {stream_topic} e Info: {info_topic}")

        self.get_logger().info(f" Subscriber pronto. Premi CTRL+C per salvare {self.channels} file audio.")


    def info_callback(self, msg, mic_index):
        # Riceve le info ma non fa nulla di essenziale, serve per la tracciabilità
        if mic_index == 0:  # Stampa solo una volta per non inondare il log
            self.sample_rate = msg.sample_rate  # Aggiorna la sample rate per sicurezza
            self.get_logger().debug(f"Info ricevute per Mic {mic_index + 1}. Sample Rate: {self.sample_rate}")

    def audio_callback(self, msg, mic_index):
        # Appende i bytes ricevuti al buffer del microfono corretto
        self.mic_buffers[mic_index].extend(msg.data)
        self.get_logger().debug(
            f"Dati ricevuti per Mic {mic_index + 1}. Dimensione buffer: {len(self.mic_buffers[mic_index])}")

    def save_audio_files(self):
        """Salva tutti i buffer in file WAV separati."""

        # Crea la cartella di output se non esiste
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        self.get_logger().info(f"Tentativo di salvare {self.channels} file nella directory: {self.output_directory}")

        for i in range(self.channels):
            buffer_data = self.mic_buffers[i]
            if len(buffer_data) > 0:
                # 1. Converte i bytes in un array NumPy di float32
                # Il publisher inviava float32, quindi riconvertiamo
                audio_np = np.frombuffer(buffer_data, dtype=np.float32)

                # 2. Genera il nome del file (es. recorded_audio/output_mic1.wav)
                output_path = os.path.join(self.output_directory, f"{self.output_base_name}{i + 1}.wav")

                # 3. Salva come file WAV
                try:
                    wavfile.write(output_path, self.sample_rate, audio_np)
                    self.get_logger().info(f"Salvato Mic {i + 1} ({len(audio_np)} samples) in: {output_path}")
                except Exception as e:
                    self.get_logger().error(f"Errore nel salvataggio del Mic {i + 1}: {e}")
            else:
                self.get_logger().warn(f"Buffer Mic {i + 1} vuoto, nessun dato da salvare.")


def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Subscriber interrotto. Avvio salvataggio file...")
    finally:
        if node.save_to_disk: 
            node.get_logger().info("💾 Salvataggio dei file WAV...") 
            # Assicurati che il nome della tua funzione di salvataggio sia 'save_all_waves' 
            node.save_all_waves() 
            node.get_logger().info("✅ Salvataggio completato.") 
            
        else: 
            node.get_logger().info("🚫 Salvataggio omesso (SAVE=false).") 
    
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
