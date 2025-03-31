import math
import time
import paho.mqtt.publish as publish
from PyQt5.QtCore import QTimer, QObject
from PyQt5.QtWidgets import QApplication
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class MQTTPublisher(QObject):
    def __init__(self, broker, topics):
        super().__init__()
        self.broker = broker
        self.topics = topics if isinstance(topics, list) else [topics]  # Ensure topics is a list
        self.count = 0

        # Sine wave parameters
        self.frequency = 25# Frequency of the sine wave in Hz
        self.amplitude = (46537 - 16390) / 2  # Amplitude of the sine wave
        self.offset = (46537 + 16390) / 2  # Center the sine wave
        self.sample_rate = 1024  # Samples per second
        self.time_per_message = 1.0  # 1 second per message
        self.current_time = 0.0  # Running time for continuity

        # Set up the timer to publish every 1 second
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_message)
        self.timer.start(1000)  # 1000ms = 1 second

    def publish_message(self):
        if self.count < 50:  # Limit to 500 messages (or adjust accordingly)
            values = []
            # Generate 1024 samples for 1 second of data
            for i in range(self.sample_rate):
                t = self.current_time + (i / self.sample_rate)  # Continuous time progression
                value = self.offset + self.amplitude * math.sin(2 * math.pi * self.frequency * t)
                values.append(round(value, 2))

            # Increment time for the next message
            self.current_time += 1  # 1 second per message

            # Format the message as a comma-separated string
            message = ",".join(map(str, values))

            for topic in self.topics:  # Publish to all specified topics
                try:
                    publish.single(topic, message, hostname=self.broker, qos=1)
                    logging.info(f"[{self.count}] Published to {topic}: {message[:50]}... ({self.sample_rate} values)")
                except Exception as e:
                    logging.error(f"Failed to publish to {topic}: {str(e)}")

            self.count += 1
        else:
            self.timer.stop()
            logging.info("Publishing stopped after 500 messages.")

if __name__ == "__main__":
    app = QApplication([])  # PyQt5 QApplication instance
    broker = "192.168.1.173"  # Replace with your actual MQTT broker IP
    topics = ["sarayu/tag2/topic2|m/s"]  # List of topics (can add more)
    mqtt_publisher = MQTTPublisher(broker, topics)
    app.exec_()
