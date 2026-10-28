"""Microphone -> Google speech recognition -> voice/text.

Publishes recognised phrases only; the translator decides what they mean.
Starts disabled unless start_enabled is true; toggle with the voice/enable
service (std_srvs/SetBool).
"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    import speech_recognition as sr
except ImportError:
    sr = None


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice")
        p = self.declare_parameter
        self.device_index = int(p("device_index", -1).value)   # -1 = default microphone
        self.language = p("language", "en-US").value
        self.phrase_time_limit = float(p("phrase_time_limit_s", 4.0).value)
        self.calibration_s = float(p("calibration_s", 1.5).value)
        start_enabled = p("start_enabled", False).value

        self.text_pub = self.create_publisher(String, "voice/text", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "voice/status", latched)
        self.create_service(SetBool, "voice/enable", self._enable)

        self._lock = threading.Lock()
        self._generation = 0
        self._stopper = None
        self.enabled = False

        if sr is None:
            self.get_logger().error("SpeechRecognition is not installed; voice is unavailable")
        else:
            names = sr.Microphone.list_microphone_names()
            self.get_logger().info("Microphones: " + ", ".join(f"[{i}] {n}" for i, n in enumerate(names)))
        self._set_status("off")
        if start_enabled:
            self._set_enabled(True)

    def _set_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _enable(self, request, response):
        response.success, response.message = self._set_enabled(request.data)
        return response

    def _set_enabled(self, on):
        if sr is None:
            return False, "SpeechRecognition is not installed"
        with self._lock:
            if on == self.enabled:
                return True, "voice already " + ("on" if on else "off")
            self.enabled = on
            self._generation += 1
            generation = self._generation
            stopper, self._stopper = self._stopper, None
        if stopper is not None:
            stopper(wait_for_stop=False)
        if on:
            threading.Thread(target=self._start_listening, args=(generation,), daemon=True).start()
            self.get_logger().info("Voice enabled")
            return True, "voice on"
        self._set_status("off")
        self.get_logger().info("Voice disabled")
        return True, "voice off"

    def _start_listening(self, generation):
        recognizer = sr.Recognizer()
        recognizer.dynamic_energy_threshold = True
        recognizer.energy_threshold = 150
        try:
            index = None if self.device_index < 0 else self.device_index
            microphone = sr.Microphone(device_index=index)
            self._set_status("calibrating")
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source, duration=self.calibration_s)
        except Exception as e:  # noqa: BLE001 - any audio failure disables voice
            self.get_logger().error(f"Microphone error: {e}")
            with self._lock:
                if generation == self._generation:
                    self.enabled = False
            self._set_status("error")
            return

        with self._lock:
            if generation != self._generation:
                return
            self._stopper = recognizer.listen_in_background(
                microphone,
                lambda r, audio: self._recognize(r, audio, generation),
                phrase_time_limit=self.phrase_time_limit,
            )
        self._set_status("listening")
        self.get_logger().info("Listening")

    def _recognize(self, recognizer, audio, generation):
        try:
            text = recognizer.recognize_google(audio, language=self.language)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            self.get_logger().error(f"Google speech API error: {e}")
            return
        if generation != self._generation or not text or not rclpy.ok():
            return
        self.get_logger().info(f"Heard: {text}")
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

    def shutdown(self):
        with self._lock:
            self._generation += 1
            stopper, self._stopper = self._stopper, None
        if stopper is not None:
            stopper(wait_for_stop=False)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
