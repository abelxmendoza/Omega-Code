import threading
import time
from ultrasonic_sensor import Ultrasonic

ultrasonic = Ultrasonic()

def run_ultrasonic():
    try:
        while True:
            distance = ultrasonic.get_distance()
            print(f'Distance: {distance} cm')
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    ultrasonic_thread = threading.Thread(target=run_ultrasonic)
    ultrasonic_thread.start()
    time.sleep(10)
    print("Stopping ultrasonic thread")
    stop_thread(ultrasonic_thread)
