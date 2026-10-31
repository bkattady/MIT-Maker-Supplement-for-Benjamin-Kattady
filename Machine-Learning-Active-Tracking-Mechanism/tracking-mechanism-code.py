import time

system_start_time = time.time()

import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2
from gpiozero import Servo

servo = Servo(18)
servo_pos = 0.0
servo.value = servo_pos

DEADZONE_X = 15     
STEP_SIZE = 0.015   
TARGET_CONF = 0.90   

total_servo_time = 0.0
time.sleep(1.0)

session = ort.InferenceSession("best.onnx", providers=['CPUExecutionProvider'])

input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
img_size = input_shape[2] if len(input_shape) == 4 else 640

picam2 = Picamera2()
picam2.start()

target_locked = False

try:
    while not target_locked:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        h, w, _ = frame_bgr.shape
        center_x_screen = w // 2

        img_resized = cv2.resize(frame_bgr, (img_size, img_size))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        input_tensor = img_rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        input_tensor = np.expand_dims(input_tensor, axis=0)

        outputs = session.run(None, {input_name: input_tensor})
        predictions = np.squeeze(outputs[0]).T

        scores = np.max(predictions[:, 4:], axis=1) if predictions.shape[1] > 4 else predictions[:, 4]
        best_idx = np.argmax(scores)
        confidence = scores[best_idx]

        if confidence >= TARGET_CONF:
            target_locked = True

            sun_x = int(predictions[best_idx, 0] * (w / img_size))
            sun_y = int(predictions[best_idx, 1] * (h / img_size))
            error_x = sun_x - center_x_screen
            max_possible_error = w / 2
            alignment_pct = max(0.0, (1.0 - (abs(error_x) / max_possible_error))) * 100
            

            print("Confidence Reached")
            print(f"Alignment Accuracy: {alignment_pct:.1f}%")
            print(f"Total Motor Active Time: {total_servo_time:.4f} seconds")

        elif confidence > 0.40:  
            servo.value = servo_pos

            sun_x = int(predictions[best_idx, 0] * (w / img_size))
            error_x = sun_x - center_x_screen

            max_possible_error = w / 2
            alignment_pct = max(0.0, (1.0 - (abs(error_x) / max_possible_error))) * 100

            if abs(error_x) > DEADZONE_X:
                move_start = time.time()

                if error_x > 0:
                    servo_pos += STEP_SIZE
                else:
                    servo_pos -= STEP_SIZE

                servo_pos = max(-1.0, min(1.0, servo_pos))
                servo.value = servo_pos

                time.sleep(0.02)
                total_servo_time += (time.time() - move_start)

            print(f"Sun Detected, Alignment: {alignment_pct:.1f}%, Motor Time: {total_servo_time:.3f}s")

        else:
            time.sleep(0.02)

except KeyboardInterrupt:
    print("\nTracking stopped.")

finally:
    try:
        servo.detach()
    except Exception:
        pass
    picam2.stop()
