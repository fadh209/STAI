import cv2
import RPi.GPIO as GPIO
import time
from ultralytics import YOLO

# Inisialisasi model YOLO
model = YOLO('best.pt')  # Path ke model YOLO yang telah Anda latih

# Inisialisasi kamera
cap = cv2.VideoCapture(0)  # Sesuaikan indeks kamera jika perlu
frame_width = 1920  # Sesuaikan dengan resolusi kamera Anda (misalnya, 1080p)
frame_height = 1080

# Inisialisasi GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Servo horizontal
GPIO.setup(27, GPIO.OUT)  # Servo vertikal
GPIO.setup(22, GPIO.OUT)  # Laser kontrol

horizontal_servo = GPIO.PWM(17, 50)  # 50 Hz frequency
vertical_servo = GPIO.PWM(27, 50)

# Inisialisasi posisi servo di tengah
horizontal_servo.start(7.5)  # Posisi 0 derajat (tengah)
vertical_servo.start(2.5)    # Posisi 0 derajat (bawah)

# Fungsi untuk mengubah koordinat deteksi ke sudut servo
def map_position_to_servo(x, y, frame_width, frame_height):
    # Mengonversi posisi x ke sudut horizontal (-90 ke +90)
    horizontal_angle = ((x / frame_width) * 180) - 90  # Sudut dalam derajat
    vertical_angle = (y / frame_height) * 45  # Sudut vertikal (0 ke 45)
    return horizontal_angle, vertical_angle

def set_servo_angle(servo, angle):
    duty_cycle = (angle / 18) + 2.5  # Mengonversi derajat ke duty cycle
    servo.ChangeDutyCycle(duty_cycle)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Lakukan deteksi dengan model
        results = model(frame)

        bird_detected = False
        for result in results:
            for obj in result.boxes:
                if obj.cls == 'bird':  # Pastikan 'bird' sesuai dengan label model
                    bird_detected = True
                    x_center = (obj.xmin + obj.xmax) / 2
                    y_center = (obj.ymin + obj.ymax) / 2

                    # Hitung sudut servo untuk mengikuti posisi burung
                    horizontal_angle, vertical_angle = map_position_to_servo(
                        x_center, y_center, frame_width, frame_height
                    )

                    # Pastikan sudut berada dalam batas pergerakan servo
                    if -90 <= horizontal_angle <= 90:
                        set_servo_angle(horizontal_servo, horizontal_angle)

                    if 0 <= vertical_angle <= 45:
                        set_servo_angle(vertical_servo, vertical_angle)

                    GPIO.output(22, GPIO.HIGH)  # Nyalakan laser
                    time.sleep(0.5)  # Jeda untuk stabilitas pergerakan servo
                    break

        if not bird_detected:
            # Kembali ke posisi awal (0 derajat horizontal dan vertikal)
            set_servo_angle(horizontal_servo, 0)  # 0 derajat untuk horizontal
            set_servo_angle(vertical_servo, 0)    # 0 derajat untuk vertikal
            GPIO.output(22, GPIO.LOW)  # Matikan laser
            time.sleep(3)  # Tunggu 3 detik sebelum kembali ke patroli

except KeyboardInterrupt:
    horizontal_servo.stop()
    vertical_servo.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
