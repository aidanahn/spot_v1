from picamera2 import Picamera2, Preview
from time import sleep
import cv2

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start()


while True:
    im = picam2.capture_array()
    cv2.imshow("preview", im)

    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
