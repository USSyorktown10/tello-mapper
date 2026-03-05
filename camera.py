from djitellopy import Tello 
import cv2
import time
from threading import Thread
import numpy as np

speed = 25
command_time_seconds= 3 

print("create tello object")
tello = Tello()

print("connect to drone")
tello.connect()

battery_level = tello.get_battery()
print(f"battery percentage: {battery_level}")

time.sleep(2)

print("turn stream on")
tello.streamon()

print("read tello image")
frame_read = tello.get_frame_read()

def estimate_distance(known_width, focal_length, pixel_width):
    # triangle similarity: (object_real_width / object_pixel_width) = (distance / focal_length)
    if pixel_width == 0:
        return None
    return (known_width * focal_length) / pixel_width

# Load YOLO classes, config, and weights before the loop
classes = open('coco.names').read().strip().split('\n')
net = cv2.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')
ln = net.getLayerNames()
ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]

# FOCAL_LENGTH and known widths need to be measured/calibrated for your Tello camera
FOCAL_LENGTH = 700  # Example: pixels (replace with your value from calibration)
KNOWN_WIDTHS = {
    "person": 0.45,  # meters (average human shoulder width)
    "car": 1.8,      # meters (average car width)
    # Add more classes and widths as appropriate. Im too lazy for that
}

print('press q to quit')
while True:
   tello_video_image = frame_read.frame

   if tello_video_image is not None:
      tello_video_image = cv2.cvtColor(frame_read.frame, cv2.COLOR_RGB2BGR)
      if tello_video_image is not None:
        # Preprocess image for YOLO
        blob = cv2.dnn.blobFromImage(tello_video_image, 1/255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(ln)
        H, W = tello_video_image.shape[:2]
        boxes = []
        confidences = []
        classIDs = []

        # Post-process YOLO detections
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                if confidence > 0.5:
                    box = detection[0:4] * np.array([W, H, W, H])
                    centerX, centerY, width, height = box.astype('int')
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Draw boxes on detections
        if len(idxs) > 0:
            for i in idxs.flatten():
                x, y, w, h = boxes[i]
                color = (0, 255, 0)
                class_name = classes[classIDs[i]]
                confidence = confidences[i]
                box_width = w

                # Look up real width for this class, or use a default if unavailable
                real_width = KNOWN_WIDTHS.get(class_name, 0.3)  # Default 0.3 meters

                distance = estimate_distance(real_width, FOCAL_LENGTH, box_width)
                if distance is not None:
                    display_distance = f"{distance:.2f}m"
                else:
                    display_distance = "?"

                text = f"{class_name}: {confidence:.2f} {display_distance}"
                cv2.rectangle(tello_video_image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    tello_video_image, text, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

        cv2.imshow("tellovideo", tello_video_image)



   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

tello.land()

time.sleep(2)

tello.streamoff()

cv2.destroyWindow("tellovideo")

cv2.destroyAllWindows()
