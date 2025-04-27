
import cv2
import numpy as np
import serial
from PIL import Image
import time

last_hand_sent_time = 0  # Track when last "Hand" message was sent
hand_send_interval = 1   # Seconds

ser = serial.Serial('COM3', 2000000)
alive = True
drawing = False
ix, iy = -1, -1
new_bbox = None
reset_bbox = False
even_odd = 0
recording = False
recording_count = 1
prev_even_odd = 0
out_mp4 = None

def drawRectangle(frame, bbox):
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    cv2.circle(frame, (int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2)), 2, (0, 0, 255), 2, 1)

def drawCircle(frame, coord):
    cv2.circle(frame, (int(coord[0]), int(coord[1])), 10, (0, 255, 0), 2, 1)

def drawText(frame, txt, location, color=(50, 170, 50)):
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

# Mouse callback to draw bounding box
def draw_new_bbox(event, x, y, flags, param):
    global ix, iy, drawing, new_bbox, reset_bbox

    if event == cv2.EVENT_LBUTTONDOWN:
        # Define a fixed-size bounding box centered on (x, y)
        box_size = 50  # half of the side length
        x0 = max(x - box_size, 0)
        y0 = max(y - box_size, 0)
        w = h = box_size * 2
        new_bbox = (x0, y0, w, h)
        reset_bbox = True

# Initialize tracker
tracker = cv2.legacy.TrackerMOSSE.create()
video = cv2.VideoCapture(1)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

modelFile  = "models/ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb"
configFile = "models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt"
classFile  = "coco_class_labels.txt"

with open(classFile) as fp:
    labels = fp.read().split("\n")

net = cv2.dnn.readNetFromTensorflow(modelFile, configFile)


def detect_objects(net, im, dim = 300):

    # Create a blob from the image
    blob = cv2.dnn.blobFromImage(im, 1.0, size=(dim, dim), mean=(0, 0, 0), swapRB=True, crop=False)

    # Pass blob to the network
    net.setInput(blob)

    # Peform Prediction
    objects = net.forward()
    return objects

FONTFACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
THICKNESS = 1

def display_text(im, text, x, y):
    # Get text size
    textSize = cv2.getTextSize(text, FONTFACE, FONT_SCALE, THICKNESS)
    dim = textSize[0]
    baseline = textSize[1]

    # Use text size to create a black rectangle
    cv2.rectangle(
        im,
        (x, y - dim[1] - baseline),
        (x + dim[0], y + baseline),
        (0, 0, 0),
        cv2.FILLED,
    )

    # Display text inside the rectangle
    cv2.putText(
        im,
        text,
        (x, y - 5),
        FONTFACE,
        FONT_SCALE,
        (0, 255, 255),
        THICKNESS,
        cv2.LINE_AA,
    )

def display_objects(im, objects, threshold=0.25):

    rows = im.shape[0]
    cols = im.shape[1]

    # For every Detected Object
    for i in range(objects.shape[2]):
        # Find the class and confidence
        classId = int(objects[0, 0, i, 1])
        score = float(objects[0, 0, i, 2])

        # Recover original co-ordinates from normalized coordinates
        x = int(objects[0, 0, i, 3] * cols)
        y = int(objects[0, 0, i, 4] * rows)
        w = int(objects[0, 0, i, 5] * cols - x)
        h = int(objects[0, 0, i, 6] * rows - y)

        # Check if the detection is of good quality
        if score > threshold:
            display_text(im, "{}".format(labels[classId]), x, y)
            cv2.rectangle(im, (x, y), (x + w, y + h), (255, 255, 255), 2)

def warn_for_people(im, objects, threshold=0.5):
    global last_hand_sent_time  # Add this line

    rows = im.shape[0]
    cols = im.shape[1]

    for i in range(objects.shape[2]):
        # Find the class and confidence
        classId = int(objects[0, 0, i, 1])
        score = float(objects[0, 0, i, 2])
        x = int(objects[0, 0, i, 3] * cols)
        y = int(objects[0, 0, i, 4] * rows)
        w = int(objects[0, 0, i, 5] * cols - x)
        h = int(objects[0, 0, i, 6] * rows - y)

        # Check conditions: confidence, class is person, and size > 50x50
        if score > threshold and w > 150 and h > 150 and classId in [1, 2, 3, 4] :
            ser.write(f"{classId},{x+(w/2)}\n".encode())
            # print(f"{labels[classId]},{x+(w/2)} nearby")

        if score > threshold and w > 800 and h > 500 and classId in [1] :
            current_time = time.time()
            if current_time - last_hand_sent_time > hand_send_interval:
                ser.write(f"Hand nearby\n".encode())
                print(f"Hand nearby {w},{h}")
                last_hand_sent_time = current_time
  
win_name = 'Camera Preview'
cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(win_name, draw_new_bbox)

good, frame = video.read()
bbox = (600, 320, 80, 80)
tracker.init(frame, bbox)

frame_width = int(video.get(3))
frame_height = int(video.get(4))

out_mp4 = cv2.VideoWriter("recording_out.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 10, (frame_width, frame_height))


while alive:
    good, frame = video.read()
    if not good:
        break

    rows = frame.shape[0]
    cols = frame.shape[1]

    objects = detect_objects(net, frame)
    display_objects(frame, objects, 0.25)
    warn_for_people(frame, objects, 0.35)

    timer = cv2.getTickCount()
    drawCircle(frame, (640, 360))


    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.isdigit():
                even_odd = int(line)
                print(f"Received: {even_odd}")
        except:
            pass


    # Reset bounding box if a new one was drawn
    if reset_bbox and new_bbox is not None:
        tracker = cv2.TrackerCSRT.create()
        tracker.init(frame, new_bbox)
        bbox = new_bbox
        reset_bbox = False

    ok, bbox = tracker.update(frame)

    key = cv2.waitKey(1)
    if key in [ord("q"), ord("Q"), 13]:
        alive = False
    elif key in [ord("c"), ord("C")]:
        bbox = (600, 320, 80, 80)
        tracker = cv2.TrackerCSRT.create()
        tracker.init(frame, bbox)

    height, width = frame.shape[:2]
    seeker_view = None

    if ok:
        drawRectangle(frame, bbox)
        x, y, w, h = map(int, bbox)
        midx = int(x+(w/2))
        midy = int(y+(h/2))
        y1, y2 = midy-150, midy+150
        x1, x2 = midx-150, midx+150
        cv2.line(frame, (0, midy), (1280, midy), (0, 0, 255), thickness=1)
        cv2.line(frame, (midx, 0), (midx, 720), (0, 0, 255), thickness=1)
        seeker_view = frame[y1:y2, x1:x2]


        ser.write(f"{midx},{midy}\n".encode())

    else:
        drawText(frame, "Tracking failure detected", (315, 60), (0, 0, 255))

        
    drawText(frame, "GOTURN Tracker", (900, 60))
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    drawText(frame, "FPS : " + str(int(fps)), (900, 100))
    drawText(frame, f"Co-ords : {int(bbox[0] + bbox[2] / 2)}, {int(bbox[1] + bbox[3] / 2)}", (900, 140))

    if seeker_view is not None and seeker_view.size > 0:
        inset = cv2.resize(seeker_view, (200, 200))
        frame[420:620, 1020:1220] = inset
        cv2.rectangle(frame, (1020, 420), (1220, 620), (0, 255, 255), 2)

    if even_odd == 1:
        drawText(frame, "Recording", (100, 60))
        if not recording:
            # Start a new recording
            output_filename = f"recording_out{recording_count}.mp4"
            out_mp4 = cv2.VideoWriter(output_filename, cv2.VideoWriter_fourcc(*"mp4v"), 10, (frame_width, frame_height))
            print(f"Started recording: {output_filename}")
            recording = True
            recording_count += 1
        if out_mp4:
            out_mp4.write(frame)

    elif even_odd == 0:
        if recording:
            print("Stopped recording.")
            if out_mp4:
                out_mp4.release()
            out_mp4 = None
            recording = False
        drawText(frame, " ", (100, 60))

    cv2.imshow(win_name, frame)

video.release()
out_mp4.release()
cv2.destroyAllWindows()