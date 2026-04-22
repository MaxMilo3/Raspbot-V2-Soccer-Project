from ultralytics import YOLO
#import cv2

#image_loc = "dataset/images/train/capture_20.jpg"
#img = cv2.imread(image_loc)


def get_detections(img, model = YOLO("image_detection/runs/detect/train/weights/best.pt"),con = 0.55):
    results = model(img, conf = con, verbose = False)
    
    num_balls = 0
    ball_detections = []
    num_posts = 0
    post_detections = []

    for r in results:
        h, w = r.orig_shape

        for box in r.boxes:
            cls = int(box.cls[0])
            x1, y1, x2, y2 = box.xyxy[0]

            x_center = float((x1 + x2) / 2)
            y_center = float((y1 + y2) / 2)

            if cls == 0:
                ball_detections.append((x_center / w, y_center / h))

                num_balls += 1
            elif cls == 1:
                post_detections.append((x_center / w, y_center / h))
                num_posts += 1

    return num_balls, ball_detections, num_posts, post_detections

def detect_ball(img, model = YOLO("image_detection/runs/detect/train/weights/best.pt"),con = 0.8):
    results = model(img, conf = con, verbose = False)
    
    num_balls = 0
    ball_detections = []
    ball_widths = []

    for r in results:
        h, w = r.orig_shape

        for box in r.boxes:
            cls = int(box.cls[0])
            x1, y1, x2, y2 = box.xyxy[0]

            x_center = float((x1 + x2) / 2)
            y_center = float((y1 + y2) / 2)

            if cls == 0:
                ball_detections.append(x_center / w)
                ball_widths.append(float(abs(x1 - x2)/w))
                num_balls += 1

    return num_balls, ball_detections, ball_widths

"""
num_balls, ball_detections, ball_widths = detect_ball(img)

print(ball_widths[0])
"""
