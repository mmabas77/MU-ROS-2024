import cv2
import imutils
import math
from object_detection import ObjectDetection

# Initialize Object Detection
od = ObjectDetection()
cap = cv2.VideoCapture("los_angeles.mp4")

# Tracking variables
center_points_prev_frame = []
tracking_objects = {}
track_id = 0
distance_threshold = 20


def get_center(x, y, w, h):
    """Calculate the center point of a bounding box."""
    return int(x + w / 2), int(y + h / 2)


def detect_and_draw_boxes(frame):
    """Detect objects and draw bounding boxes on the frame."""
    (class_ids, scores, boxes) = od.detect(frame)
    center_points_cur_frame = [get_center(x, y, w, h) for (x, y, w, h) in boxes]

    # Draw bounding boxes
    for (x, y, w, h) in boxes:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)

    return center_points_cur_frame


def update_tracking_objects(center_points_cur_frame):
    """Update tracking objects based on current frame's center points."""
    global tracking_objects, track_id

    # Prepare a set of remaining points to assign new IDs
    unmatched_points = set(center_points_cur_frame)
    new_tracking_objects = {}

    # Update existing tracked objects with nearest detected points
    for object_id, prev_point in tracking_objects.items():
        # Find the closest new point
        closest_point = min(
            unmatched_points,
            key=lambda pt: math.hypot(prev_point[0] - pt[0], prev_point[1] - pt[1]),
            default=None
        )

        if closest_point and math.hypot(prev_point[0] - closest_point[0],
                                        prev_point[1] - closest_point[1]) < distance_threshold:
            new_tracking_objects[object_id] = closest_point
            unmatched_points.discard(closest_point)  # Remove matched point

    # Assign new IDs to unmatched points
    for pt in unmatched_points:
        new_tracking_objects[track_id] = pt
        track_id += 1

    tracking_objects = new_tracking_objects


def display_tracking_info(frame):
    """Draw tracking information for each tracked object on the frame."""
    for object_id, pt in tracking_objects.items():
        cv2.circle(frame, pt, 5, (0, 0, 255), -1)
        cv2.putText(frame, str(object_id), (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 1)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = imutils.resize(frame, width=608 , height=608)
    center_points_cur_frame = detect_and_draw_boxes(frame)

    # Only update tracking objects after the first frame
    if center_points_prev_frame:
        update_tracking_objects(center_points_cur_frame)

    display_tracking_info(frame)
    center_points_prev_frame = center_points_cur_frame.copy()

    cv2.imshow("Frame", frame)
    if cv2.waitKey(5) == 27:  # Exit on ESC key
        break

cap.release()
cv2.destroyAllWindows()
