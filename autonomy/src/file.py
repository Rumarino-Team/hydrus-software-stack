import numpy as np 
import cv2
import custom_types
from ultralytics import YOLO 
from typing import List
from motpy import MultiObjectTracker, Detection

model = YOLO("yolo11n.pt")
tracker = MultiObjectTracker(
    dt=0.1,
    tracker_kwargs={'max_staleness': 5}
)

def yolo_object_detection(image: np.ndarray) -> List[Detection]:
    result_list = []
    results = model(image)  # This returns a list of results

    for result in results:
        if hasattr(result, 'boxes'):  # Ensure the result has boxes
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                conf = float(box.conf.cpu().numpy()[0])  # Convert to float
                if conf > 0.5:
                     result_list.append(Detection(box=[x1, y1, x2, y2], score=conf))

    # Motpy tracking
    tracked_objects = tracker.step(result_list)

    # Annotate frame with tracking results
    for obj in tracked_objects:
        x1, y1, x2, y2 = map(int, obj.box)
        track_id = obj.id
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, f"ID: {track_id}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image

def main():
    cap = cv2.VideoCapture(0)  # Replace with video file path if needed

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Detect and track objects
        frame = yolo_object_detection(frame)

        # Show the output frame
        cv2.imshow("YOLO + Motpy", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()