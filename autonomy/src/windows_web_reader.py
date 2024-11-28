import cv2
import os 

script_dir = os.path.dirname(os.path.abspath(__file__))
save_folder = os.path.join(script_dir, "frames")

os.makedirs(save_folder, exist_ok=True)

cap = cv2.VideoCapture(0)  
frame_count = 0
save_path = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Save each frame as an image
    save_path = os.path.join(save_folder, f"frame_{frame_count}.jpg")
    cv2.imshow("Frames being saved", frame)
    cv2.imwrite(save_path, frame)
    frame_count += 1

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print(str(frame_count) + " frames where saved to " + str(save_folder))

cap.release()
cv2.destroyAllWindows()
