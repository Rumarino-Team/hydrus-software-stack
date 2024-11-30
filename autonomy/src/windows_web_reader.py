import cv2
import os 

script_dir = os.path.dirname(os.path.abspath(__file__))
newFolder = os.path.join(script_dir, "Videos")
os.makedirs(newFolder, exist_ok=True)
video_count = len([f for f in os.listdir(newFolder) if f.startswith("video") and f.endswith(".mp4")])

video_save_path = os.path.join(newFolder, f"video{video_count + 1}.mp4")  # Path for the video

cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30  # Default to 30 if FPS is not detected

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter(video_save_path, fourcc, fps, (frame_width, frame_height))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Recording", frame)

    # Write the frame to the video file
    out.write(frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print(f"Video saved to {video_save_path}")

cap.release()
out.release()
cv2.destroyAllWindows()
