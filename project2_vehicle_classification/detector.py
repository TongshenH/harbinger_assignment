import os
import sys

current = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, current + "/materials")

import cv2
import torch

video_path = f'{sys.path[0]}/backup.mkv'
model_path = f'{sys.path[0]}/best.pt'


class Detector:
    def __init__(self, v_fp, m_fp):
        # Initialize detector parameters
        self.v_fp = v_fp
        self.m_fp = m_fp
        self.model = None

    def detect_model(self):
        """
        # Load YOLOv5 model with pre-trained weights
        """
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.m_fp, force_reload=True)
        ckpt = torch.load(self.m_fp)
        self.model.names = ckpt['model'].names

    def visualization(self, save=False):
        """
        Visualize backup video
        """
        cap = cv2.VideoCapture(self.v_fp)

        # Define video writer for output video with bounding boxes
        width = int(cap.get(3))
        height = int(cap.get(4))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('Integration.mp4', fourcc, 20.0, (width, height))

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # Perform inference
                results = self.model(frame)
                # Draw bounding boxes on the frame
                frame = results.render()[0]
                # Display the frame
                cv2.imshow('YOLOv5 Object Detection', frame)
                if save:
                    out.write(frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

        # Release resources
        cap.release()
        out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detect = Detector(video_path, model_path)
    detect.detect_model()
    detect.visualization(save=True)
