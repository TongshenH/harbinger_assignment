import sys
import pyzed.sl as sl
import cv2
from signal import signal, SIGINT
import argparse
import os

current = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, current + "/materials/backup.svo")
save_fp = f"{current}/materials/backup_zed.avi"

zed = sl.Camera()

# Set SVO path for playback
input_path = sys.path[0]
init_parameters = sl.InitParameters()
init_parameters.set_from_svo_file(input_path)

# Open the ZED
err = zed.open(init_parameters)

svo_image = sl.Mat()
image_size = zed.get_camera_information().camera_resolution
width = image_size.width
height = image_size.height

while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Read side by side frames stored in the SVO
        zed.retrieve_image(svo_image, sl.VIEW.LEFT)
        # Get frame count
        svo_position = zed.get_svo_position()
        left_image_data = svo_image.get_data()

        image_rgb = cv2.cvtColor(left_image_data, cv2.COLOR_RGBA2RGB)

        # Create video writer with MPEG-4 part 2 codec
        video_writer = cv2.VideoWriter(save_fp, cv2.VideoWriter_fourcc("MP4V"),
                                       30, (width, height))
        video_writer.write(image_rgb)

    elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
        print("SVO end has been reached. Looping back to first frame")
        zed.set_svo_position(0)
