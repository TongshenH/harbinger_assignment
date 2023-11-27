import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, current + "/materials")

import cv2
import numpy as np
import tty
import termios
import matplotlib.pyplot as plt

fp = f"{sys.path[0]}/backup_zed_10.avi"


class BackupTraj:

    def __init__(self, file_path):
        # Initialize vehicle parameters
        self.W = 1.2
        self.L = 4.5
        self.max_str = 45

        self.fp = file_path

        # Desired steering angle
        self.delta = None
        # the lookahead distance of vehicle from back up camera
        self.lookahead_len = 6
        self.yl = None
        self.yr = None
        self.traj_w4 = None
        self.traj_w2 = None
        self.traj_w = None
        self.baseline = None

        # Keyboard control
        self.fd = sys.stdin
        self.tty_settings = termios.tcgetattr(self.fd)

    def str_input(self):
        """"Get the desired steering angle in radians from terminal
        """
        self.delta = input("Type your desired steering angle (deg):")
        self.delta = np.radians(float(self.delta))

    def ackerman_model(self):
        """
        Calculate the rear wheel steering radius, r3 represents the rear left wheel, and
        r4 represents the rear right wheel, turn right is positive
        :return: left and right rear wheel steering radius
        """
        r1 = self.L / np.tan(self.delta)
        r4 = np.sqrt(r1 ** 2 - self.L ** 2)
        r3 = r4 + self.W
        if self.delta > 0:
            return r3, r4
        if self.delta < 0:
            return r4, r3

    def traj_cartesian(self, visualization=False):
        """
        Get the trajectory in cartesian frame
        """

        if self.delta != 0:
            rl, rr = self.ackerman_model()
        # Trajectory width
        traj_wx = np.linspace(-self.W / 2, self.W / 2, 100)
        lengths = np.linspace(0, self.lookahead_len, 100)
        # the left and right trajectory
        self.yl = [(-self.W / 2, length) for length in lengths]
        self.yr = [(self.W / 2, length) for length in lengths]
        traj_wy4 = np.full(len(traj_wx), self.yl[-1][1])
        traj_wy2 = np.full(len(traj_wx), self.yl[-1][1] / 2)
        traj_wy = np.full(len(traj_wx), self.yl[-1][1] / 4)
        self.traj_w4 = [traj_wx, traj_wy4]
        self.traj_w2 = [traj_wx, traj_wy2]
        self.traj_w = [traj_wx, traj_wy]
        self.baseline = [self.yl, self.yr]

        if self.delta < 0:
            theta = self.lookahead_len / (rl + self.W / 2)
            theta_increments = np.linspace(theta, 0, 100)
            self.yl = [(-self.W / 2 + rl * (np.cos(theta_i) - 1), rl * np.sin(theta_i))
                       for i, theta_i in enumerate(theta_increments)]
            self.yr = [(-self.W / 2 - rl + rr * np.cos(theta_i), rr * np.sin(theta_i))
                       for i, theta_i in enumerate(theta_increments)]

        if self.delta > 0:
            theta = self.lookahead_len / (rr + self.W / 2)
            theta_increments = np.linspace(theta, 0, 100)
            self.yl = [(-self.W / 2 + rl * (1 - np.cos(theta_i)), rl * np.sin(theta_i))
                       for i, theta_i in enumerate(theta_increments)]
            self.yr = [(self.W / 2 + rr * (1 - np.cos(theta_i)), rr * np.sin(theta_i))
                       for i, theta_i in enumerate(theta_increments)]

        if visualization:
            x, y = zip(*self.yl)
            plt.plot(x, y, "g")
            x, y = zip(*self.yr)
            plt.plot(x, y, "g")
            plt.plot(self.traj_w4[0], self.traj_w4[1], "y")
            plt.plot(self.traj_w2[0], self.traj_w2[1], "orange")
            plt.plot(self.traj_w[0], self.traj_w[1], "r")
            base_l, base_r = self.baseline[0], self.baseline[1]
            b_lx, b_ly = zip(*base_l)
            plt.plot(b_lx, b_ly, "y")
            b_rx, b_ry = zip(*base_r)
            plt.plot(b_rx, b_ry, "y")
            plt.xlabel("Vehicle width")
            plt.ylabel("Lookahead distance")
            plt.xlim((-4, 4))
            plt.ylim((0, 8))
            plt.title("dynamic trajectory in cartesian frame (top down view)")
            plt.grid()
            plt.savefig("top_down_view.svg")
            plt.show()

    def coordinate_transform(self):
        """
        Transform trajectory points in the cartesian frame to the pixel frame.
        Camera focal length in pixels fx = 1907.78, fy = 1908.92.
        Optical center coordinate in pixels cx = 986.75, cy = 547.373.
        Camera height h = 0.65m
        :return: The trajectory in the pixels frames
        """

        # Initialize the camera extrinsic parameter
        h = 0.65
        theta = np.radians(13)

        # Load trajectories
        xl, yl = zip(*self.yl)
        xr, yr = zip(*self.yr)
        xw4, yw4 = self.traj_w4[0], self.traj_w4[1]
        xw2, yw2 = self.traj_w2[0], self.traj_w2[1]
        xw, yw = self.traj_w[0], self.traj_w[1]
        baseline_l, baseline_r = self.baseline[0], self.baseline[1]
        bs_xl, bs_yl = zip(*baseline_l)
        bs_xr, bs_yr = zip(*baseline_r)

        # World frame to the pixel frame
        # Left wheel trajectory in the pixel frame
        pixl_u = []
        pixl_v = []
        for i, x in enumerate(xl):
            u, v = self.cam2pix(theta, h, x, yl[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixl_u.append(u), pixl_v.append(v)

        # Right wheel trajectory in the pixel frame
        pixr_u = []
        pixr_v = []
        for i, x in enumerate(xr):
            u, v = self.cam2pix(theta, h, x, yr[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixr_u.append(u), pixr_v.append(v)

        # Left wheel Baseline in the pixel frame
        pixbl_u = []
        pixbl_v = []
        for i, x in enumerate(bs_xl):
            u, v = self.cam2pix(theta, h, x, bs_yl[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixbl_u.append(u), pixbl_v.append(v)

        # Right wheel Baseline in the pixel frame
        pixbr_u = []
        pixbr_v = []
        for i, x in enumerate(bs_xr):
            u, v = self.cam2pix(theta, h, x, bs_yr[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixbr_u.append(u), pixbr_v.append(v)

        # 6m Baseline width in the pixel frame
        pixw4_u = []
        pixw4_v = []
        for i, x in enumerate(xw4):
            u, v = self.cam2pix(theta, h, x, yw4[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixw4_u.append(u), pixw4_v.append(v)

        # 3m Baseline width in the pixel frame
        pixw2_u = []
        pixw2_v = []
        for i, x in enumerate(xw2):
            u, v = self.cam2pix(theta, h, x, yw2[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixw2_u.append(u), pixw2_v.append(v)

        # 1.5m Baseline width in the pixel frame
        pixw_u = []
        pixw_v = []
        for i, x in enumerate(xw):
            u, v = self.cam2pix(theta, h, x, yw[i])
            u, v = self.out_of_screen(u, v)
            if u is None:
                pass
            else:
                pixw_u.append(u), pixw_v.append(v)

        base_l = np.column_stack((pixbl_u, pixbl_v))
        base_r = np.column_stack((pixbr_u, pixbr_v))
        base_w4 = np.column_stack((pixw4_u, pixw4_v))
        base_w2 = np.column_stack((pixw2_u, pixw2_v))
        base_w = np.column_stack((pixw_u, pixw_v))
        traj_l = np.column_stack((pixl_u, pixl_v))
        traj_r = np.column_stack((pixr_u, pixr_v))

        return base_l, base_r, base_w4, base_w2, base_w, traj_l, traj_r

    @staticmethod
    def cam2pix(theta, h, xw, zw):
        """
        The formula for coordinate transform
        :param theta: the angle between horizontal line and camera facing
        :param h: the camera height
        :param xw: x value in world frame
        :param zw: z value in world frame
        :return: the list of pixel value
        """

        # Initialize the camera intrinsic parameters
        fx = 1907.78
        fy = 1908.92
        cx = 982.75
        cy = 547.373

        u = cx + (fx * xw) / (h * np.sin(theta) + np.cos(theta) * zw)
        v = cy + fy * (h * np.cos(theta) - np.sin(theta) * zw) / (h * np.sin(theta) + np.cos(theta) * zw)

        return u, v

    @staticmethod
    def out_of_screen(u, v):
        """
        # Exclude each trajectory pixels out of the screen (the camera dead zone)
        :param u: u value in pixel frame
        :param v: v value in pixel frame
        :return:
        """
        if 0 <= u <= 1920 and 0 <= v <= 1080:
            return int(u), int(v)
        else:
            return None, None

    def visualization(self):
        """
        Visualize the trajectory in the recorded video, pressing a represents turning left ,
        pressing d represents turning right.
        """
        cap = cv2.VideoCapture(self.fp)

        # Check if camera opened successfully
        if cap.isOpened() is False:
            print("Error opening video file")

        # Read until video is completed
        while cap.isOpened():
            hot_key = self.get_key()
            if hot_key == "a":
                self.delta -= np.radians(1)
            if hot_key == "d":
                self.delta += np.radians(1)

            # Get the pixel points
            self.traj_cartesian()
            bl, br, bw4, bw2, bw, tl, tr = self.coordinate_transform()

            ret, frame = cap.read()
            if ret is True:
                # # Visualize the dynamic trajectory
                frame = cv2.polylines(frame, [tl, tr], False, (0, 255, 100), 8)
                # Visualize the baseline
                frame = cv2.polylines(frame, [bl, bw4, br], False, (0, 180, 255), 8)
                # Visualize the baseline interval
                frame = cv2.polylines(frame, [bw2], False, (0, 70, 255), 8)
                frame = cv2.polylines(frame, [bw], False, (0, 0, 255), 8)
                cv2.imshow('Dynamic Backup Line', frame)
                print("Current steering angle is:", np.degrees(self.delta))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                break
        # Release the VideoCapture and close all windows
        cap.release()
        cv2.destroyAllWindows()

    def get_key(self):
        """
        Get the hotkey value from the terminal
        :return: hotkey value
        """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.tty_settings)
        return key


if __name__ == "__main__":
    backup_traj = BackupTraj(fp)
    backup_traj.str_input()
    backup_traj.traj_cartesian(visualization=False)
    backup_traj.coordinate_transform()
    backup_traj.visualization()
