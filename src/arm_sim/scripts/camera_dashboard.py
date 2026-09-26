#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class CameraDashboard(Node):

    def __init__(self):
        super().__init__('camera_dashboard')

        self.bridge = CvBridge()

        self.declare_parameter(
            'rgb_camera',
            '/camera/image'
        )

        self.declare_parameter(
            'elbow_camera',
            '/elbow_camera/image'
        )

        self.declare_parameter(
            'gripper_camera',
            '/gripper_camera/image'
        )

        topics = [
            self.get_parameter('rgb_camera').value,
            self.get_parameter('elbow_camera').value,
            self.get_parameter('gripper_camera').value,
        ]

        self.images = [None, None, None]

        self.subscribers = []

        for i, topic in enumerate(topics):

            self.subscribers.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, index=i:
                    self.image_callback(msg, index),
                    10
                )
            )

            self.get_logger().info(
                f'Camera {i + 1}: {topic}'
            )

        self.get_logger().info(
            'Camera dashboard started.'
        )

    def image_callback(self, msg, index):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.images[index] = image

        except Exception as e:

            self.get_logger().error(
                f'Could not convert camera {index + 1}: {e}'
            )

    def resize_image(
        self,
        image,
        width,
        height
    ):

        if image is None:

            output = np.zeros(
                (height, width, 3),
                dtype=np.uint8
            )

            cv2.putText(
                output,
                'NO IMAGE',
                (
                    width // 2 - 80,
                    height // 2
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (255, 255, 255),
                2
            )

            return output

        h, w = image.shape[:2]

        scale = min(
            width / w,
            height / h
        )

        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(
            image,
            (new_w, new_h),
            interpolation=cv2.INTER_AREA
        )

        output = np.zeros(
            (height, width, 3),
            dtype=np.uint8
        )

        x = (width - new_w) // 2
        y = (height - new_h) // 2

        output[
            y:y + new_h,
            x:x + new_w
        ] = resized

        return output

    def create_dashboard(self):

        tile_width = 640
        tile_height = 480

        names = [
            'D435i RGB',
            'ELBOW CAMERA',
            'GRIPPER CAMERA'
        ]

        tiles = []

        for i in range(3):

            tile = self.resize_image(
                self.images[i],
                tile_width,
                tile_height
            )

            # Title bar
            cv2.rectangle(
                tile,
                (0, 0),
                (tile_width, 45),
                (0, 0, 0),
                -1
            )

            cv2.putText(
                tile,
                names[i],
                (15, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2
            )

            tiles.append(tile)

        # Arrange three cameras:
        #
        # ┌──────────────┬──────────────┐
        # │  D435i RGB   │ Elbow Camera │
        # ├──────────────┴──────────────┤
        # │       Gripper Camera        │
        # └─────────────────────────────┘

        top = np.hstack(
            (tiles[0], tiles[1])
        )

        # Put gripper camera in the centre
        # of a 1280px-wide row.
        bottom = np.zeros(
            (tile_height, tile_width * 2, 3),
            dtype=np.uint8
        )

        bottom[
            :,
            tile_width // 2:
            tile_width // 2 + tile_width
        ] = tiles[2]

        dashboard = np.vstack(
            (top, bottom)
        )

        return dashboard


def main(args=None):

    rclpy.init(args=args)

    node = CameraDashboard()

    window_name = 'ARM CAMERA DASHBOARD'

    cv2.namedWindow(
        window_name,
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        window_name,
        1280,
        960
    )

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            dashboard = node.create_dashboard()

            cv2.imshow(
                window_name,
                dashboard
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
