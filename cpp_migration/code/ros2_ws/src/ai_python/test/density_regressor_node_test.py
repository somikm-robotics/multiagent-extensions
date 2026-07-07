import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from pathlib import Path
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from transport_interfaces.srv import DensityRegressor
from ai_python.nodes.density_regressor_node import DensityRegressorNode

class DensityRegressorNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_return_density_success(self):
        self.response_done = False
        self.response_success = None
        self.response_density = None
        density_regressor_node = DensityRegressorNode()
        client_node = rclpy.create_node("density_regressor_test_node")

        executor = MultiThreadedExecutor()
        executor.add_node(density_regressor_node)
        executor.add_node(client_node)

        client = client_node.create_client(DensityRegressor, "density_regressor")

        start = time.time()
        while not client.wait_for_service(timeout_sec=0.1):
            if time.time() - start > 3.0:
                self.fail("Density Regressor Service is not available")

        bridge = CvBridge()

        image_path = (
            Path(get_package_share_directory("shared_infrastructure"))
            /"images"
            /"test"
            /"dust_plume_image.jpg"
        )
        print(image_path)
        print(image_path.exists())
        cv_image = cv2.imread(str(image_path))
        self.assertIsNotNone(cv_image, f"Could not read image: {image_path}")
        
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        request = DensityRegressor.Request()
        request.detected_image = image_msg

        future = client.call_async(request)

        while rclpy.ok() and not future.done():
            executor.spin_once(timeout_sec=0.05)

        self.handle_response(future)

        self.assertTrue(self.response_success, "Test failed")
        density_regressor_node.get_logger().info(f"Test Passed - Density: {self.response_density}")

        executor.remove_node(density_regressor_node)
        executor.remove_node(client_node)

        density_regressor_node.destroy_node()
        client_node.destroy_node()

    def handle_response(self, future):
        #self.get_logger().info("In handle response")
        response = future.result()
        self.response_density = response.density
        self.response_success = response.success
        self.response_done = True

        



if __name__ == "__main__":
    unittest.main()        
