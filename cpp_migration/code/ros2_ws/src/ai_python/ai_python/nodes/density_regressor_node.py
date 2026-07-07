import os, time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cv2
import torch
from torch import nn
from torchvision import transforms as T
from torchvision import models

from cv_bridge          import CvBridge
from sensor_msgs.msg    import Image

from transport_interfaces.srv import DensityRegressor

class DensityRegressorNode(Node):
    
    def __init__(self):
        super().__init__("density_regressor_node")
        pkg_share = get_package_share_directory('ai_python')
        ai_model_path = os.path.join(pkg_share, "ai_models", "density_regressor.pt")

        if not os.path.exists(ai_model_path):
            self.get_logger().fatal(f"Density model not found: {ai_model_path}")
            raise SystemExit(1)

        # device
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        #  model
        backbone = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)  # same as training
        backbone.fc = nn.Sequential(                                        # same head
            nn.Linear(backbone.fc.in_features, 1)
        )

        state = torch.load(ai_model_path, map_location=device)              # state_dict
        backbone.load_state_dict(state, strict=True)                        # keys match
        backbone.to(device).eval()

        self.model: nn.Module = backbone
        self.device            = device

        self.get_logger().info(f"Density regressor loaded: {ai_model_path}")

          #  Pre-processing 
        self.bridge  = CvBridge()
        self.pre_tf  = T.Compose([
            T.ToPILImage(),
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225]),
        ])

        self.density_regressor_srv = self.create_service(DensityRegressor, "density_regressor", self.handle_req)
        self.get_logger().info("Density regressor node initialised")

    def handle_req(self,
                   req:DensityRegressor.Request,
                   res:DensityRegressor.Response):
        """
        Return the predicted dust-plume density for the *latest* image sent.
        """
        try:
            self.get_logger().info("In handle_req")
            cv_img = self.bridge.imgmsg_to_cv2(req.detected_image, "bgr8")
            self.get_logger().info("In handle_req - created cv_img")
            # BGR ➜ RGB
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self.get_logger().info("In handle_req - created rgb_img")
            tensor = self.pre_tf(rgb_img).unsqueeze(0)          # 1 × 3 × 224 × 224
            tensor   = tensor.to(self.device)      
            self.get_logger().info("In handle_req - created tensor")
            with torch.no_grad():
                pred = self.model(tensor).squeeze().item()
                
            self.get_logger().info("In handle_req - got pred")
            res.success = True
            res.density = float(pred)
            self.get_logger().info(f"In handle_req - response sent: {res.success} - {res.density}")
            return res
        except Exception as ex:
            self.get_logger().info(f"Some exception occurred: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = DensityRegressorNode()
    node.get_logger().info("✅ density regressor node main() started")
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
