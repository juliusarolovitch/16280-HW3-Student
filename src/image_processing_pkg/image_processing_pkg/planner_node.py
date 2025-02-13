import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_detection_interfaces.msg import DetectionArray
import message_filters
from AmenablePDDL import AmenableP  

def dfs(interface, depth_limit=50):
    """
    Args:
        interface: An instance of the AmenablePDDL interface.
        depth_limit: Maximum search depth.
    
    Returns:
        A plan (list of (action, binding) tuples) if a goal state is reached; otherwise, None.
    """
    initial_state = interface.get_initial_state()
    # Stack elements: (state, plan, depth_remaining)
    stack = [(initial_state, [], depth_limit)]
    visited = set()
    
    while stack:
        state, plan, depth_remaining = stack.pop()
        
        if interface.is_goal_state(state):
            return plan
        
        if depth_remaining <= 0:
            continue
        
        state_key = frozenset(state)
        if state_key in visited:
            continue
        visited.add(state_key)
        
        for action, binding in interface.find_applicable_actions(state):
            new_state = interface.apply_action(action, state, binding)
            new_plan = plan + [(action, binding)]
            stack.append((new_state, new_plan, depth_remaining - 1))
    
    return None

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.bridge = CvBridge()
        
        self.image_sub = message_filters.Subscriber(self, Image, '/image_raw')
        self.detection_sub = message_filters.Subscriber(self, DetectionArray, '/detections')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.callback)
        
        self.get_logger().info("BBox Planner Node Initialized")
        
        self.labels_dict = {1: "blue", 2: "green", 3: "red"}
        
        self.valid_permutations = {
            ("red", "green", "blue"): 1,
            ("red", "blue", "green"): 2,
            ("green", "red", "blue"): 3,
            ("green", "blue", "red"): 4,
            ("blue", "red", "green"): 5,
            ("blue", "green", "red"): 6
        }
    
    def callback(self, image_msg, detections_msg):
        if len(detections_msg.detections) != 3:
            self.get_logger().warning(
                f"Invalid number of detections. Expected 3, got {len(detections_msg.detections)}"
            )
            return
        
        sorted_detections = sorted(detections_msg.detections, key=lambda d: d.bbox[0])
        
        ordering = []
        for detection in sorted_detections:
            color = self.labels_dict.get(detection.label, None)
            if color is None:
                self.get_logger().warning(f"Unknown detection label: {detection.label}")
                return
            ordering.append(color)
        
        ordering_tuple = tuple(ordering)
        if ordering_tuple not in self.valid_permutations:
            self.get_logger().warning(
                f"Detected configuration {ordering} is not valid."
            )
            return
        
        permutation_id = self.valid_permutations[ordering_tuple]
        self.get_logger().info(f"Detected permutation {ordering} with ID {permutation_id}")
        
        domain_file = "domain.pddl"
        problem_file = f"problem_{permutation_id}.pddl"
        self.get_logger().info(f"Loading PDDL files: {domain_file}, {problem_file}")
        
        interface = AmenableP(domain_file, problem_file)
        
        plan = dfs(interface, depth_limit=50)
        
        if plan:
            self.get_logger().info("Plan found:")
            plan_str = ""
            for step, (action, binding) in enumerate(plan, start=1):
                bound_str = " ".join(str(binding[param]) for param in action.parameters)
                step_str = f"{step}: {action.name} {bound_str}\n"
                plan_str += step_str
                self.get_logger().info(step_str.strip())
            
            with open("plan.txt", "w") as f:
                f.write(plan_str)
            self.get_logger().info("Plan saved to plan.txt")
        else:
            self.get_logger().warning("No plan found for the detected configuration.")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
