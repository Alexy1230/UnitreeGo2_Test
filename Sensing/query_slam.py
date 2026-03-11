" use ros topics to query slam service."
import json
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('slam_test')

pub = node.create_publisher(String, '/rt/api/request', 10)

msg = String()

data = {
    "api_id": 1801,
    "data": {"slam_type":"indoor"}
}

msg.data = json.dumps(data)

pub.publish(msg)

print("Start mapping command sent")
