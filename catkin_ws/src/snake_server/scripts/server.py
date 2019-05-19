#!/usr/bin/env python
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from snake_msgs.msg import ArrayModule, Module
import json, threading, sys, rospy
from urlparse import urlparse, parse_qs

class SnakeRequestHandler(BaseHTTPRequestHandler):

    PORT_NUMBER = 6666
    joints_states = ArrayModule()

	# Handler for the GET requests
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        query_components = parse_qs(urlparse(self.path).query)
        module = int(query_components["module"][0])
        module_data = self.joints_states.ArrayModule[module - 1]
        response = {
            "speed": module_data.speed,
            "pitch": module_data.pitch,
            "yaw": module_data.yaw,
        }
        self.wfile.write(json.dumps(response))
        return

    def joint_states_callback(self, data):
        self.joints_states = data

    def start_node():
        rospy.Subscriber("snake_joints_states",
                         ArrayModule,
                         self.joint_states_callback)
        rospy.init_node('snake_joint_state_listener', anonymous=True)


if __name__ == '__main__':
    try:
        server = HTTPServer(
            ('', SnakeRequestHandler.PORT_NUMBER), SnakeRequestHandler)
        thread = threading.Thread(target = server.serve_forever)
        thread.daemon = True
        thread.start()
        print('Started httpserver on port ' , PORT_NUMBER)
        SnakeRequestHandler.start_node()
        print('Started snake_joint_state_listener ros node')
        rospy.spin()
    except:
        server.server_close()
        server.shutdown()
        sys.exit(0)
