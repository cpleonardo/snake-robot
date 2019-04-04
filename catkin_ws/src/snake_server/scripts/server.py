#!/usr/bin/env python
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from snake_msgs.msg import ArrayModule, Module
import json, threading, sys, rospy
from urlparse import urlparse, parse_qs


PORT_NUMBER = 6666
joints_states = ArrayModule()


def jointsStatesCallback(data):
    global joints_states
    joints_states = data

def start_node():
    rospy.Subscriber("snake_joints_states",
                     ArrayModule,
                     jointsStatesCallback)
    rospy.init_node('snake_joint_state_listener', anonymous=True)


class SnakeRequestHandler(BaseHTTPRequestHandler):

	# Handler for the GET requests
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        query_components = parse_qs(urlparse(self.path).query)
        module = int(query_components["module"][0])
        global joints_states
        module_data = joints_states.ArrayModule[module - 1]
        response = {
            "speed": module_data.speed,
            "pitch": module_data.pitch,
            "yaw": module_data.yaw,
        }
        print(response)
        self.wfile.write(json.dumps(response))
        return


if __name__ == '__main__':
    try:
        server = HTTPServer(('', PORT_NUMBER), SnakeRequestHandler)
        thread = threading.Thread(target = server.serve_forever)
        thread.daemon = True
        thread.start()
        print('Started httpserver on port ' , PORT_NUMBER)
        start_node()
        print('Started snake_joint_state_listener ros node')
        rospy.spin()
    except:
        server.server_close()
        server.shutdown()
        sys.exit(0)
