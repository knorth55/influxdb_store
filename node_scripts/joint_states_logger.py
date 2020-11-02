#!/usr/bin/env python

import influxdb
import rospy
import threading

from influxdb_store.utils import timestamp_to_influxdb_time

from sensor_msgs.msg import JointState


class JointStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = rospy.Subscriber(
            '~input', JointState, self._cb, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(3.0), self._timer_cb)
        self.lock = threading.Lock()
        self.query = []

    def _cb(self, msg):
        time = timestamp_to_influxdb_time(msg.header.stamp)
        joint_names = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort
        for joint_name, pos, vel, eff in zip(
                joint_names, position, velocity, effort):
            with self.lock:
                self.query.append({
                    "measurement": "joint_states",
                    "tags": {
                        "joint_name": joint_name
                    },
                    "time": time,
                    "fields": {
                        "position": pos,
                        "velocity": vel,
                        "effort": eff
                    }
                })

    def _timer_cb(self, event):
        with self.lock:
            if len(self.query) == 0:
                return
            self.client.write_points(self.query, time_precision='ms')
            self.query = []


if __name__ == '__main__':
    rospy.init_node('joint_states_logger')
    logger = JointStatesLogger()
    rospy.spin()
