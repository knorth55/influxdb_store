#!/usr/bin/env python

import copy
import influxdb
import rospy
import threading
import time

from influxdb_store.utils import timestamp_to_influxdb_time

from sensor_msgs.msg import JointState


class JointStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.duration = rospy.get_param('~duration', 3.0)
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = rospy.Subscriber(
            '~input', JointState, self._cb, queue_size=300)
        self.timer = rospy.Timer(rospy.Duration(self.duration), self._timer_cb)
        self.lock = threading.Lock()
        self.query = []

    def _cb(self, msg):
        influx_time = timestamp_to_influxdb_time(msg.header.stamp)
        joint_names = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort
        with self.lock:
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "position"
                },
                "time": influx_time,
                "fields": dict(zip(joint_names, position))
            })
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "velocity"
                },
                "time": influx_time,
                "fields": dict(zip(joint_names, velocity))
            })
            self.query.append({
                "measurement": "joint_states",
                "tags": {
                    "type": "effort"
                },
                "time": influx_time,
                "fields": dict(zip(joint_names, effort))
            })

    def _timer_cb(self, event):
        start_time = time.time() * 1000
        with self.lock:
            if len(self.query) == 0:
                return
            query = copy.deepcopy(self.query)
            self.query = []
        end_time = time.time() * 1000
        rospy.logdebug("copy time: {}ms".format(end_time - start_time))
        rospy.logdebug("data length: {}ms".format(len(query)))
        self.client.write_points(query, time_precision='ms')
        end_time = time.time() * 1000
        rospy.logdebug("timer cb time: {}ms".format(end_time - start_time))
        if ((end_time - start_time) > (self.duration * 1000)):
            rospy.logerr("timer cb time exceeds: {} > {}".format(
                end_time - start_time, self.duration * 1000))


if __name__ == '__main__':
    rospy.init_node('joint_states_logger')
    logger = JointStatesLogger()
    rospy.spin()
