#!/usr/bin/env python

from datetime import datetime
import influxdb
import rospy

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
            '~input', JointState, self._cb, queue_size=10)

    def _cb(self, msg):
        timestamp = msg.header.stamp
        time = datetime.utcfromtimestamp(timestamp.to_sec())
        time = time.isoformat("T") + "Z"
        joint_names = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort
        for joint_name, pos, vel, eff in zip(
                joint_names, position, velocity, effort):
            query = [{
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
            }]
            self.client.write_points(query, time_precision='n')


if __name__ == '__main__':
    rospy.init_node('joint_states_logger')
    logger = JointStatesLogger()
    rospy.spin()
