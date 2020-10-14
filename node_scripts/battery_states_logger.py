#!/usr/bin/env python

import influxdb
import rospy

from influxdb_store.utils import timestamp_to_influxdb_time

from pr2_msgs.msg import BatteryServer2


class BatteryStatesLogger(object):
    def __init__(self):
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)
        self.sub = rospy.Subscriber(
            '~input', BatteryServer2, self._cb, queue_size=10)

    def _cb(self, msg):
        time = timestamp_to_influxdb_time(msg.header.stamp)
        battery_id = msg.id
        battery_charge = msg.average_charge / 100.0
        query = [{
            "measurement": "battery_states",
            "tags": {
                "battery_id": battery_id
            },
            "time": time,
            "fields": {
                "charge_percent": battery_charge,
            }
        }]
        self.client.write_points(query, time_precision='n')


if __name__ == '__main__':
    rospy.init_node('battery_states_logger')
    logger = BatteryStatesLogger()
    rospy.spin()
