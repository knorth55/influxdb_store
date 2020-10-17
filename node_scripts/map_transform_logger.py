#!/usr/bin/env python

import influxdb
import rospy
import yaml

import tf2_ros

from influxdb_store.utils import timestamp_to_influxdb_time
from tf2_msgs.srv import FrameGraph


class MapTransformLogger(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service("~tf2_frames")
        self.get_frames = rospy.ServiceProxy("~tf2_frames", FrameGraph)

        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 8086)
        database = rospy.get_param('~database', 'test')
        self.client = influxdb.InfluxDBClient(
            host=host, port=port, database=database)
        self.client.create_database(database)

        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.update_rate = rospy.Duration(
            rospy.get_param('~update_rate', 0.5))
        self.timer = rospy.Timer(self.update_rate, self._cb)

    def _cb(self, event):
        if event.last_real:
            timestamp = event.last_real
        else:
            timestamp = event.current_real - self.update_rate
        time = timestamp_to_influxdb_time(timestamp)
        try:
            graph = yaml.load(self.get_frames().frame_yaml)
        except Exception as e:
            rospy.logerr(e)
            return

        query = []
        for child_id, info in graph.items():
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.map_frame_id, child_id, timestamp)
            except tf2_ros.ExtrapolationException:
                continue
            translation = transform_stamped.transform.translation
            rotation = transform_stamped.transform.rotation
            query.append({
                "measurement": "map_transform",
                "tags": {
                    "frame_id": transform_stamped.header.frame_id,
                    "child_frame_id": transform_stamped.child_frame_id
                },
                "time": time,
                "fields": {
                    "translation.x": translation.x,
                    "translation.y": translation.y,
                    "translation.z": translation.z,
                    "rotation.x": rotation.x,
                    "rotation.y": rotation.y,
                    "rotation.z": rotation.z,
                    "rotation.w": rotation.w,
                }
            })
        if len(query) > 0:
            self.client.write_points(query, time_precision='n')


if __name__ == '__main__':
    rospy.init_node('map_transform_logger')
    logger = MapTransformLogger()
    rospy.spin()
