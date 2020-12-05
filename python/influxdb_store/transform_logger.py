import influxdb
import numpy as np
import rospy
import yaml

import tf2_ros

from influxdb_store.utils import timestamp_to_influxdb_time
from tf2_msgs.srv import FrameGraph


class TransformLogger(object):
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

        trans_x_fields = {}
        trans_y_fields = {}
        trans_z_fields = {}
        rot_x_fields = {}
        rot_y_fields = {}
        rot_z_fields = {}
        rot_w_fields = {}
        rot_theta_fields = {}
        for child_frame_id, _ in graph.items():
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.frame_id, child_frame_id, timestamp)
            except tf2_ros.ExtrapolationException as e:
                rospy.logerr_throttle(
                    60.0, 'tf2_ros.ExtrapolationException: {}'.format(e))
                continue
            except tf2_ros.ConnectivityException as e:
                rospy.logerr_throttle(
                    60.0, 'tf2_ros.ConnectivityException: {}'.format(e))
                continue
            translation = transform_stamped.transform.translation
            rotation = transform_stamped.transform.rotation
            theta = 2 * np.arctan(
                np.linalg.norm(
                    [rotation.x, rotation.y, rotation.z]) / rotation.w)

            trans_x_fields[child_frame_id] = translation.x
            trans_y_fields[child_frame_id] = translation.y
            trans_z_fields[child_frame_id] = translation.z
            rot_x_fields[child_frame_id] = rotation.x
            rot_y_fields[child_frame_id] = rotation.y
            rot_z_fields[child_frame_id] = rotation.z
            rot_w_fields[child_frame_id] = rotation.w
            rot_theta_fields[child_frame_id] = theta

        query = []
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "translation",
                "field": "x",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": trans_x_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "translation",
                "field": "y",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": trans_y_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "translation",
                "field": "z",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": trans_z_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "rotation",
                "field": "x",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": rot_x_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "rotation",
                "field": "y",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": rot_y_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "rotation",
                "field": "z",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": rot_z_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "rotation",
                "field": "w",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": rot_w_fields
        })
        query.append({
            "measurement": self.measurement_name,
            "tags": {
                "type": "rotation",
                "field": "theta",
                "frame_id": self.frame_id
            },
            "time": time,
            "fields": rot_theta_fields
        })

        if len(query) > 0:
            self.client.write_points(query, time_precision='ms')
