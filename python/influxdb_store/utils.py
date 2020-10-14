from datetime import datetime


def timestamp_to_influxdb_time(timestamp):
    time = datetime.utcfromtimestamp(timestamp.to_sec())
    time = time.isoformat("T") + "Z"
    return time
