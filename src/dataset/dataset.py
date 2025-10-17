import os
import numpy as np
from queue import PriorityQueue
from collections import namedtuple
from threading import Thread

if __name__ == "__main__":
    import sys
    sys.path.append(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from src.common.datatypes import SensorType
else:
    from ..common.datatypes import SensorType

class Sensor:
    def __init__(
        self, 
        dataset, 
        type: SensorType, 
        output_queue: PriorityQueue):
        
        self.type = type
        self.dataset = dataset
        self.dataset_starttime = dataset.starttime
        self.starttime = None
        self.started = False
        self.stopped = False
        self.output_queue = output_queue
        
        self.field = namedtuple('sensor', ['type', 'data'])
        
        self.publish_thread = Thread(target=self.publish)
        
    def start(self, starttime):
        self.started = True
        self.starttime = starttime
        self.publish_thread.start()
        
    def stop(self):
        self.stopped = True
        if self.started:
            self.publish_thread.join()
    
    def publish(self):
        dataset = iter(self.dataset)
        while not self.stopped:
            try:
                data = next(dataset)
            except StopIteration:
                return
            
            self.output_queue.put((data.timestamp, self.field(type=self.type, data=data)))


class Dataset:

    def __init__(self, sensor_data_readers: list):
        self.output_queue = PriorityQueue()

        self.sensors = [Sensor(
            dataset=data_reader,
            type=data_reader.sensor_type,
            output_queue=self.output_queue
        ) for data_reader in sensor_data_readers]

    def start(self, starttime):
        for sensor in self.sensors:
            sensor.start(starttime)

    def stop(self):
        for sensor in self.sensors:
            sensor.stop()
            

    def get_data(self):
        while not self.output_queue.empty():
            data = self.output_queue.get()
            return data


if __name__ == "__main__":
    from src.dataset.uav import (
        VOXL_IMUDataReader,
        PX4_IMUDataReader,
        PX4_GPSDataReader,
        PX4_ActuatorMotorDataReader
    )

    base_dir = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    data_path = os.path.join(base_dir, "data", "UAV")

    px4_imu0 = PX4_IMUDataReader(
        gyro_path=os.path.join(data_path,
                               "log0001/px4/09_00_22_sensor_gyro_0.csv"),
        acc_path=os.path.join(data_path,
                              "log0001/px4/09_00_22_sensor_accel_0.csv"),
        sensor_type=SensorType.PX4_IMU0,
    )

    px4_imu1 = PX4_IMUDataReader(
        gyro_path=os.path.join(data_path,
                               "log0001/px4/09_00_22_sensor_gyro_1.csv"),
        acc_path=os.path.join(data_path,
                              "log0001/px4/09_00_22_sensor_accel_1.csv"),
        sensor_type=SensorType.PX4_IMU1,
    )

    voxl_imu0 = VOXL_IMUDataReader(
        path=os.path.join(data_path, "log0001/run/mpa/imu0/data.csv"),
        sensor_type=SensorType.VOXL_IMU0)

    voxl_imu1 = VOXL_IMUDataReader(
        path=os.path.join(data_path, "log0001/run/mpa/imu1/data.csv"),
        sensor_type=SensorType.VOXL_IMU1)


    motor = PX4_ActuatorMotorDataReader(
        path=os.path.join(data_path,
                          "log0001/px4/09_00_22_actuator_motors_0.csv"),
        sensor_type=SensorType.PX4_ACTUATOR_MOTORS
    )

    gps = PX4_GPSDataReader(path=os.path.join(
        data_path, "log0001/px4/09_00_22_sensor_gps_0.csv"),
        sensor_type=SensorType.PX4_GPS)

    sensors = [
        px4_imu0,
        px4_imu1,
        voxl_imu0,
        voxl_imu1,
        motor,
        gps
    ]
    dataset = Dataset(sensor_data_readers=sensors)
    dataset.start(starttime=0.0)
    count = 0
    while True:
        timestamp, sensor_data = dataset.get_data()
        
        if sensor_data.type is SensorType.PX4_IMU0:
            print(f"Timestamp: {timestamp}, Sensor Type: {sensor_data.type.name}")
            count += 1
        else:
            print(f"Timestamp: {timestamp}, Sensor Type: {sensor_data.type.name}")
        if count >= 10:
            break

    dataset.stop()
