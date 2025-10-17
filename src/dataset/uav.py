import os
import numpy as np
from collections import namedtuple

if __name__ == "__main__":
    import sys
    sys.path.append(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from src.common.datatypes import SensorType
else:
    from ..common.datatypes import SensorType

class VOXL_IMUDataReader:
    """
        Read IMU data on VOXL side
    """
    def __init__(
            self,
            path: str,
            sensor_type: SensorType,
            starttime=-float('inf'),
            window_size=None,
        ):
        self.path = path
        self.sensor_type = sensor_type
        self.starttime = starttime
        self.field = namedtuple('data', ['timestamp', 'a', 'w'])
        self.window_size = window_size
        self.buffer = []

    def parse(self, line):
        """
        line: 
            i,timestamp(ns),AX(m/s2),AY(m/s2),AZ(m/s2),GX(rad/s),GY(rad/s),GZ(rad/s),T(C)
            
            Apply calibration to IMU data:
                corrected_acc = (a_measured - offset) * scale
                corrected_gyro = w_measured - offset
        """
        line = [float(_) for _ in line.strip().split(',')]

        timestamp = line[1]
        a = np.array(line[2:5])
        w = np.array(line[5:8])
        return self.field(timestamp, a, w)

    def rolling_average(self, data):

        d = np.hstack([data.w, data.a])
        self.buffer.append(d)
        if len(self.buffer) > self.window_size:
            mean = np.mean(self.buffer, axis=0)
            self.buffer = self.buffer[-self.window_size:]
            return self.field(timestamp=data.timestamp, w=mean[:3], a=mean[3:])

        return self.field(timestamp=data.timestamp, w=data.w, a=data.a)

    def __iter__(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                data = self.parse(line)
                if data.timestamp < self.starttime:
                    continue
                if self.window_size is not None:
                    data = self.rolling_average(data)
                yield data

    def start_time(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                return self.parse(line).timestamp

    def set_starttime(self, starttime):
        self.starttime = starttime

class VOXL_TrackingCameraDataReader:

    def __init__(self, path: str, sensor_type: SensorType, starttime=-float('inf')):
        self.path = path
        self.sensor_type = sensor_type
        self.root_path = "/".join(path.split("/")[:-1])
        self.starttime = starttime
        self.field = namedtuple('data',
            ['timestamp', 'image_path'])

    def parse(self, line):
        """
        line: 
            i,timestamp(ns),gain,exposure(ns),format,height,width,frame_id,reserved
        """
        line = [float(_) for _ in line.strip().split(',')]

        index = int(line[0])
        frame_id = f"{index:05}.png"
        image_path = os.path.join(self.root_path, frame_id)
        timestamp = line[1]

        return self.field(timestamp, image_path)

    def __iter__(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                data = self.parse(line)
                if data.timestamp < self.starttime:
                    continue
                yield data

    def start_time(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                return self.parse(line).timestamp

    def set_starttime(self, starttime):
        self.starttime = starttime

class PX4_IMUDataReader:
    def __init__(
            self,
            gyro_path: str,
            acc_path: str,
            sensor_type: SensorType,
            starttime=-float('inf'),
            window_size=None,
        ):
        self.multiplier = 1000
        self.sensor_type = sensor_type
        self.gyro_path = gyro_path
        self.acc_path = acc_path
        self.starttime = starttime
        self.field = namedtuple('data',
            ['timestamp', 'a', 'w'])

        self.window_size = window_size
        self.buffer = []

    def parse(self, gyro_line, acc_line):
        """
        gyro_line: 
            timestamp,timestamp_sample,device_id,x,y,z,temperature,error_count,clip_counter[0],clip_counter[1],clip_counter[2],samples
        acc_line:
            timestamp,timestamp_sample,device_id,x,y,z,temperature,error_count,clip_counter[0],clip_counter[1],clip_counter[2],samples
            
        Apply calibration to IMU data:
            corrected_acc = (a_measured - offset) * scale
            corrected_gyro = w_measured - offset
        """
        gyro_line = [float(_) for _ in gyro_line.strip().split(',')]
        acc_line = [float(_) for _ in acc_line.strip().split(',')]

        timestamp = gyro_line[0] * self.multiplier
        w = np.array(gyro_line[3:6])
        a = np.array(acc_line[3:6])
        return self.field(timestamp, w, a)

    def rolling_average(self, data):

        d = np.hstack([data.w, data.a])
        self.buffer.append(d)
        if len(self.buffer) > self.window_size:
            mean = np.mean(self.buffer, axis=0)
            self.buffer = self.buffer[-self.window_size:]
            return self.field(timestamp=data.timestamp, w=mean[:3], a=mean[3:])

        return self.field(timestamp=data.timestamp, w=data.w, a=data.a)

    def __iter__(self):
        with open(self.gyro_path, 'r') as gyro_f, open(self.acc_path, 'r') as acc_f:
            next(gyro_f)
            next(acc_f)
            for gyro_line, acc_line in zip(gyro_f, acc_f):
                data = self.parse(gyro_line, acc_line)
                if data.timestamp < self.starttime:
                    continue
                if self.window_size is not None:
                    data = self.rolling_average(data)
                yield data

    def start_time(self):
        with open(self.gyro_path, 'r') as f:
            next(f)
            for line in f:
                return self.parse(line).timestamp

    def set_starttime(self, starttime):
        self.starttime = starttime

class PX4_GPSDataReader:
    def __init__(
            self,
            path,
            sensor_type: SensorType,
            starttime=-float('inf')):
        self.multiplier = 1000
        self.path = path
        self.sensor_type = sensor_type
        self.starttime = starttime
        self.initial_pose = None
        self.field = namedtuple('data',
            ['timestamp', 'lon', 'lat', 'alt'])

    def parse(self, line):
        """
        line: 
            timestamp,timestamp_sample,time_utc_usec,device_id,lat,lon,alt,alt_ellipsoid,s_variance_m_s,c_variance_rad,eph,epv,hdop,vdop,noise_per_ms,jamming_indicator,vel_m_s,vel_n_m_s,vel_e_m_s,vel_d_m_s,cog_rad,timestamp_time_relative,heading,heading_offset,heading_accuracy,rtcm_injection_rate,automatic_gain_control,fix_type,jamming_state,spoofing_state,vel_ned_valid,satellites_used,selected_rtcm_instance
        """
        line = [float(_) for _ in line.strip().split(',')]
        timestamp = int(line[0]) * self.multiplier
        # timestamp = line[1]

        return self.field(timestamp, lon=int(line[5]), lat=int(line[4]), alt=int(line[6]))

    def __iter__(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                data = self.parse(line)
                if data.timestamp < self.starttime:
                    continue
                yield data

    def start_time(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                return self.parse(line).timestamp

    def set_starttime(self, starttime):
        self.starttime = starttime

class PX4_ActuatorMotorDataReader:
    def __init__(
            self,
            path: str,
            sensor_type: SensorType,
            starttime=-float('inf'),
            window_size=None,
        ):
        self.multiplier = 1000
        self.path = path
        self.sensor_type = sensor_type
        self.starttime = starttime
        self.field = namedtuple('data',
            ['timestamp', 'c0', 'c1', 'c2', 'c3'])

        self.window_size = window_size
        self.buffer = []

    def parse(self, line):
        """
        line:
            timestamp,timestamp_sample,control[0],control[1],control[2],control[3],control[4],control[5],control[6],control[7],control[8],control[9],control[10],control[11],reversible_flags
        """
        line = [float(_) for _ in line.strip().split(',')]

        timestamp = int(line[0]) * self.multiplier
        controls = np.array(list(map(float, line[2:6]))) # in range [0, 1] or [-1, 1]
        c0, c1, c2, c3 = controls
        return self.field(timestamp, c0, c1, c2, c3)

    def rolling_average(self, data):

        d = np.array([data.c0, data.c1, data.c2, data.c3])
        self.buffer.append(d)
        if len(self.buffer) > self.window_size:
            mean = np.mean(self.buffer, axis=0)
            self.buffer = self.buffer[-self.window_size:]
            return self.field(timestamp=data.timestamp, c0=mean[0], c1=mean[1], c2=mean[2], c3=mean[3])

        return self.field(timestamp=data.timestamp, c0=data.c0, c1=data.c1, c2=data.c2, c3=data.c3)

    def __iter__(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                data = self.parse(line)
                if data.timestamp < self.starttime:
                    continue
                if self.window_size is not None:
                    data = self.rolling_average(data)
                yield data

    def start_time(self):
        with open(self.path, 'r') as f:
            next(f)
            for line in f:
                return self.parse(line).timestamp

    def set_starttime(self, starttime):
        self.starttime = starttime


if __name__ == "__main__":
    import os

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

    i = 0
    voxl_datareader = iter(voxl_imu1)
    while True:
        try:
            data = next(voxl_datareader)
            print(data)
        except StopIteration:
            break
        i += 1
