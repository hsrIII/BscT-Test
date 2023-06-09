import evdev
import numpy as np
#import websocket
#import json
#import time
import Operators


class InputReader:
    def __init__(self):
        self._devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        self._device = None
        self._channel = None
        self._qOp = Operators.Operators()
        self.rec_length = 2
        self.wx_bias = -0.5359484696760158
        self.wy_bias = 6.677578750560675
        self.wz_bias = -5.771425796114199

        self.gyr_data_raw = None
        self.acc_data_raw = None
        self.timestamps_raw = None

        self.gyr_data = None
        self.acc_data = None
        self.timestamps = None

        # TODO: How do Acc. commonly use + and - values?
        #  acc. to /https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md
        #   Acc. lying still produce + value because Force is acting upwards
        #   if this is true - can inverting the coord. frame be made unnessesary?
        self.invert_acc_coord_frame = True
        '''
        Scaling Gyro Values according to
        /https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md
        Multiplying with 2+pi to get angular rate rad/s from rpm 
        '''
        # wx, wy, wz seemed to high --> fixed it by multiplying with 1/1000, was written with mX instead of kX!
        # w_scaling = 0.0001694 * 2*np.pi * 1/1000
        # Accelerometer Scaling value also has false prefixes, yet value is still correct because it is multiplied
        #  with 1/1000 on the website
        self.w_scaling = 4000/(2*32767000+1000)/360 * 2*np.pi

    def read_sensor(self):
        gyr_data_raw = np.empty((0, 3), float)
        acc_data_raw = np.empty((0, 3), float)
        timestamps_raw = np.array([])
        print(" ... recording ... ")
        for event in self._device.read_loop():
            t = event.timestamp()
            if len(timestamps_raw) == 0:
                t0 = t
            if (t-t0) > self.rec_length:
                print("recording stopped")
                break

            wx_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_RX']).value
            wy_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_RY']).value
            wz_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_RZ']).value
            wt_raw = np.array([wx_raw, wy_raw, wz_raw])

            ax_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_X']).value
            ay_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_Y']).value
            az_raw = self._device.absinfo(evdev.ecodes.ecodes['ABS_Z']).value
            at_raw = np.array([ax_raw, ay_raw, az_raw])

            gyr_data_raw = np.append(gyr_data_raw, np.array([wt_raw]), axis=0)
            acc_data_raw = np.append(acc_data_raw, np.array([at_raw]), axis=0)
            timestamps_raw = np.append(timestamps_raw, t)

        self.gyr_data_raw = gyr_data_raw
        self.acc_data_raw = acc_data_raw
        self.timestamps_raw = timestamps_raw

    def real_sensor_events(self, method="by hand"):
        attitudes = np.empty((0,4), float)  # empty array
        timestamps = np.array([])
        gyr_data = np.empty((0,3), float)
        acc_data = np.empty((0, 3), float)

        for event in self._device.read_loop():
            if len(timestamps) == 0:
                t0 = event.timestamp()
                t_prev = t0  #?
                q = np.array([1, 0, 0, 0])  # Annahme: q(t=0)=[1,0,0,0]

            else:
                #print(timestamps)
                t_prev = timestamps[-1]

            t = event.timestamp()-t0  # t in sec, Auflösung: Microsekunde
            if t > self.rec_length:
                break

            wx_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RX'])
            wy_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RY'])
            wz_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RZ'])
            '''
            Scaling Gyro Values according to
            /https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md
            Multiplying with 2+pi to get angular rate rad/s from rpm 
            '''
            wx = wx_absinfo.value * self.w_scaling#0.0001694 * 2*np.pi
            wy = wy_absinfo.value * self.w_scaling#0.0001694 * 2*np.pi
            wz = wz_absinfo.value * self.w_scaling #0.0001694 * 2*np.pi

            ax_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_X'])
            ay_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_Y'])
            az_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_Z'])

            if self.invert_acc_coord_frame == True:
                inv_f = -1
            else:
                inv_f = 1
            #TODO: ax is likely falsely scaled! Use value from website https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md#normal-8000-mg
            ax = ax_absinfo.value / ax_absinfo.resolution * inv_f
            ay = ay_absinfo.value / ay_absinfo.resolution * inv_f
            az = az_absinfo.value / az_absinfo.resolution * inv_f

            wx = wx - self.wx_bias
            wy = wy - self.wy_bias
            wz = wz - self.wz_bias

            wt = np.array([wx, wy, wz])
            at = np.array([ax, ay, az])

            q, t = self._qOp.angular_velocity_to_q(wt, at, q, t, t_prev, method=method)
            timestamps = np.append(timestamps, t)
            attitudes = np.append(attitudes, np.array([q]), axis = 0)
            gyr_data = np.append(gyr_data, np.array([wt]), axis=0)
            acc_data = np.append(acc_data, np.array([at]), axis=0)

        self.gyr_data = gyr_data
        self.acc_data = acc_data
        self.timestamps = timestamps

        return attitudes, timestamps, gyr_data, acc_data


    def calibrate(self, duration = 5):
        input("put controller down and input 'Enter'-key at the end of this line to start gyro calibration: ")
        print(" ... calibrating, do not move sensor ... ")
        wx_data = np.array([])
        wy_data = np.array([])
        wz_data = np.array([])

        for count, event in enumerate(self._device.read_loop()):
            if count == 0:
                t0 = event.timestamp()
            t = event.timestamp()-t0
            if t > duration:
                break
            wx_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RX'])
            wy_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RY'])
            wz_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_RZ'])

            wx = wx_absinfo.value * self.w_scaling#0.0001694 * 2*np.pi
            wy = wy_absinfo.value * self.w_scaling#0.0001694 * 2*np.pi
            wz = wz_absinfo.value * self.w_scaling#0.0001694 * 2*np.pi

            wx_data = np.append(wx_data, wx)
            wy_data = np.append(wy_data, wy)
            wz_data = np.append(wz_data, wz)

        #print(wx_data, wy_data, wz_data)
        self.wx_bias = np.mean(wx_data)
        self.wy_bias = np.mean(wy_data)
        self.wz_bias = np.mean(wz_data)
        print("calibration done")


    @property
    def device(self):
        return self._device

    @device.setter
    def device(self, device_index):    #Input: Device-Nr. (Index in List)
        assert isinstance(device_index, int)
        self._device = self._devices[device_index]

    @property
    def channel(self):
        return self._channel

    @channel.setter
    def channel(self, channel):
        assert isinstance(channel, int)
        self._channel = channel

    def show_devices(self):
        print(f"\n#Nr. |    Path    |    Name    |    Phys")
        for index, device in enumerate(self._devices):
            print(f"{index}    |  {device.path}  |  {device.name}  |  {device.phys}")
        print()

    def show_capabilities(self, device_nr = None, verboseit = False, writetotxt = False):     # https://www.kernel.org/doc/Documentation/input/event-codes.txt
        if device_nr != None:
            device = self.devices[device_nr]
        else:
            device = self._device

        print("\n{      Capabilities of: ",device.name,"        see: https://www.kernel.org/doc/Documentation/input/event-codes.txt")
        for key in device.capabilities(verbose=True).keys():
            print(f" {key}  :  {device.capabilities(verbose=True)[key]}")
        print("}\n")
        return device.capabilities(verbose = verboseit)



    def writetotxt(self, writeme, mode="w"):
        #file = input("Enter name of file")
        name = input("Enter name of text file: ")
        f = open(f"{str(name)}.txt", f"{mode}")
        f.write(str(writeme))
        f.close()
