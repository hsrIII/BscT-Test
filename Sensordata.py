import evdev
import numpy as np
import websocket
import json
import time
import Operators


class InputReader:
    def __init__(self):
        self._devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        self._device = None
        self._channel = None
        self._qOp = Operators.Operators()
        self.rec_length = 2
        self.wx_bias = 0
        self.wy_bias = 0
        self.wz_bias = 0

    def read_device(self, event_type = None, maxevents = None, printit = False):
        print("reading device...")
        event_count = 0
        events = []
        for event in self._device.read_loop():
            print()
            t = event.timestamp()
            print(t)
            #print(f"code: {event.code}")
            print(f"event.value: {event.value}")
            #print(f"evdev.ecodes.BTN_DPAD_DOWN: {evdev.ecodes.BTN_DPAD_DOWN}")
            print(f"event.type: {event.type}")
            #print(f"evdev.ecodes.EV_Key: {evdev.ecodes.EV_KEY}")
            print(f"evdev.ecodes.ecodes['ABS_X']: {evdev.ecodes.ecodes['ABS_X']}")
            print(f"evdev.ecodes.ecodes['ABS_Y']: {evdev.ecodes.ecodes['ABS_Y']}")
            print(f"evdev.ecodes.ecodes['ABS_Z']: {evdev.ecodes.ecodes['ABS_Z']}")
            print(f"evdev.ecodes.ecodes['ABS_RX']: {evdev.ecodes.ecodes['ABS_RX']}")
            print(f"evdev.ecodes.ecodes['ABS_RY']: {evdev.ecodes.ecodes['ABS_RY']}")
            print(f"evdev.ecodes.ecodes['ABS_RZ']: {evdev.ecodes.ecodes['ABS_RZ']}")

            print(f"self._device.absinfo: {self._device.absinfo(evdev.ecodes.ecodes['ABS_RZ'])}")

            if str(printit) == "cat":
                print(evdev.categorize(event))

            else:
                if event_type != None:
                    if event.type == event_type:
                        if printit == True:
                            print(f"{event.type} |  {event}")
                        #event_count = event_count+1
                        #events.append(event)
                else:
                    if printit == True:
                        print(f"{event.type} |  {event}")
            events.append(event)
            event_count = event_count+1
            if maxevents != None:
                if event_count >= maxevents:
                    print(f"... read {event_count} events")
                    return events


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
            wx = 2*np.pi * wx_absinfo.value * 0.0001694
            wy = 2*np.pi * wy_absinfo.value * 0.0001694
            wz = 2*np.pi * wz_absinfo.value * 0.0001694
            #print(1/wx_absinfo.resolution)

            ax_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_X'])
            ay_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_Y'])
            az_absinfo = self._device.absinfo(evdev.ecodes.ecodes['ABS_Z'])

            ax = ax_absinfo.value / ax_absinfo.resolution
            ay = ay_absinfo.value / ay_absinfo.resolution
            az = az_absinfo.value / az_absinfo.resolution

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


        return attitudes, timestamps, gyr_data, acc_data


    def calibrate(self, duration = 5):
        input("put controller down and input 'Enter'-key at the end of this line to start gyro calibration: ")
        print("starting calibration")
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

            wx = 2*np.pi * wx_absinfo.value * 0.0001694
            wy = 2*np.pi * wy_absinfo.value * 0.0001694
            wz = 2*np.pi * wz_absinfo.value * 0.0001694

            wx_data = np.append(wx_data, wx)
            wy_data = np.append(wy_data, wy)
            wz_data = np.append(wz_data, wz)

        print(wx_data, wy_data, wz_data)
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