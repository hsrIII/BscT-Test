import matplotlib.pyplot as plt
import Operators
import Testdata
import Sensordata
import numpy as np

Testdata = Testdata.Testdata(print_case = False, signal_length=10, sampling_rate = 0.01)
_qOp = Operators.Operators(sampling_rate = Testdata.sampling_rate)
IR = Sensordata.InputReader()

# call functions use_testdata() or use_sensor() to select workflow
def use_testdata():
    # Reference Attitude
    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 reference attitude")
    attitudes, t_ = Testdata.orientation_testcases(case = "E")

    # Sensor Data
    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 02 Sensor data from reference attitude")
    gyr_data, acc_data = _qOp.forwardoperator(attitudes, t_)      # Gyroscope Data, Accelerometer Data

    # #Add Noise
    # print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  02b Add Noise")
    # mu, sigma = 0, 0.001 # bias, standard deviation
    # gyr_data += np.random.normal(mu, sigma, gyr_data.shape)
    # t_ += np.random.normal(0, 0.01/10, t_.shape)

    # Integration of Forwardoperator-Data
    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 03 attitude from angular rate")
    att_res_hand, t_hand = _qOp.simulate_sensor_events(gyr_data, acc_data, t_, method = "by hand")     #resulting Attitude
    att_res_integrator, t_integrator = _qOp.simulate_sensor_events(gyr_data, acc_data, t_, method = "integrator")
    att_res_ekf, t_ekf = _qOp.simulate_sensor_events(gyr_data, acc_data, t_, method = "EKF")

    t_ = t_hand

    # Plotting
    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 04 plotting")
    error = attitudes - att_res_hand
    plots = _qOp.plot_res(attitudes, gyr_data, att_res_hand, t_, error)
    comp = _qOp.plot_comparision(att_res_hand, att_res_integrator, att_res_ekf, t_)

    if input("plot? y, n: ") == "y":
        plt.show()

def use_sensor():
    IR.show_devices()
    IR.device = int(input("select #Nr. of IMU: "))
    IR.invert_acc_coord_frame = True
    if input("calibrate sensor? ('n' to use default calibration) y,n: ").lower() == "y":
        IR.calibrate(int(input("calibrate for how many seconds?: ")))

    IR.rec_length = int(input("Record sensor data for how many seconds?: "))

    usemethod = input(f"Input one integration method of the following: {_qOp.w_to_q_methods} (Default method = 'by hand') After pressing 'Enter' recording will start: ")
    print(f" ... recording for {IR.rec_length} sec ... ")
    q_array, t_, gyr_data, acc_data = IR.real_sensor_events(method=usemethod)
    print(f"recording stopped")
    if input("animate senor data (WARNING: can be computationally expensive)? y,n: ").lower() == "y":
        _qOp.animate_attitudes(q_array, t_, float(input("enter time dt between plots in seconds: ")))

    #plot = _qOp.plot_one(q_array, t_, "q")
    #plot_gyr = _qOp.plot_one(gyr_data, t_, "w")
    #plot_acc = _qOp.plot_one(acc_data, t_, "a")
    #plt.show()
    _qOp.writetotxt(gyr_data, "01_gyr_data")
    _qOp.writetotxt(acc_data, "01_gyr_data")
    _qOp.writetotxt(t_, "01_timestamps")

def read_sensor():
    IR.show_devices()
    IR.device = int(input("select #Nr. of IMU: "))

    IR.rec_length = int(input("Record sensor data for how many seconds?: "))
    IR.read_sensor()

    return IR.gyr_data_raw, IR.acc_data_raw, IR.timestamps_raw

#gyr_data, acc_data, timestamps = read_sensor()
#print(gyr_data, acc_data, timestamps)
'''
IR.writetotxt(gyr_data)#"01_gyr_data"
IR.writetotxt(acc_data)#01_acc_data
IR.writetotxt(timestamps)#01_timestamps
'''
#01: sensor laying still for 2 min
#02: turning sensor for 1 min