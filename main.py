import matplotlib.pyplot as plt
import Operators
import Testdata
import Sensordata

Testdata = Testdata.Testdata(print_case = False, signal_length=10, sampling_rate = 0.01)
_qOp = Operators.Operators(sampling_rate = Testdata.sampling_rate)
IR = Sensordata.InputReader()


'''
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
#plots = _qOp.plot_res(attitudes, gyr_data, att_res_hand, t_, error)
#comp = _qOp.plot_comparision(att_res_hand, att_res_integrator, att_res_ekf, t_)

if input("plot? y, n: ") == "y":
    plt.show()
'''

IR.show_devices()
IR.device = 0 #int(input("select IMU #: "))
IR.rec_length = 300
#IR.calibrate(600)
#IR.wx_bias = -0.5359484696760158 #-0.3728908817918771
#IR.wy_bias = 6.677578750560675 #6.526214062387713
#IR.wz_bias = -5.771425796114199 #-5.720190872099066
q_array, t_, gyr_data, acc_data = IR.real_sensor_events(method="by hand")
plot = _qOp.plot_one(q_array, t_, "q")
plot_gyr = _qOp.plot_one(gyr_data, t_, "w")
plot_acc = _qOp.plot_one(acc_data, t_, "a")
plt.show()
print(q_array)
print(acc_data)
print(gyr_data)
print(f"biases: {IR.wx_bias, IR.wy_bias, IR.wz_bias}")