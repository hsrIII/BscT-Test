import numpy as np
import matplotlib.pyplot as plt
import Operators as _qOp
import Testdata
import ahrs


Testdata.print_case = False
Testdata.signal_length = 10
Testdata.sampling_rate = 0.01
_qOp.sampling_rate = Testdata.sampling_rate

# Reference Attitude
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 reference attitude")
attitudes, t_ = Testdata.orientation_testcases(case = "E")

# Sensor Data
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 02 Sensor data from reference attitude")
gyr_data, acc_data = _qOp.forwardoperator(attitudes, t_)      # Gyroscope Data, Accelerometer Data

'''
# Add Noise
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  02b Add Noise")
mu, sigma = 0, 0.001 # bias, standard deviation
gyr_data += np.random.normal(mu, sigma, gyr_data.shape)
t_ += np.random.normal(0, 0.01/10, t_.shape)
'''

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