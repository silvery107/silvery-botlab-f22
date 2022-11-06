import numpy as np
import matplotlib.pyplot as plt
import sys

FILENAME="{}/motor_calibration/calibration_numbers.csv".format(sys.path[0])
data = np.genfromtxt(FILENAME, delimiter=',')


[ml, bl] = np.polyfit(data[6:51,1],data[6:51,0],1)
[mr, br] = np.polyfit(data[59:102,1],data[59:102,0],1)

print("LEFT_PWM  = %f * LEFT_MOTOR_SPEED_POSITIVE + %f" % (ml, bl))
print("LEFT_PWM  = %f * LEFT_MOTOR_SPEED_NEGATIVE + %f" % (mr, br))

plt.plot(data[6:51,1],data[6:51,0], 'ro', label='LEFT Motor Positive Data')
plt.plot(data[59:102,1],data[59:102,0], 'bo', label='LEFT Motor Negative Data')
plt.plot(data[6:51,1],ml*data[6:51,1]+bl, 'r', label='LEFT Motor Positive Fit')
plt.plot(data[59:102,1],mr*data[59:102,1]+br, 'b', label='LEFT Motor Negative Fit')
plt.xlabel(r'SPEED ($\frac{m}{s}$)')
plt.ylabel('PWM')
plt.legend()
plt.savefig("{}/motor_calibration/calibration_result_left.png".format(sys.path[0]))
# plt.show()

output_filename = "{}/motor_calibration/calibration_result_left.txt".format(sys.path[0])
with open(output_filename, 'w') as result_file:
    result_file.write("LEFT_PWM  = %f * LEFT_MOTOR_SPEED_POSITIVE + %f" % (ml, bl))
    result_file.write("LEFT_PWM  = %f * LEFT_MOTOR_SPEED_NEGATIVE + %f" % (mr, br))
print("Calibration result saved at motor_calibration/calibration_result_left.txt")





[ml, bl] = np.polyfit(data[108:153,1],data[108:153,0],1)
[mr, br] = np.polyfit(data[159:,1],data[159:,0],1)

print("RIGHT_PWM  = %f * RIGHT_MOTOR_SPEED_NEGATIVE + %f" % (ml, bl))
print("RIGHT_PWM  = %f * RIGHT_MOTOR_SPEED_POSITIVE + %f" % (mr, br))

plt.plot(data[108:153,1],data[108:153,0], 'ro', label='RIGHT Motor Negative Data')
plt.plot(data[159:,1],data[159:,0], 'bo', label='RIGHT Motor Positive Data')
plt.plot(data[108:153,1],ml*data[108:153,1]+bl, 'r', label='RIGHT Motor Negative Fit')
plt.plot(data[159:,1],mr*data[159:,1]+br, 'b', label='RIGHT Motor Positive Fit')
plt.xlabel(r'SPEED ($\frac{m}{s}$)')
plt.ylabel('PWM')
plt.legend()
plt.savefig("{}/motor_calibration/calibration_result_right.png".format(sys.path[0]))
# plt.show()

output_filename = "{}/motor_calibration/calibration_result_right.txt".format(sys.path[0])
with open(output_filename, 'w') as result_file:
    result_file.write("RIGHT_PWM  = %f * RIGHT_MOTOR_SPEED_NEGATIVE + %f" % (ml, bl))
    result_file.write("RIGHT_PWM  = %f * RIGHT_MOTOR_SPEED_POSITIVE + %f" % (mr, br))
print("Calibration result saved at motor_calibration/calibration_result_right.txt")