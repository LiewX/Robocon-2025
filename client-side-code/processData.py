import matplotlib.pyplot as plt
import numpy as np
import csv

# read the CSV file
pwm = []
ticks_motor_UL = []
ticks_motor_UR = []
ticks_motor_BL = []
ticks_motor_BR = []

#TODO: use own csv file
# read data from the CSV file
with open('messages.csv', mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header
    for row in reader:
        pwm.append(float(row[0]))
        ticks_motor_UL.append(int(row[1]))  # Ticks for UL motor
        ticks_motor_UR.append(int(row[2]))  # Ticks for UR motor
        ticks_motor_BL.append(int(row[3]))  # Ticks for BL motor
        ticks_motor_BR.append(int(row[4]))  # Ticks for BR motor

# plot the data for each motor
plt.figure(figsize=(10, 6))

# UL Motor 
plt.subplot(2, 2, 1)
plt.scatter(pwm, ticks_motor_UL, color='blue', label='UL Motor Data')
plt.title('UL Motor: PWM vs TicksPerSample')
plt.xlabel('PWM Value')
plt.ylabel('TicksPerSample')
plt.grid(True)

# UR Motor 
plt.subplot(2, 2, 2)
plt.scatter(pwm, ticks_motor_UR, color='red', label='UR Motor Data')
plt.title('UR Motor: PWM vs TicksPerSample')
plt.xlabel('PWM Value')
plt.ylabel('TicksPerSample')
plt.grid(True)

# BL Motor 
plt.subplot(2, 2, 3)
plt.scatter(pwm, ticks_motor_BL, color='green', label='BL Motor Data')
plt.title('BL Motor: PWM vs TicksPerSample')
plt.xlabel('PWM Value')
plt.ylabel('TicksPerSample')
plt.grid(True)

# BR Motor 
plt.subplot(2, 2, 4)
plt.scatter(pwm, ticks_motor_BR, color='orange', label='BR Motor Data')
plt.title('BR Motor: PWM vs TicksPerSample')
plt.xlabel('PWM Value')
plt.ylabel('TicksPerSample')
plt.grid(True)

# perform linear regression for UL Motor's data
slope_motor_UL, intercept_motor_UL = np.polyfit(pwm, ticks_motor_UL, 1)
# plot the regression line for UL Motor
plt.subplot(2, 2, 1)
plt.plot(pwm, np.array(pwm) * slope_motor_UL + intercept_motor_UL, color='black', label='Linear Fit')
plt.legend()

# perform linear regression for UR Motor's data
slope_motor_UR, intercept_motor_UR = np.polyfit(pwm, ticks_motor_UR, 1)
# plot the regression line for UR Motor
plt.subplot(2, 2, 2)
plt.plot(pwm, np.array(pwm) * slope_motor_UR + intercept_motor_UR, color='black', label='Linear Fit')
plt.legend()

# perform linear regression for BL Motor's data
slope_motor_BL, intercept_motor_BL = np.polyfit(pwm, ticks_motor_BL, 1)
# plot the regression line for BL Motor
plt.subplot(2, 2, 3)
plt.plot(pwm, np.array(pwm) * slope_motor_BL + intercept_motor_BL, color='black', label='Linear Fit')
plt.legend()

# perform linear regression for BR Motor's data
slope_motor_BR, intercept_motor_BR = np.polyfit(pwm, ticks_motor_BR, 1)
# plot the regression line for BR Motor
plt.subplot(2, 2, 4)
plt.plot(pwm, np.array(pwm) * slope_motor_BR + intercept_motor_BR, color='black', label='Linear Fit')
plt.legend()

# Show the plots
plt.tight_layout()
plt.show()

# print the gradient and y-intercept values
print(f"UL Motor - Gradient: {slope_motor_UL}, Y-Intercept: {intercept_motor_UL}")
print(f"UR Motor - Gradient: {slope_motor_UR}, Y-Intercept: {intercept_motor_UR}")
print(f"BL Motor - Gradient: {slope_motor_BL}, Y-Intercept: {intercept_motor_BL}")
print(f"BR Motor - Gradient: {slope_motor_BR}, Y-Intercept: {intercept_motor_BR}")