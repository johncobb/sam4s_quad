### load firmware using edbg
```
/Users/jcobb/dev/edbg/edbg -bpv -t atmel_cm4 -f ~/dev/projects/sam4sd32c_xplained_pro/getting-started-flash.bin
```

### cli commands:

####
```
autopilot:
apx 0.000   # sets desired_angle_x
apy 0.000   # sets desired_angle_y

pid:
kp 1.0      # sets kp
ki 0.00001  # sets ki
kd 0.0001   # sets kd

imu:
imu_calibrate 1 # calibrates 

motor:
motor_armed 1       # arms motors
motor_offset 200    # motor power offset
motor_min           # sets motor to minimum power
motor_mid           # sets motor to mid power
motor_max           # sets motor to max power


logging:
log_motor 1     # toggles motor log 1/0
log_imu 1       # toggles imu log 1/0
```