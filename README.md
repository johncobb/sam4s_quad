## Building an autopilot for the SAM4S
This project walks you through the process of building a drone autopilot system from the ground up. This will exercise the use of I2C to interface with a gyro for balance and stability, Puls Width Modulation (PWM) for interfacing with motor controller system. The system will also use an RF interface for remote control

### Prerequisites:

### Installing avr-gcc

Installing arm-none-eabi-gcc on MacOS w/Brew
```console
brew tap osx-cross/arm
brew install arm-gcc-bin
```
verify installation
```console
arm-none-eabi-gcc --version
```


### Installing edbg
Before you start ensure automake is installed on your machine.
```console
brew install automake
```

Clone edbg from repository navigate to edbg folder and build and test
```console
https://github.com/ataradov/edbg.git
cd edbg
make all
./edbg --help
```


### load firmware using edbg
```console
/Users/johncobb/dev/edbg/edbg -bpv -t atmel_cm4 -f ~/dev/sam4s_quad/sam4s_quad.bin
```



### cli commands:


```console
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

$\cos$
$\alpha$
####