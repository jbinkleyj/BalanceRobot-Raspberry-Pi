# BalanceRobotPi
BalanceRobotPi Raspberry Pi written in c++, 
build:
make clean, qmake, make

Remote module written in android studio java , uses Rfcomm serial connection via bluetooth

Balance Robot Arduino Due or Mega

The Balance Robot uses:

18650 battery holder,

Raspberry Pi 3 

2 x 12V 37mm 350 Rpm 30:1 DC GearMotor with Encoder

MPU6050 (6-axis motion-tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer) With MPU6050 , you can get stable angle when the Kalman filter is used. Place the sensor in the lower section and center in the middle of dc motors. This prevents oscillations.

RPi Motor Driver Board for Raspberry (each channel supply 5a)

Video:https://vimeo.com/243418683

Calibrating your PID Controller

Set all PID constants to zero. This is as good a place to start as any… Slowly increase the P-constant value. While you are doing this, hold the robot to make sure it doesn’t fall over and smash into a million pieces! You should increase the P-constant until the robot responds quickly to any tilting, and then just makes the robot overshoot in the other direction. Now increase the I-constant. This component is a bit tricky to get right. You should keep this relatively low, as it can accumulate errors very quickly. In theory, the robot should be able to stabilise with only the P and I constants set, but will oscillate a lot and ultimately fall over. Raise the D-constant. A lot. The derivative components works against any motion, so it helps to dampen any oscillations and reduce overshooting.
