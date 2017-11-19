# BalanceRobotPi
BalanceRobotPi Raspberry Pi written in c++ <br />

Building c code: <br />
qmake, make <br />

Remote module written in android studio java , uses Rfcomm serial connection via bluetooth <br />
Please firstly run remote android app and kepp it in listening mode. <br />
And run raspberry pi code with sudo. The default start mode is off.  <br />
You will press start button from android app.. <br />

For autostart please read below instructions. <br />

The Balance Robot parts are:

18650 battery holder,

Raspberry Pi 3 

2 x 12V 37mm 350 Rpm 30:1 DC GearMotor with Half Encoder

MPU6050 (6-axis motion-tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer) With MPU6050 , you can get stable angle when the Kalman filter is used. Place the sensor in the lower section, middle part of robot.
I put it in the middle of two dc motors. This prevents oscillations.

RPi Motor Driver Board for Raspberry (each channel supply 5a)<br />
https://www.waveshare.com/wiki/RPi_Motor_Driver_Board<br />

//physcal pins<br />
#define PWMR1  31  IN1 <br />
#define PWMR2  33  IN2 <br />
#define PWML1  38  IN3<br />
#define PWML2  40  IN4<br />
#define PWMR   32  PWMR<br />
#define PWML   37  PWML<br />
<br />
//encoder define<br />
#define SPD_INT_L 12   //interrupt R Phys:12 A<br />
#define SPD_PUL_L 16   //Phys:16<br />
#define SPD_INT_R 18   //interrupt L Phys:18 B<br />
#define SPD_PUL_R 22   //Phys:22<br />
<br />
Video:https://vimeo.com/243418683

https://www.youtube.com/watch?v=WB55xWTzZrw

AutoStart the app on boot:<br />

I assume that your code and exec (bin) will be in /home/pi/BalanceRobotPi folder.<br />

create a script start.sh on your home folder (home/pi)<br />
place below code in it.<br />

#!/bin/bash<br />
sudo chown root.root /home/pi/BalanceRobotPi/BalanceRobot<br />
sudo chmod 4755 /home/pi/BalanceRobotPi/BalanceRobot<br />
cd /home/pi/BalanceRobotPi<br />
./BalanceRobot<br />

Open a sample unit file using the command as shown below:<br />

sudo nano /lib/systemd/system/startrobot.service<br />
Add in the following text :<br />

 [Unit]<br />
 Description=My Robot Service<br />
 After=multi-user.target<br />
<br />
 [Service]<br />
 Type=idle<br />
 ExecStart=/home/pi/start.sh<br />
<br />
 [Install]<br />
 WantedBy=multi-user.target<br />
 <br />
You should save and exit the nano editor.<br />
<br />
The permission on the unit file needs to be set to 644 :<br />
sudo chmod 644 /lib/systemd/system/startrobot.service<br />
<br />
Configure systemd<br />
Now the unit file has been defined we can tell systemd to start it during the boot sequence :<br />
sudo systemctl daemon-reload<br />
sudo systemctl enable startrobot.service<br />
Reboot the Pi and your custom service should run:<br />
sudo reboot<br />
<br />
Calibrating your PID Controller

Set all PID constants to zero. This is as good a place to start as any… Slowly increase the P-constant value. While you are doing this, hold the robot to make sure it doesn’t fall over and smash into a million pieces! You should increase the P-constant until the robot responds quickly to any tilting, and then just makes the robot overshoot in the other direction. Now increase the I-constant. This component is a bit tricky to get right. You should keep this relatively low, as it can accumulate errors very quickly. In theory, the robot should be able to stabilise with only the P and I constants set, but will oscillate a lot and ultimately fall over. Raise the D-constant. The derivative components works against any motion, so it helps to dampen any oscillations and reduce overshooting.
