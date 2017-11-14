#include <QCoreApplication>

#include "Pid.h"
#include <stdint.h>
#include <iostream>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <softPwm.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"
#include <math.h>
#include <sys/time.h>
#include "ComPacket.h"
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/sdp.h>
#include <bluetooth/hci.h>
#include <bluetooth/sdp_lib.h>
#include <bluetooth/rfcomm.h>
#include <signal.h>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pthread.h>
#include <wiringPi.h>

//#include <wiringPiI2C.h>
//sudo apt-get install libbluetooth-dev

#define SLEEP_PERIOD 1000 //us;
#define SERIAL_TIME 50 //ms
#define SAMPLE_TIME 10 //ms

#define KF_VAR_ACCEL 0.0075 // Variance of pressure acceleration noise input.
#define KF_VAR_MEASUREMENT 0.05
#define RESTRICT_PITCH

//physcal pins
#define PWMR1  31
#define PWMR2  33
#define PWML1  38
#define PWML2  40
#define PWMR  32
#define PWML  37

//encoder define
#define SPD_INT_L 12   //interrupt R Phys:12
#define SPD_PUL_L 16   //Phys:16
#define SPD_INT_R 18   //interrupt L Phys:18
#define SPD_PUL_R 22   //Phys:22

MPU6050 accelgyro;
Kalman kalmanX;
Kalman kalmanY;
std::string RfCommAndroidMac = "5C:2E:59:D6:67:4B";// change it with your phone mac

bool m_IsRunning = false;
bool m_IsMainThreadRunning = false;
bool m_IsSerialThreadRunning = false;

char buf[500];
ComPacket SerialPacket;

int fd; //rfcomm0
int Speed_L,Speed_R;
int mSpeed,pwm_l,pwm_r;
int Speed_Need = 0;
int Turn_Need = 0;
int Speed_Diff = 0;
int Speed_Diff_ALL = 0;
int Position_AVG = 0;
int pwnLimit = 255;

uint32_t timer;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double RAD_TO_DEG = 57.2958;
double timediff = 0.0;
double Correction = 0.0;
double Setpoint = 0.0;
double aggKp = 50.0;
double aggKi = 75.0;
double aggKd = 2.0;
double aggVs = 10.0; //Velocity wheel
double aggKm = 1.0; //Velocity wheel
double angle_error = 0.0;
double Angle_MPU = 0.0;
double Gyro_MPU = 0.0;
double Temperature = 0.0;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double Input, Output;

double DataAvg[3];

//Specify the links and initial tuning parameters
PID balancePID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

template<typename T>
  T StringToNumber(const std::string& numberAsString)
  {
     T valor;

     std::stringstream stream(numberAsString);
     stream >> valor;
     if (stream.fail()) {
        std::runtime_error e(numberAsString);
        throw e;
     }
     return valor;
  }

  template<class T>
  const T& constrain(const T& x, const T& a, const T& b) {
      if(x < a) {
          return a;
      }
      else if(b < x) {
          return b;
      }
      else
          return x;
  }

unsigned int millis()
{
    struct timeval timer;
    gettimeofday(&timer, NULL);
    double time_in_mill = (timer.tv_sec) * 1000 + (timer.tv_usec) / 1000 ; // convert tv_sec & tv_usec to millisecond
    return (unsigned int)time_in_mill;
}

unsigned int micros()
{
    struct timeval timer;
    gettimeofday(&timer, NULL);
    unsigned int time_in_micros = 1000000 * timer.tv_sec + timer.tv_usec;
    return time_in_micros;
}

void ResetValues()
{
    Setpoint = 0.0;
    Speed_Diff = 0;
    Speed_Diff_ALL = 0;
    Position_AVG = 0;
    Input = 0.0;
    Angle_MPU = 0.0;
    Gyro_MPU = 0.0;
    Temperature = 0.0;
    Speed_Need = 0;
    Turn_Need = 0;
    Correction = 0.0;
    Speed_L = 0;
    Speed_R = 0;
    mSpeed = 0;
    pwm_l = 0;
    pwm_r = 0;
}

bool connetRfComm()
{
    struct termios options;

    // linux
    fd = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NONBLOCK);

    /* get the current options */
    tcgetattr(fd, &options);

    /* Set Baud Rate */
    cfsetospeed (&options, (speed_t)B9600);
    cfsetispeed (&options, (speed_t)B9600);

    /* Setting other Port Stuff */
    options.c_cflag     &=  ~PARENB;            // Make 8n1
    options.c_cflag     &=  ~CSTOPB;
    options.c_cflag     &=  ~CSIZE;
    options.c_cflag     |=  CS8;

    options.c_cflag     &=  ~CRTSCTS;           // no flow control
    options.c_cc[VMIN]   =  1;                  // read doesn't block
    options.c_cc[VTIME]  =  1;                  // 0.1 seconds read timeout
    options.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&options);

    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );
    if ( tcsetattr ( fd, TCSANOW, &options ) != 0)
    {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
       return false;
    }

    return true;
}

void disConnetRfComm() {

    // If the port is actually open, close it
    if (fd != -1) {
        close(fd);
        printf("RfComm: Device %d is now closed.\n", fd);
    }
}

uchar *trim(uchar *s)
{
  uchar *first;
  uchar *last;

  first = s;
  while(isspace(*first))
    ++first;

  last = first + strlen((const char*)first) - 1;
  while(last > first && isspace(*last))
    --last;

  memmove(s, first, last - first + 1);
  s[last - first + 1] = '\0';

  return s;
}

///SETTINGS.INI FUNCTIONS

void getStatusFromConfigurationFile() {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini("settings.ini", pt);

    aggKp       = StringToNumber<double>(pt.get<std::string>("Robot.aggKp"));
    aggKi       = StringToNumber<double>(pt.get<std::string>("Robot.aggKi"));
    aggKd       = StringToNumber<double>(pt.get<std::string>("Robot.aggKd"));
    aggVs       = StringToNumber<double>(pt.get<std::string>("Robot.aggVs"));
    aggKm       = StringToNumber<double>(pt.get<std::string>("Robot.aggKm"));
    Correction  = StringToNumber<double>(pt.get<std::string>("Robot.Correction"));
    RfCommAndroidMac = pt.get<std::string>("Robot.RfCommAndroidMac");

    printf("aggKp: %f aggKi: %f aggKd: %f\n",aggKp,aggKi,aggKd);

}

void updateConfigurationFileFromStatus() {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini("settings.ini", pt);

    printf("aggKp: %.1f aggKi: %.1f aggKd: %.1f\n",aggKp,aggKi,aggKd);

    pt.put("Robot.aggKp", aggKp);
    pt.put("Robot.aggKi", aggKi);
    pt.put("Robot.aggKd", aggKd);
    pt.put("Robot.aggVs", aggVs);
    pt.put("Robot.aggKm", aggKm);
    pt.put("Robot.Correction", Correction);
    pt.put("Robot.RfCommAndroidMac", RfCommAndroidMac);

    boost::property_tree::ini_parser::write_ini("settings.ini", pt);
    printf("settings.ini file updated.\n");
}

void createConfigurationFile() {
    boost::property_tree::ptree pt;

    printf("aggKp: %.1f aggKi: %.1f aggKd: %.1f\n",aggKp,aggKi,aggKd);

    //create with default parameters
    pt.put("Robot.aggKp", aggKp);
    pt.put("Robot.aggKi", aggKi);
    pt.put("Robot.aggKd", aggKd);
    pt.put("Robot.aggVs", aggVs);
    pt.put("Robot.aggKm", aggKm);
    pt.put("Robot.Correction", Correction);
    pt.put("Robot.RfCommAndroidMac", RfCommAndroidMac);

    boost::property_tree::ini_parser::write_ini("settings.ini", pt);
    printf("settings.ini file created.\n\n");
}

void initConf()
{
    try
    {
        getStatusFromConfigurationFile();
    }
    catch (const std::exception& e)
    {
        printf("File settings.ini not found.\nCreating settings.ini file.\n");
        createConfigurationFile();
        getStatusFromConfigurationFile();
    }
}

void updateConf()
{
    try
    {
          updateConfigurationFileFromStatus();
    }
    catch (const std::exception& e)
    {
        printf("Exception handled: %s.", e.what());
    }
}

/////////////////

bool sendData(char *data, unsigned int buf_size)
{
    if (fd != -1) {
        return write(fd, data, buf_size);
    }
    else return -1;
}

void RobotDirection()
{
  unsigned char Speed = SerialPacket.m_Buffer[1];
  if(SerialPacket.m_Buffer[3] == 0x07)
  {
      m_IsRunning = true, printf("started...\n");ResetValues();
      return;
  }
  else if(SerialPacket.m_Buffer[3] == 0x08)
  {
      m_IsRunning = false; printf("stopped...\n");ResetValues();
      return;
  }

  switch(SerialPacket.m_Buffer[3])
  {
    case 0x00: Speed_Need = 0;Turn_Need = 0;break;
    case 0x01: Speed_Need = -Speed; break;
    case 0x02: Speed_Need = Speed; break;
    case 0x03: Turn_Need = Speed; break;
    case 0x04: Turn_Need = -Speed; break;
    case 0x05: Correction = Correction + 0.1; break;
    case 0x06: Correction = Correction - 0.1; break;
    default:break;
  }

  updateConf();
  sprintf(buf,"Robot Direction(0x%02X): Sn: %d Tn: %d Cr: %0.2f",SerialPacket.m_Buffer[3],Speed_Need,Turn_Need,Correction);
  printf("%s\n",buf);
}

void UpdatePID()
{  
  unsigned int Upper,Lower;
  double NewPara;
  Upper = SerialPacket.m_Buffer[2];
  Lower = SerialPacket.m_Buffer[1];
  NewPara = (float)(Upper<<8 | Lower)/100.0;

  switch(SerialPacket.m_Buffer[3])
  {
    case 0x01:aggKp = NewPara;break;
    case 0x02:aggKi = NewPara;break;
    case 0x03:aggKd = NewPara;break;
    case 0x04:aggVs = NewPara;break;
    case 0x05:aggKm = NewPara;break;
    default:break;
  }

  updateConf();
  sprintf(buf,"Update PID(0x%02X) Kp: %0.2f Ki: %0.2f Kd: %0.2f  Vs: %0.2f Km: %0.2f",SerialPacket.m_Buffer[3],aggKp,aggKi,aggKd,aggVs,aggKm);
  printf("%s\n",buf);
}


void UserComunication()
{
  if(SerialPacket.m_PackageOK == true)
  {
    SerialPacket.m_PackageOK = false;

    switch(SerialPacket.m_Buffer[4])
    {
      case 0x01:  break;
      case 0x02:  UpdatePID();break;
      case 0x03:  RobotDirection();break;
      case 0x04:  break;
      case 0x05:  break;
      case 0x06:  break;
      case 0x07:  break;
      default:    break;
    }
  }
}

void MySerialEvent(uchar c)
{
      uchar tmp = 0;

      for(int i = 5; i > 0; i--)
      {
         SerialPacket.m_Buffer[i] = SerialPacket.m_Buffer[i-1];
      }

      SerialPacket.m_Buffer[0] = c;

      if(SerialPacket.m_Buffer[5] == 0xAA)
      {
         tmp = SerialPacket.m_Buffer[1]^SerialPacket.m_Buffer[2]^SerialPacket.m_Buffer[3];
         if(tmp == SerialPacket.m_Buffer[0])
         {
           SerialPacket.m_PackageOK = true;

           UserComunication();
         }
      }
}

int getData(uchar* data) {

    int n = 0,
    spot = 0;
    char buf = '\0';
    char response[1024];
    memset(response, '\0', sizeof response);

    // If the port is actually open, read the data
    if (fd != -1) {

        do {
            n = read( fd, &buf, 1 );
            MySerialEvent((uchar)buf);
            sprintf( &response[spot], "%c", buf );
            spot += n;
        } while( buf != '\r' && n > 0);

        memcpy(data,response,sizeof response);

        // Grab the data and return the nubmer of bytes actually read
        return spot;
        //return read(fd, data, sizeof(data));
    }
    // Port is closed!
    else return -1;
}

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

int sdp_search_spp(sdp_session_t *sdp, uint8_t *channel)
{
    int result = -1;
    sdp_list_t *srch, *attrs, *rsp;
    uuid_t svclass;
    uint16_t attr;
    int err;

    if (!sdp)
        return -1;

    sdp_uuid16_create(&svclass, SERIAL_PORT_SVCLASS_ID);
    srch = sdp_list_append(NULL, &svclass);

    attr = SDP_ATTR_PROTO_DESC_LIST;
    attrs = sdp_list_append(NULL, &attr);

    err = sdp_service_search_attr_req(sdp, srch, SDP_ATTR_REQ_INDIVIDUAL, attrs, &rsp);
    if (err)
        return -1;

    sdp_list_t *r = rsp;

   // go through each of the service records
   for (; r; r = r->next ) {
       sdp_record_t *rec = (sdp_record_t*) r->data;
       sdp_list_t *proto_list;

       // get a list of the protocol sequences
       if( sdp_get_access_protos( rec, &proto_list ) == 0 ) {
       sdp_list_t *p = proto_list;

       // go through each protocol sequence
       for( ; p ; p = p->next ) {
           sdp_list_t *pds = (sdp_list_t*)p->data;

           // go through each protocol list of the protocol sequence
           for( ; pds ; pds = pds->next ) {

               // check the protocol attributes
               sdp_data_t *d = (sdp_data_t*)pds->data;
               int proto = 0;
               for( ; d; d = d->next ) {
                   switch( d->dtd ) {
                       case SDP_UUID16:
                       case SDP_UUID32:
                       case SDP_UUID128:
                           proto = sdp_uuid_to_proto( &d->val.uuid );
                           break;
                       case SDP_UINT8:
                           if( proto == RFCOMM_UUID ) {
                               *channel=d->val.int8;
                               result = 0;
                           }
                           break;
                   }
               }
           }
           sdp_list_free( (sdp_list_t*)p->data, 0 );
       }
       sdp_list_free( proto_list, 0 );
       }
       sdp_record_free( rec );
   }
   sdp_close(sdp);
   return result;
}

bool bindRFComm(const char* dest)
{
     bdaddr_t my_bdaddr_any = {0, 0, 0, 0, 0, 0};
     bdaddr_t target;
     sdp_session_t *session = 0;
     str2ba(dest, &target );

     //connect to the SDP server running on the remote machine
     session = sdp_connect( &my_bdaddr_any, &target, 0 );
     u_int8_t channel;

     if(sdp_search_spp(session,&channel) == 0)
     {
          printf("Found rfcomm on channel: %d\n",channel);
          sprintf(buf,"rfcomm bind 0 %s %d",dest,channel);
          printf("%s\n",buf);
          std::string bind = exec(buf);
          printf("%s\n",bind.c_str());
     }
     else
     {
          printf("Rfcomm service not found\n");
          return false;
     }

    return true;
}


void calculateGyro()
{
    timediff = (micros() - timer)/1000;
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time

    timer = micros();

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // display accel/gyro x/y/z values
    //printf("accel/gyro: %6hd %6hd %6hd   %6hd %6hd %6hd\n",ax,ay,az,gx,gy,gz);

    accX = (int16_t)(ax);
    accY = (int16_t)(ay);
    accZ = (int16_t)(az);

    gyroX = (int16_t)(gx);
    gyroY = (int16_t)(gy);
    gyroZ = (int16_t)(gz);

    // It is then converted from radians to degrees
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

    #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif

    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    DataAvg[2] = DataAvg[1];
    DataAvg[1] = DataAvg[0];
    DataAvg[0] = kalAngleX;

    Angle_MPU = (DataAvg[0]+DataAvg[1]+DataAvg[2])/3;

    Gyro_MPU = gyroXrate;
    Temperature = (double)accelgyro.getTemperature() / 340.0 + 36.53;
    //printf("Angle_MPU: %.2f  Time_Diff: %.1f\n",Angle_MPU,timediff);

}


/*

float K = 1.064 ;       //1.9 * 1.12;  // wheels 80mm
//float K = 1.9 ;      // wheels 100mm
float Kp = 15;
float Ki = 6.9;
float Kd = 20;
float Kp_Wheel = -0.025;
float Kd_Wheel = -9;

int last_error = 0;
int integrated_error = 0;
float pTerm=0, iTerm=0, dTerm=0, pTerm_Wheel=0, dTerm_Wheel=0;

int updatePid(float targetPosition, float currentPosition)   {
  float error = targetPosition - currentPosition;
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);
  last_error = error;
  pTerm_Wheel = Kp_Wheel * count;           //  -(Kxp/100) * count;
  dTerm_Wheel = Kd_Wheel * (count - last_count);
  last_count = count;
  return -constrain(K*(pTerm + iTerm + dTerm + pTerm_Wheel + dTerm_Wheel), -255, 255);
}
}*/

void encodeL (void)
{
    if (digitalRead(SPD_PUL_L))
        Speed_L += 1;
    else
        Speed_L -= 1;
}

void encodeR (void)
{
    if (digitalRead(SPD_PUL_R))
        Speed_R += 1;
    else
        Speed_R -= 1;
}

void PWM_Calculate()
{
    //forward: l= - ; r= +
    Speed_Diff = Speed_R + Speed_L;
    Input = Angle_MPU;

    angle_error = abs(Setpoint - Input); //distance away from setpoint

    if (angle_error < 10)
    {   //we're close to setpoint, use conservative tuning parameters
        balancePID.SetTunings(aggKp/3, aggKi/3, aggKd/3);
    }
    else
    {   //we're far from setpoint, use aggressive tuning parameters
        balancePID.SetTunings(aggKp, aggKi, aggKd);
    }

    balancePID.Compute();   

    mSpeed = -1*(int)Output;

    pwm_l = mSpeed + aggVs * Speed_Diff;
    pwm_r = mSpeed - aggVs * Speed_Diff;

    printf("Angle_MPU: %.2f  pwm_r: %d pwm_l: %d  Speed_Diff: %d  Speed_L: %d  Speed_R: %d\n",Angle_MPU,pwm_r,pwm_l,Speed_Diff,Speed_L,Speed_R);

    Speed_L = 0;
    Speed_R = 0;
}

void Robot_Control()
{
   /* M1 	M2 	M3 	M4 	Descriptions
    1 	0 	1 	0 	When the motors rotate forwards, the robot goes straight
    0 	1 	0 	1 	When the motors rotate backwards, the robot draws back
    0 	0 	1 	0 	When the right motor stops and left motor rotates forwards, the robot turns right
    1 	0 	0 	0 	When the left motor stops and right motor rotates forwards, the robot turns left
    0 	0 	0 	0 	When the motors stop, the robot stops*/

    if (pwm_r>0)
    {
        digitalWrite(PWMR1, LOW);
        digitalWrite(PWMR2, HIGH);
    }

    if (pwm_l>0)
    {
        digitalWrite(PWML1, HIGH);
        digitalWrite(PWML2, LOW);
    }

    if (pwm_r<0)
    {
        digitalWrite(PWMR1, HIGH);
        digitalWrite(PWMR2, LOW);
        pwm_r =- pwm_r;  //cchange to positive
    }

    if (pwm_l<0)
    {
        digitalWrite(PWML1, LOW);
        digitalWrite(PWML2, HIGH);
        pwm_l = -pwm_l;
    }

    sprintf(buf, "Data:%d:%d:%0.2f:%d:%d:%d:%d:%0.2f:%0.2f:%0.2f:%0.2f:%0.2f:%0.2f:%0.2f:%0.2f:",
    pwm_l, pwm_r,Angle_MPU,Speed_Need,Turn_Need,Speed_L,Speed_R,aggKp,aggKi,aggKd,aggVs,aggKm,Temperature,Correction,angle_error);

    if( Angle_MPU > 45 || Angle_MPU < -45 || !m_IsRunning)
    {
        pwm_l = 0;
        pwm_r = 0;
    }

    softPwmWrite(PWML, pwm_l);
    softPwmWrite(PWMR, pwm_r);

}

PI_THREAD (serialThread)
{
    while (m_IsSerialThreadRunning)
    {
        uchar buffer[128];
        getData(buffer);        
        sendData(buf,strlen(buf));

        ::usleep(SLEEP_PERIOD * SERIAL_TIME);
    }
}

PI_THREAD (mainThread)
{   
    while (1)
    {
        if(!m_IsMainThreadRunning)
        {
            softPwmWrite(PWML, 0);
            softPwmWrite(PWMR, 0);
            continue;
        }

        calculateGyro();
        PWM_Calculate();
        Robot_Control();

        ::usleep(SLEEP_PERIOD * SAMPLE_TIME);
    }
}

void initRfcomm()
{
    bool rfcomm_connected = false;
    printf("\nTrying to connect rfcomm port...\n");

    if(!connetRfComm())
    {
        fprintf (stderr, "Unable to connet rfcomm0: %s\n", strerror (errno));
        printf("Trying to bind rfcomm port...\n");

        if(bindRFComm(RfCommAndroidMac.c_str()))
        {
            if(connetRfComm())
            {
                rfcomm_connected = true;
                printf("Connet rfcomm0 successful.\n\n");
            }
        }
        else
            fprintf (stderr, "Unable to connet rfcomm0: %s\n", strerror (errno));
    }
    else
    {
         rfcomm_connected = true;
         printf("Connet rfcomm0 successful.\n\n");
    }

    if(rfcomm_connected)
    {
        m_IsSerialThreadRunning = true;
        piThreadCreate (&serialThread) ;
    }

}

void init()
{
    bool isMPU6050_Found = false;
    m_IsMainThreadRunning = false;
    m_IsSerialThreadRunning = false;

    // initialize device
    printf("\nInitializing I2C devices.\n");
    accelgyro.initialize();

    if(accelgyro.testConnection())
    {
        printf("MPU6050 connection successful.\n\n" );
        isMPU6050_Found = true;
    }
    else
    {
        printf("MPU6050 connection failed.\n\n");
    }

    if (wiringPiSetupPhys () < 0)
    {
        fprintf (stderr, "Unable to setup wiringPiSetupGpio: %s\n\n", strerror (errno)) ;
    }
    else
    {
        pinMode(PWML1, OUTPUT);
        pinMode(PWML2, OUTPUT);
        pinMode(PWMR1, OUTPUT);
        pinMode(PWMR2, OUTPUT);

        printf("Set pinModes ok.\n");

        softPwmCreate(PWML,0,pwnLimit);
        softPwmCreate(PWMR,0,pwnLimit);

        if (wiringPiISR (SPD_INT_L, INT_EDGE_FALLING, &encodeL) < 0)
        {
         fprintf (stderr, "Unable to setup ISR for left channel: %s\n", strerror (errno));
         return;
        }
        else
        {
         printf("Setup encodeL for left channel successful.\n");
        }

        if (wiringPiISR (SPD_INT_R, INT_EDGE_FALLING, &encodeR) < 0)
        {
         fprintf (stderr, "Unable to setup ISR for right channel: %s\n", strerror (errno));
         return;
        }
        else
        {
         printf("Setup encodeR for right channel successful.\n");
        }

        printf("wiringPiSetupPhys ok.\n\n");

        initRfcomm();

        initConf();

        balancePID.SetMode(AUTOMATIC);
        balancePID.SetSampleTime(SAMPLE_TIME);
        balancePID.SetOutputLimits(-pwnLimit, pwnLimit);        

        DataAvg[0]=0; DataAvg[1]=0; DataAvg[2]=0;

        if(isMPU6050_Found)
        {
            m_IsMainThreadRunning = true;
            piThreadCreate (&mainThread) ;
        }
    }

    timer = micros();
}


void  ExitHandler(int sig)
{
     signal(sig, SIG_IGN);
     m_IsMainThreadRunning = false;
     m_IsSerialThreadRunning = false;
     usleep(1000 * 100);

     printf("\b\bExiting...\n");

     softPwmWrite(PWML, 0);
     softPwmWrite(PWMR, 0);
     exit(0);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    signal(SIGINT, ExitHandler);

    if (getuid())
    {
        printf("%s", "You must be root!\n");
        exit(0);
    }
    else
    {
        ResetValues();
        init();
    }
    return a.exec();
}

/*void readMPU6050() // alternate read i2c method
{
    int fd;
    fd = wiringPiI2CSetup (0x68);
    wiringPiI2CWriteReg8 (fd,0x6B,0x00);//set the sleep unenable
    printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

    while(1)
    {
        printf("My valueX:%X+%X\n",wiringPiI2CReadReg8(fd, 0x43),wiringPiI2CReadReg8(fd, 0x44));
        printf("My valueY:%X+%X\n",wiringPiI2CReadReg8(fd, 0x45),wiringPiI2CReadReg8(fd, 0x46));
        printf("My valueZ:%X+%X\n---------------------\n",wiringPiI2CReadReg8(fd, 0x47),wiringPiI2CReadReg8(fd, 0x48));

        printf("My valueX:%X\n",wiringPiI2CReadReg16(fd, 0x43));
        printf("My valueY:%X\n",wiringPiI2CReadReg16(fd, 0x45));
        printf("My valueZ:%X\n------------------------\n",wiringPiI2CReadReg16(fd, 0x47));

        delay(1000);
    }
}*/

