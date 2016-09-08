#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup_mpu() {
    // initialize device
	mpu.reset();
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void GetYPRFromMPU() {
    // if programming failed, don't try to do anything
    if (!dmpReady)
	{
		return;
	}
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
	
	//#ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        //#endif

        /*#ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            //printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
	    return q;
        #endif
	
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        #endif

        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
        printf("\n");*/
    }
}
/**********************************************************/
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define IN1 24
#define IN2 25
#define IN3 26
#define IN4 27

#define RESET 4

int gpio_low_time = 500;//ms

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    const char *a = msg->data.c_str();
	//ROS_INFO("I Heard [%s]",a);
	switch(*a)
	{
		case 'W':digitalWrite (IN1, HIGH) ;digitalWrite (IN2, LOW) ;digitalWrite (IN3, HIGH) ;digitalWrite (IN4, LOW) ;break;
		case 'S':digitalWrite (IN1, LOW) ;digitalWrite (IN2, HIGH) ;digitalWrite (IN3, LOW) ;digitalWrite (IN4, HIGH) ;break;
		case 'A':digitalWrite (IN1, HIGH) ;digitalWrite (IN2, LOW) ;digitalWrite (IN3, LOW) ;digitalWrite (IN4, HIGH) ;break;
		case 'D':digitalWrite (IN1, LOW) ;digitalWrite (IN2, HIGH) ;digitalWrite (IN3, HIGH) ;digitalWrite (IN4, LOW) ;break;
		default:break;
	}
	usleep(20000);
	gpio_low_time = 500;
}

double yaw = 0;
void* get_ypr_thread(void *)
{
	while(1)
	{
		//GetYPRFromMPU();
		yaw = HMC5883L::GetAngle();
		usleep(50000); //100ms
	}
	return NULL;
}

void* make_gpio_low(void *)
{
	while(1)
	{
		usleep(100000);//100ms
		gpio_low_time -= 100;
		if(gpio_low_time < 1)
		{
			digitalWrite (IN1, LOW);
			digitalWrite (IN2, LOW);
			digitalWrite (IN3, LOW);
			digitalWrite (IN4, LOW);
		}
	}
    return NULL;
}

void setup_gpio()
{
    wiringPiSetup();
  	pinMode (IN1, OUTPUT);
	pinMode (IN2, OUTPUT);
	pinMode (IN3, OUTPUT);
	pinMode (IN4, OUTPUT);
	digitalWrite (IN1, LOW);
	digitalWrite (IN2, LOW);
	digitalWrite (IN3, LOW);
	digitalWrite (IN4, LOW);
	//
	pinMode (RESET, OUTPUT);
	digitalWrite (RESET, HIGH);
	usleep(1000000);
	digitalWrite (RESET, LOW);
}
/**********************************************************/
int get_crc_position(char *p/*,int len=5*/)
{
    int i=0,j=-1;
    //...
    unsigned char crc = 0xAA;
    while(1)
    {
        i++;
        j = (j++ >= 9) ? 0 : j;

        //printf("i=%d,j=%d\n",i,j);

        if(0 == i%9)
        {
            if(crc == p[j])
            {
                return j;
            }
            else
            {
                crc = 0xAA;
                crc += p[j];
                i = 0;
            }
            //a circle
            if(0 == j)
            {
                return -1;
            }
        }
        else
        {
            crc += p[j];
        }

    }
}

int SerialBuffer[2] = {0};

void* uart_data_handle(void *)
{
    printf("uart_data_handle() enter!!!\n");
    int fd= 0;
    if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
	 {
	    fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	    return NULL;
	 }

	 int ch=0,flag=0,cursor=0;
	 char recvBuf[9] = {0};
	 while(1)
	 {
        int i;
        for(i=0;i<9;i++)
        {
            ch = serialGetchar(fd);
            if(-1 != ch)
            {
                recvBuf[i]=(char)ch;
            }
            else
            {
                //printf("ch == -1\n");
                break;
            }
        }
        //read 5 bytes
        if(9 == i)
        {
            cursor = get_crc_position(recvBuf);
            //printf("crc position: %d\n",cursor);
            if(cursor >= 0 && cursor < 8) //data order error
            {
                for(i=0;i<(cursor+1);i++)
                {
                    serialGetchar(fd);
                }
            }
            else if(8 == cursor) // crc success
            {
                char *p = (char *)SerialBuffer;
                //
                p[0] = recvBuf[3];
                p[1] = recvBuf[2];
                p[2] = recvBuf[1];
                p[3] = recvBuf[0];
                //
                p[4] = recvBuf[7];
                p[5] = recvBuf[6];
                p[6] = recvBuf[5];
                p[7] = recvBuf[4];
                //memcpy(SerialBuffer,recvBuf,4);//??? error
                //printf("0x%X 0x%X 0x%X 0x%X\n",recvBuf[0],recvBuf[1],recvBuf[2],recvBuf[3]);
            }
            else //no crc byte
            {
                //printf("error crc position: %d\n",cursor);
            }
        }
        else
        {
            printf("ch == -1\n");
            continue;
        }

	 }
	 if(fd) serialClose(fd);
	 printf("uart_data_handle() leave!!!\n");
	 return NULL;
}
/**********************************************************/

void* ros_subscriber_handle(void *)
{
	printf("ros_subscriber_handle() enter!!!\n");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
		ros::spinOnce();

		loop_rate.sleep();
    }
    printf("ros_subscriber_handle() leave!!!\n");
    return NULL;
}
/**********************************************************/
// Loop, getting and printing characters
void inthandle(int sig)
{
    printf("INT Signal!!!\n");
    exit(0);
}

#include <signal.h>

#include "odom.cpp"

int main(int argc, char **argv)
{
    //
    signal(SIGINT, inthandle);
    //
    HMC5883L::Init();
    //setup_mpu();
    sleep(1);
    //
    printf("setup_gpio()\n");
    setup_gpio();
    //
    printf("setup_ros()\n");
    ros::init(argc, argv, "ROSBerryPi");
    ros::NodeHandle n;
    Odometry o;
    o.InitOdometry(n);
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("cmd_vel/base", 5, chatterCallback);
    ros::Rate loop_rate(30);
    //
    pthread_t id_1,id_2,id_3,id_4;
    pthread_create(&id_1, NULL, ros_subscriber_handle, NULL);
    pthread_create(&id_2, NULL, uart_data_handle, NULL);
    pthread_create(&id_3, NULL, make_gpio_low, NULL);
    pthread_create(&id_4, NULL, get_ypr_thread, NULL);
    //
    while(ros::ok())
    {
        //o.UpdateOdometry(ypr[0],SerialBuffer[1],SerialBuffer[0]);
	o.UpdateOdometry(yaw,SerialBuffer[1],SerialBuffer[0]);
        //printf("MPU Data: %7.2f %7.2f %7.2f %7.2f\n",q.w,q.x,q.y,q.z);
        //printf("ypr %7.2f %7.2f %7.2f\n", ypr[0], ypr[1], ypr[2]);
        //printf("UART Fata: %d %d\n",SerialBuffer[0],SerialBuffer[1]);
	//sleep(100000);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
