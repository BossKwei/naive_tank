#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>

namespace HMC5883L
{
        int fd;
	//
	short x_min=-687,x_max=-260;
	short y_min=39,y_max=433;
	short x_offset,y_offset;
        //
        void Init()
        {
                fd = wiringPiI2CSetup(0x1e);    // HMC5883L
                wiringPiI2CWriteReg8(fd,0,0xe0);        // Config A = 0xE0
                wiringPiI2CWriteReg8(fd,2,0x00);        // Continuous Mode
                delay(500);
        }
        double GetAngle()
        {
                short dat,x,y,z;
                double angle,mag;
                //
                //dat=wiringPiI2CReadReg8(fd,9);  // Status
                //printf("Status:%x\n",dat);

                x=(wiringPiI2CReadReg8(fd,3)<<8) | wiringPiI2CReadReg8(fd,4);
                //printf("X:%d\n",x);

                y=(wiringPiI2CReadReg8(fd,7)<<8) | wiringPiI2CReadReg8(fd,8);
                //printf("Y:%d\n",y);

                z=(wiringPiI2CReadReg8(fd,5)<<8) | wiringPiI2CReadReg8(fd,6);
                //printf("Z:%d\n",z);

		//if(x>x_max) x_max=x;
		//if(x<x_min) x_min=x;
		//if(y>y_max) y_max=y;
		//if(y<y_min) y_min=y;

		x_offset = (x_max + x_min) / 2;
		y_offset = (y_max + y_min) / 2;

                angle=atan2((short)y-y_offset,(short)x-x_offset);
                //mag=sqrt(x*x+y*y+z*z);

                //printf("Angle:%f Mag:%f\n\n",angle,mag);
                return angle;
        }

}
