#include "LPMS.h"

LPMS::LPMS(PinName tx, PinName rx) : Serial(tx, rx, 115200)
{
    format(8, SerialBase::None, 1);
    LPMS_counter = 0;
    newDataArrived = false;

    actionTimeout.start(); //This is for the LPMS timeout, lest trapped in infinite loop/ cannot read char.
    //POTENTIAL BUG HERE. USE TIMER (MBED) FOR TIMEOUT TO ESCAPE LOOP.
    while (LPMSTimeout.read_ms() < 5000) //Wait for 5 seconds to check if it receives all information.
    {
        if (readable())
        {
            char c = getc();
            if (translate(c))
            {
                calculatePos(aeReadPos.x, aeReadPos.y, aeReadPos.w, true); //global
                break;
                
                
            }
        }
    } //start first to take offset from encoder... in case already moved
    
    actionTimeout.stop(); //Stop the timer, since it has no use afterwards.
}


bool LPMS::translate(char c)//The pc.putc is for debug use
{
    //////////////////////
    switch(LPMS_counter){
    case 0:
        if(IMU.getc() == 0x3A)
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 1:
        if(IMU.getc() == 0x01)
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 2:
        if(IMU.getc() == 0x00)
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 3:
        if(IMU.getc() == 0x09)
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 4:
        if(IMU.getc() == 0x00)
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 5:
        if(IMU.getc() == 0x38)//56 bytes
        {LPMS_counter ++;}
        else LPMS_counter = 0;
        break;
    case 6:
        if(IMU.getc() == 0x00)//56 bytes
        {LPMS_counter ++;
        }
        else LPMS_counter = 0;
        break;
    case 7:
        for(int k=0;k<56;k++)
        {posture.data[k] = IMU.getc();
        pc.putc(posture.data[k]);
        LPMS_counter++;
        }
        //pc.printf("\nTimestamp:'%f'",posture.valF[0]);
        //pc.printf("\nGyroscope:'%f','%f','%f'",posture.valF[1],posture.valF[2],posture.valF[3]);
        //pc.printf("\nAccelerometer: '%f','%f','%f'",posture.valF[4],posture.valF[5],posture.valF[6]);
        //pc.printf("\nMagnetometer: '%f','%f','%f'",posture.valF[7],posture.valF[8],posture.valF[9]);
        //pc.printf("\nQuaternion: '%f','%f','%f'",posture.valF[10],posture.valF[11],posture.valF[12]);
        Time = posture.valF[0];
        Gyroscope.x = posture.valF[1];
        Gyroscope.y = posture.valF[2];
        Gyroscope.z = posture.valF[3];
        Accelerometer.x = posture.valF[4];
        Accelerometer.y = posture.valF[5];
        Accelerometer.z = posture.valF[6];
        Magnetometer.x = posture.valF[7];
        Magnetometer.y = posture.valF[8];
        Magnetometer.z = posture.valF[9];
        Quaternion.x = posture.valF[10];
        Quaternion.y = posture.valF[10];
        Quaternion.z = posture.valF[11];
        newDataArrived = true;
        LPMS_counter = 0;
        return true;
    default:
        LPMS_counter = 0;
        break;
        }
    return false;

void LPMS::calculatePos(float time, float GX, float GY, float gZ, float AX, float AY, float AZ, bool start)
{
    if (start)
    {
        /*
        startOffset.x = tx + y_tbias; //-
        startOffset.y = ty - x_tbias;
        //startOffset.x = tx + y_tbias ;
        //startOffset.y = ty - x_tbias ;  //+
        startOffset.w = _A * pi / float(180); //degree 2 rad
        */
        Time_L = time;
    }
    else
    {
        dt = time - Time_L;
        Time_L = time;
        Velocity.x = Velocity.x + AX;
        /*
        curPos.x = tx + y_tbias - startOffset.x; //-
        curPos.y = ty - x_tbias - startOffset.y;
        //curPos.x = tx + y_tbias   - startOffset.x;  //+
        //curPos.y = ty - x_tbias   - startOffset.y;
        curPos.w = _A * pi / float(180) - startOffset.w;*/
    }
}

bool LPMS::curPosIsAvailable()
{
    if (readable())
    {
        char c = getc();
        if (translate(c))
        {
            calculatePos(Gyroscope.x, Gyroscope.y, Gyroscope.w, Accelerometer.x, Accelerometer.y, Accelerometer.w);
            return true;
        }
    }
    return false;
}

struct position LPMS::getCurPos()
{
    return curPos;
}
/*
float LPMS::getR()
{
    return r;
}*/







