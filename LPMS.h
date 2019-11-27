#include "mbed.h"
#include "common.h"

class LPMS : public Serial {
    private:
        // Robot Parameters
        /*static const float encoder_2_global_angle = AE_TO_GLOBAL_W;       //encoder coordinate system + 30 degree    =>  global coordinate system
        static const float encoder_2_global_x     = AE_TO_GLOBAL_X;    //encoder to center distance  in x   (tung)
        static const float encoder_2_global_y     = AE_TO_GLOBAL_Y;    //encoder to center distance  in y   (tung)
        static const float pi = PI;
        
        float encoder_to_center;
        float encoder2local_angle;
        float encoder_position_angle;//90 +  angle to encoder location
        float r;//encoder to center radius*/
        Timer LPMSTimeout;
        union {
            int8_t data[56];
            float valF[14];
        } posture;
        int LPMS_counter;
        float Time,Time_L//TimeStamp
        struct position tempPos, degreePos, aeReadPos, startOffset, curPos;//For actionEncoder
        struct Vector Velocity, AngularV;
        struct Vector Gyroscope,Accelerometer,Magnetometer;
        struct Vector Gyroscope_L,Accelerometer_L,Magnetometer_L;
        struct Quad Quaternion;
        struct rotate;//x,y,z is pitch, yaw, roll angle respectively
        bool newDataArrived;
        float dt;
    public:
        LPMS(PinName tx, PinName rx);
        bool translate(char c);
        void calculatePos(float time, float GX, float GY, float gZ, float AX, float AY, float AZ, bool start = false);
        bool curPosIsAvailable();
        struct position getCurPos();
        float getR();
};





