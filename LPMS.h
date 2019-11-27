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
        Timer actionTimeout;
        union {
            int8_t data[56];
            float valF[14];
        } posture;
        int LPMS_counter;
        int i;
        struct position tempPos, degreePos, aeReadPos, startOffset, curPos;//For actionEncoder
        struct Gyroscope,Accelerometer,Magnetometer,Quaternion;
        int LastRead;
        bool newDataArrived;
        char buffer[8];
        
    public:
        LPMS(PinName tx, PinName rx);
        bool translate(char c);
        void calculatePos(float GX, float GY, float gZ, float AX, float AY, float AZ, bool start = false);
        bool curPosIsAvailable();
        struct position getCurPos();
        float getR();
};





