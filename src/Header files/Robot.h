#include "HardwareSerial.h"
#include "Arduino.h"
#include "MultiStepper.h"
#include "AccelStepper.h"
#include <string.h>
#include <vector>
using namespace std;
/*Robot's pins*/
#define en PC1
#define dirX PC0
#define stepX PB9
#define dirY PB8
#define stepY PB7
#define dirZ PB6
#define stepZ PB5
#define endX PB14
#define endY PB13
#define endZ PB12
//#define LED_BUILTIN PC8
#define pump PA9
#define valve PA11
/**/

extern float POINT_A[3];
extern float POINT_B[3];
extern float POINT_C[3];
extern int   tProcess;

/*Size of Teaching, Moving and Box array*/

/**/


/*The tasks to do in the interrupt */
enum PROCESS_t 
{
  NONE,
  CIRCLE,
  SCURVE,
  WARNING,
  CAL_MOVING,
  RUN_AUTO
};
/**/

class Robot_Control {
  private:


  public:
    /*Robot control varialbes*/
    float mVelMax;
    float mAccelMax;
    float mPosMax;
    float mMidStep1;
    float mMidStep2;
    float mMidStep3;
    float mTime;  //current time
    float mA1;
    float mA2, mB2;
    float mA3, mB3, mC3;

    float mKx;
    float mKy;
    float mKz;

    float mS1;
    float mS2;
    float cVel;
    int mProcess;
    float cPos;

    float mT_Total;  //total travelling time in s curve
    float mT_a;      //Ta in s curve
    float mRadius;
    /*Teaching variables*/
    int running_point; //the current point that robot is running toward(RUN_AUTO mode)


    void Config(int v, int a);
    //HIGH chạy xuống, LOW chạy lên
    void GetPosition(float theta1, float theta2, float theta3);
    void Calib_home();
    void GetPositionConst(float theta1, float theta2, float theta3, float v1, float v2, float v3);
    void EofGetPos_sCurve(); //S curve motion profile
    void EofGetCircle();  //circle path planning
    void SaveTeachingPos(int point_num, float t_x, float t_y, float t_z, float t_vel, float t_time); //save current position to wanted "cell"
    void SaveMovingPos(int point_num, float m_x, float m_y, float m_z, float m_vel, float m_time);
    void SaveObjPos(int b_i, float b_x, float b_y, float b_z, float b_time);
    void ClearTeachingPos(int point_num);
    void ClearMovingPos(int point_num);
    void ClrAllPos();
    void ClrObjPos();
    void MovingProcess();
    Robot_Control(int vx, int ax, int vy, int ay, int vz, int az);

};
