#include <Arduino.h>
#include "Header files/Robot.h"
#include "Header files/Kinematics.h"
#define null 0
Robot_Control Robot(20000,20000,20000,20000,20000,20000);
Kinematics Kin;
HardwareTimer timer1(TIM1);
HardwareTimer timer2(TIM4);
HardwareSerial serial4(UART4);
extern AccelStepper stepperX;
extern AccelStepper stepperY;
extern AccelStepper stepperZ;
float arr[5];
char buff[25];
int tProcess = NONE;
float POINT_A[3],POINT_B[3],POINT_C[3]; 
int curr_mp; //current moving point
int m_point;//the number of moving points displayed on screen (point 1, point 2, point 3)

/*Vectors in Robot.cpp flie*/
extern vector <vector<float>> MovingPoints;
extern vector <vector<float>> TeachingPoints;
extern vector <vector<float>> ObjPoints;
void HAL_TIM1_CallBack();
void data();
void displayVector(float type_array);
/**/


void setup() {
  timer1.pause();
  timer1.setOverflow(1000,HERTZ_FORMAT);
  timer1.attachInterrupt(HAL_TIM1_CallBack);
  timer1.resume();
  // timer2.pause();
  // timer2.setOverflow(10,HERTZ_FORMAT);
  // timer2.attachInterrupt(data);
  // timer2.resume();
  serial4.begin(9600);//khởi tạo PA9,PA10
  pinMode(LED_BUILTIN, OUTPUT);
  // Robot.Calib_home();
}

void loop() {
  
  while(serial4.available())
  {
    
    String var = serial4.readStringUntil('\n');
    int str_len = var.length() + 1; 
    char char_array[str_len];
    var.toCharArray(char_array, str_len);
    char *s0= strtok(char_array," ");
    char *s1= strtok(NULL," ");
    char *s2= strtok(NULL," ");
    char *s3= strtok(NULL," ");
    char *s4= strtok(NULL," ");
    String s11=String(s1);
    float s22=String(s2).toFloat();
    float s33=String(s3).toFloat();
    float s44=String(s4).toFloat();
    char s11_char = s11.charAt(0); 
    switch(s11_char)
    {
      case 'f':
        {
          serial4.println("f");
          Robot.GetPosition(s22,s33,s44);
        }
        break;
      case 'h'://home 
        {
          Robot.Calib_home();
        }
        break;
      case 'i':
        {
          //di chuyển đến tọa độ
          Kin.inverse(s22,s33,s44);
          Robot.GetPosition(Kin.theta_1, Kin.theta_2, Kin.theta_3);
        }
        break;
      case 'c': 
        { //path planning circle
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");
          char *s7= strtok(NULL," ");
          
          float s55=String(s5).toFloat();
          float s66=String(s6).toFloat();
          float s77=String(s7).toFloat();      
          
          POINT_A[0]=s22;
          POINT_A[1]=s33;
          POINT_A[2]=s44;

          Robot.mVelMax = s55;
          Robot.mT_Total = s66;
          Robot.mRadius = s77;

          Robot.mPosMax = 2*pi*Robot.mRadius;
          

          if(Robot.mT_Total > (2*Robot.mPosMax)/Robot.mVelMax)
          {
            Robot.mT_Total = (2*Robot.mPosMax)/Robot.mVelMax;
          }
          if(Robot.mT_Total <= (Robot.mPosMax/Robot.mVelMax))
          {
            Robot.mT_Total = (3*Robot.mPosMax)/(2*Robot.mVelMax);
          }
          Robot.mT_a = Robot.mT_Total - Robot.mPosMax/Robot.mVelMax;
          Robot.mAccelMax = 2*Robot.mVelMax/Robot.mT_a;
          Robot.mTime = 0;
          Robot.cPos = 0;
          tProcess = CIRCLE;
        }
        break;
      case 's'://motion s curve
        {
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");     
          float s55=String(s5).toFloat(); //VEL
          float s66=String(s6).toFloat(); //TOTAL TIME
          float theta_x=stepperX.currentPosition()*0.0140625;
          float theta_y=stepperY.currentPosition()*0.0140625;
          float theta_z=stepperZ.currentPosition()*0.0140625;
          Kin.forward(theta_x,theta_y,theta_z,&Kin.x, &Kin.y, &Kin.z);      
          POINT_A[0]=round(Kin.x);
          POINT_A[1]=round(Kin.y);
          POINT_A[2]=round(Kin.z);
          POINT_B[0]=s22;
          POINT_B[1]=s33;
          POINT_B[2]=s44;
          Robot.mVelMax = s55;  // vmax
          Robot.mT_Total = s66; //Total travelling time
          Robot.mPosMax = sqrt(pow(POINT_B[0] - POINT_A[0],2)+pow(POINT_B[1] - POINT_A[1],2)+pow(POINT_B[2] - POINT_A[2],2));
          Robot.mKx = (POINT_B[0]-POINT_A[0])/Robot.mPosMax;
          Robot.mKy = (POINT_B[1]-POINT_A[1])/Robot.mPosMax;
          Robot.mKz = (POINT_B[2]-POINT_A[2])/Robot.mPosMax;

          if(Robot.mT_Total > (2*Robot.mPosMax)/Robot.mVelMax)
          {
            Robot.mT_Total = (2*Robot.mPosMax)/Robot.mVelMax;
          }
          if(Robot.mT_Total <= (Robot.mPosMax/Robot.mVelMax))
          {
            Robot.mT_Total = (3*Robot.mPosMax)/(2*Robot.mVelMax);
          }
          Robot.mT_a = Robot.mT_Total - Robot.mPosMax/Robot.mVelMax;
          Robot.mAccelMax = 2*Robot.mVelMax/Robot.mT_a;
          Robot.mTime = 0;
          Robot.cPos = 0;
          if(Robot.mPosMax == 0)
            {tProcess = NONE;}
          else 
            // startTime = micros();
            tProcess = SCURVE;
        }
        break;
      case 't': //save teaching
        {
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");
          char *s7= strtok(NULL," ");
          
          float s55=String(s5).toFloat();
          float s66=String(s6).toFloat();
          float s77=String(s7).toFloat();  
          Robot.SaveTeachingPos(s22,s33,s44,s55,s66,s77);
        }
        break;
      case 'm'://save moving pos
        {
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");
          char *s7= strtok(NULL," ");

          float s55=String(s5).toFloat();
          float s66=String(s6).toFloat();
          float s77=String(s7).toFloat();  
          Robot.SaveMovingPos(s22, s33, s44, s55, s66, s77);
        }
        break;
      case 'o': // clear teaching
        {
          Robot.ClearTeachingPos(s22);
        }
        break;
      case 'e': //clear moving
        {
          Robot.ClearMovingPos(s22);
        }
        break;
      case 'a': //clear all
        {
          Robot.ClrAllPos();
        }
        break;
      case 'g': //Go + the number of moving points
        {

          m_point = s22; //the number of moving points
          Robot.running_point = 1;
          curr_mp = 1;
          tProcess = CAL_MOVING;
        }
        break;
      case 'p': //pump
        {
          String s22 = String(s2);
          if (s22 == "o")
          {
            digitalWrite(pump, HIGH);
          }
          else if( s22 == "f")
          {
            digitalWrite(pump, LOW);
          }
        }
        break;
      case 'v': //valve
        {
          String s22 = String(s2);
          if (s22 == "o")
          {
            digitalWrite(valve, HIGH);
          }
          else if( s22 == "f")
          {
            digitalWrite(valve, LOW);
          }
        }
        break; 
      case 'd'://detect box
        {
          char *s5= strtok(NULL," ");
          float s55=String(s5).toFloat();
          char *s6= strtok(NULL," ");
          float s66=String(s6).toFloat();
          Robot.SaveObjPos(s22,s33,s44,s55,s66);
        }
        break;
      case 'w':
        {
          displayVector(s22);
        }
        break;
    }  
  }
}
 
void HAL_TIM1_CallBack()
{
  switch(tProcess)
  {
    case NONE:
      break;
    case CIRCLE:
      Robot.EofGetCircle();
      break;
    case SCURVE:
      Robot.EofGetPos_sCurve();
      break;
    case WARNING:
      // Robot.OutOfRange();
      break;   
    case CAL_MOVING:
      {
        float theta_x=stepperX.currentPosition()*0.0140625;
        float theta_y=stepperY.currentPosition()*0.0140625;
        float theta_z=stepperZ.currentPosition()*0.0140625;
        Kin.forward(theta_x,theta_y,theta_z,&Kin.x, &Kin.y, &Kin.z);      
        POINT_A[0] = round(Kin.x);
        POINT_A[1] = round(Kin.y);
        POINT_A[2] = round(Kin.z);
        if(Robot.running_point == 2) //go to picking point
        {
          //Go to picking point

          POINT_B[0] = ObjPoints[0][1];
          POINT_B[1] = ObjPoints[0][2];
          POINT_B[2] = ObjPoints[0][3];
          Robot.mVelMax =  350;  // vmax
          Robot.mT_Total = ObjPoints[0][4]; //Total travelling time    
        }
        else if(Robot.running_point == 4) //go to placing point
        {
          int index  = ObjPoints[0][0];
          POINT_B[0] = TeachingPoints[index][1];
          POINT_B[1] = TeachingPoints[index][2];
          POINT_B[2] = TeachingPoints[index][3];
          Robot.mVelMax = TeachingPoints[index][4];  // vmax
          Robot.mT_Total = TeachingPoints[index][5]; //Total travelling time   
        }
        else
        {
          //Normal moving point
          POINT_B[0] = MovingPoints[Robot.running_point-1][0];
          POINT_B[1] = MovingPoints[Robot.running_point-1][1];
          POINT_B[2] = MovingPoints[Robot.running_point-1][2];
          Robot.mVelMax =  MovingPoints[Robot.running_point-1][3];  // vmax
          Robot.mT_Total = MovingPoints[Robot.running_point-1][4]; //Total travelling time   
          curr_mp++; 
        }    
        
        if(POINT_B[0] == 0 && POINT_B[1] == 0 && POINT_B[2] == 0)
        {
          tProcess = NONE;
        }

        Robot.mPosMax = sqrt(pow(POINT_B[0] - POINT_A[0],2)+pow(POINT_B[1] - POINT_A[1],2)+pow(POINT_B[2] - POINT_A[2],2));
        Robot.mKx = (POINT_B[0]-POINT_A[0])/Robot.mPosMax;
        Robot.mKy = (POINT_B[1]-POINT_A[1])/Robot.mPosMax;
        Robot.mKz = (POINT_B[2]-POINT_A[2])/Robot.mPosMax;

        if(Robot.mT_Total > (2*Robot.mPosMax)/Robot.mVelMax)
        {
          Robot.mT_Total = (2*Robot.mPosMax)/Robot.mVelMax;
        }
        if(Robot.mT_Total <= (Robot.mPosMax/Robot.mVelMax))
        {
          Robot.mT_Total = (3*Robot.mPosMax)/(2*Robot.mVelMax);
        }
        Robot.mT_a = Robot.mT_Total - Robot.mPosMax/Robot.mVelMax;
        Robot.mAccelMax = 2*Robot.mVelMax/Robot.mT_a;
        Robot.mTime = 0;
        Robot.cPos = 0;

        if(curr_mp > m_point)
        {
          digitalWrite(valve, LOW);
          serial4.print("p d"); //PnP done!
          tProcess = NONE;
        }
        Robot.running_point++; 
        tProcess = RUN_AUTO;
      }
      break;
    case RUN_AUTO:
      {
        Robot.MovingProcess();
      }
      break;
  }

}
void data(){
    float alpha1=stepperX.currentPosition()*0.0140625;
    float alpha2=stepperY.currentPosition()*0.0140625;
    float alpha3=stepperZ.currentPosition()*0.0140625;
    Kin.forward(alpha1,alpha2,alpha3,&Kin.x,&Kin.y,&Kin.z);
    serial4.print("s");
    serial4.print(" ");
    serial4.print("k");
    serial4.print(" ");
    serial4.print(round(Kin.x));
    serial4.print(" ");
    serial4.print(round(Kin.y));
    serial4.print(" ");
    serial4.println(round(Kin.z));
}
void displayVector(float type_array)
{
  serial4.print("w");
  if(type_array == 0)
  {
    for(auto i : TeachingPoints)
    {
      for(auto j:i )
      {
        serial4.print(i[j]);
        serial4.print(" ");
      }
      serial4.println();
    }
  }
  else if(type_array == 1)
  {
        for(auto i : MovingPoints)
    {
      for(auto j:i )
      {
        serial4.print(i[j]);
        serial4.print(" ");
      }
      serial4.println();
    }
  }
}

