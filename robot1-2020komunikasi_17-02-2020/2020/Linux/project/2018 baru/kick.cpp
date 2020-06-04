#include "kick.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "MX28.h"
#include "MotionManager.h"
#include "CM730.h"
#include "MotionStatus.h"
#include "LinuxDARwIn.h"
#include <math.h>
#include "Vector.h"
#include "Matrix.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"

using namespace Robot;
using namespace std;
void kanan();
void kanan1();
void kiri();
void kiri1();

LinuxCM730 slinux_scm730("/dev/ttyUSB0");
CM730 scm730(&slinux_scm730);
//                           1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
int InitPose[21] = {2047, 1480, 2610, 1747, 2300, 2147, 1944, 2017, 2047, 2047, 2047, 2013, 2080, 2047, 2047, 2063, 2030, 2047, 2047, 2047, 1762};

int n = 0;
int param[JointData::NUMBER_OF_JOINTS * 5];
int wGoalPosition, wStartPosition, wDistance;
int count =0;
int kecepatan=0;
int araha=0;
int x1=0;

int tdd;
int t1;
int y1;
int a1;
int y2;
int y3;
int y5;
int y7;
//double sudut; 
double theta;
double keseimbangan;
	
int kick::_getch()
{
	int ch;
	ch = getchar();
	return ch;
}

kick::kick(double sudut, int eg)
{
	minIni* ini = new minIni(INI_FILE_PATH);

    //////////////////// Framework Initialize ////////////////////////////	
    if(MotionManager::GetInstance()->Initialize(&scm730) == false)
    {
        printf("Initializing Motion Manager failed!\n");
        //return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    /////////////////////////////////////////////////////////////////////
	
	void DrawIntro(Robot::CM730 *scm730);
{
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
		
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 3;
        if( wDistance < 8 )
            wDistance = 8;
		
		
		param[n++] = id;
		fprintf(stderr, "ID:%d  \n \r", id);
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
		printf ("wDistance = %d \n \r", wDistance);		
		
		if(count == 1 ) 
		{
            kanan();  /* Control of the program moves to kanan; */
        } 	
		
		if(count == 2 ) 
		{
            kiri();  /* Control of the program moves to kiri; */
        } 
		
    }

    scm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
	_getch();
	
	
		////////////////////////////////////////////////////////////////////////////////////////////	
			scm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L,1480,0);
			scm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L,2610,0);
		////////////////////// ID 3 dan 4 ////////////////////////	
			scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,1747,0);
			scm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,2300,0);
		////////////////////// ID 5 dan 6 ////////////////////////		
			scm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L,2147,0);
			scm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L,1944,0);
		////////////////// ID 7 dan 8 /////////////////////////////		
			scm730.WriteWord(JointData::ID_R_HIP_YAW, MX28::P_GOAL_POSITION_L,2017,0);
			scm730.WriteWord(JointData::ID_L_HIP_YAW, MX28::P_GOAL_POSITION_L,2047,0);
		//////////////////// ID 9 dan 10 ////////////////////////////
			scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);	
		////////////// ID 11 dan 12 /////////////////////////////////
			scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,2013,0);
			scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,2080,0);
		/////////////////// ID 13 dan 14 ////////////////////////////
			scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
		/////////////////// ID 15 dan 16 /////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2063,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2030,0);
		/////////////////// ID 17 dan 18 //////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
		//////////////////// ID 19 dan 20 /////////////////////////////
			scm730.WriteWord(JointData::ID_HEAD_PAN, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_HEAD_TILT, MX28::P_GOAL_POSITION_L,1762,0);			
	

cout<< "Masukkan araha Tedangan :";
cin>>araha;
if (araha==1);
else if (araha==2);
else { cout<< "silahkan masukkan araha :";}

cout<< "Masukkan Sudut :";
cin>>sudut;

cout<< "masukan kecepatan : ";
cin>>kecepatan;

theta = (sudut*5.7) + (29/sudut *29);

if(araha==1)
{ y1=2017+theta; } //7
else
{ y1=2047-theta; } //8

if(araha==1)
{ a1=2047+theta; } //9
else
{ a1=2047-theta; } //10

if(araha==1)
{ y2=2013+theta; } //11
else
{ y2=2080-theta; } //12

if(araha==1)
{ y3=2047+theta; } //13
else 
{ y3=2047-theta; } //14 

if(araha==1)
{ y7=2063+theta; } //15
else 
{ y7=2030-theta; } //16

if(araha==1)
{ y5=2047+theta; } //17
else
{ y5=2047-theta; } //18

keseimbangan = sudut*5.7; 
if(araha==1)
{ tdd=2300-keseimbangan; } //4
else
{ tdd=1747+keseimbangan; } //3
cout<<endl;

/////////////////// tendangan kanan //////////////////
if (araha==1)
{
//if (sudut>9&& sudut<30)
	
cout<<"araha tendangan ke kanan";
cout<<" Sudut  : "<<sudut<<endl;
        //////////////// Fase 1 ///////////////////// 
		printf ("Fase 1 \n");
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5+70,0); //17
		usleep(2000000);
		scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y+8,0); //18
		usleep(2000000);
		/////////////// Fase 2 /////////////////////
		printf ("Fase 2 \n");
		scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1-50,0); // 9
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); // 17
		usleep(1000000);
		printf ("keseimbangan tangan \n");
		scm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,tdd,0);//4
		usleep(2000000);
		/////////////// Fase 3 /////////////////////
		printf ("Fase 3 \n");
		scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2,0); // 11
		scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,y3,0); // 13
		usleep(3000000);
		/////////////// Fase 4 /////////////////////
		count =1;
		kanan();	

}
//////////////////////// tendangan kiri /////////////////
else if (araha==2)
{
//if (sudut>9&& sudut<30)
cout<<"araha tendangan ke kiri";
cout<<" Sudut  : "<<sudut<<endl;

		//////////////// Fase 1 ///////////////////// 
		printf ("Fase 1 \n");
		scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5-60,0);//18
		usleep(2000000);
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); //17
		usleep(2000000);
		/////////////// Fase 2 /////////////////////
		printf ("Fase 2 \n");
		scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1+50,0); //10
		scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); // 18
		usleep(2000000);
		printf ("keseimbangan tangan \n");
		scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,tdd,0);// 3
		usleep(1000000);
		/////////////// Fase 3 /////////////////////
		printf ("Fase 3 \n");
		scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2,0); //12
		scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,y3,0); // 14
		usleep(3500000);
		/////////////// Fase 4 /////////////////////
		count =2;
		kiri();	
	
		} 
	}
}

///////////////////////////////////////////////// tendangan kanan ///////////////////////////////////////
void kick::kanan()
{
	int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;
	
	
	for(int id=JointData::ID_R_HIP_ROLL; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;
		
		
        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance+kecepatan);
        param[n++] = CM730::GetHighByte(wDistance+kecepatan);
		printf ("kecepatan = %d \n \r", kecepatan);
		if(count == 3 ) 
		{
            kanan1();  /* Control of the program moves to kanan1; */
        } 	
    }
    scm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

if(araha==1)
{ y1=2017+theta; } //7
else
{ y1=2047-theta; } //8

if(araha==1)
{ a1=2047+theta; } //9
else
{ a1=2047-theta; } //10

if(araha==1)
{ y2=2013-theta; } //11
else
{ y2=2080-theta; } //12

if(araha==1)
{ y3=2047-theta; } //13
else 
{ y3=2047+theta; } //14 

if(araha==1)
{ y7=2063-theta; } //15
else 
{ y7=2030-theta; } //16

if(araha==1)
{ y5=2047+theta; } //17
else
{ y5=2047+theta; } //18
 
if(araha==1)
{ tdd=2300-keseimbangan; } //4
else
{ tdd=1747+keseimbangan; } //3
		/////////////// Fase 4 /////////////////////
		printf ("Fase 4 \n");
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); //17
		scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5-20,0); //18
		scm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,y7-40,0); //15
		scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1-50,0); // 9
		scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2-50,0); // 11
		scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,y3-50,0); // 13
	usleep(2500000);
	count =3;
	kanan1();			
}

///////////////////// tendangan kanan/////////////////
void kick::kanan1()
{
	int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;
	
	for(int id=JointData::ID_R_HIP_ROLL; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 3;
        if( wDistance < 8 )
            wDistance = 8;
		
        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
		printf ("wDistance = %d \n \r", wDistance);
    }

    scm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
				
if(araha==1)
{ y1=2017+theta; } //7
else
{ y1=2047-theta; } //8

if(araha==1)
{ a1=2047-theta; } //9
else
{ a1=2047-theta; } //10

if(araha==1)
{ y2=2013+theta; } //11
else
{ y2=2080-theta; } //12

if(araha==1)
{ y3=2047+theta; } //13
else 
{ y3=2047+theta; } //14 

if(araha==1)
{ y7=2063+theta; } //15
else 
{ y7=2030-theta; } //16

if(araha==1)
{ y5=2047+theta; } //17
else
{ y5=2047+theta; } //18
 
if(araha==1)
{ tdd=2300-keseimbangan; } //4
else
{ tdd=1747+keseimbangan; } //3
		/////////////// Fase 5 /////////////////////
		printf ("Fase 5 \n");
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); //17
		scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5-20,0); //18
		scm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,y7,0); //15
		scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1+50,0); // 9
		scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2,0); // 11
		scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,y3,0); // 13	
		usleep(1200000);
		/////////////// Fase 6 /////////////////////		
		printf ("Fase 6 \n");
		///////////////////////////////ID 3 dan ID 4 //////////////////////////// 
			scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,1747,0);
			scm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,2300,0);
			///////////////////////////////ID 9 dan ID 10 //////////////////////////// 
			scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);	
		////////////// ID 11 dan 12 /////////////////////////////////
			scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,2013,0);
			scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,2080,0);
		/////////////////// ID 13 dan 14 ////////////////////////////
			scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
		/////////////////// ID 15 dan 16 /////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2063,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2030,0);
		/////////////////// ID 17 dan 18 //////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
		//////////////////// ID 19 dan 20 /////////////////////////////
			scm730.WriteWord(JointData::ID_HEAD_PAN, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_HEAD_TILT, MX28::P_GOAL_POSITION_L,2263,0);
}

/////////////////////// tendangan kiri///////////////
void kick::kiri()
{
	int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;
	
	
	for(int id=JointData::ID_R_HIP_ROLL; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;
		
		
        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance+kecepatan);
        param[n++] = CM730::GetHighByte(wDistance+kecepatan);
		printf ("kecepatan = %d \n \r", kecepatan);
		if(count == 4 ) 
		{
            kiri1();  /* Control of the program moves to jump; */
        } 	
    }

    scm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);

if(araha==1)
{ y1=2017+theta; } //7
else
{ y1=2047-theta; } //8

if(araha==1)
{ a1=2047+theta; } //9
else
{ a1=2047-theta; } //10

if(araha==1)
{ y2=2013-theta; } //11
else
{ y2=2080+theta; } //12

if(araha==1)
{ y3=2047-theta; } //13
else 
{ y3=2047-theta; } //14 

if(araha==1)
{ y7=2063+theta; } //15
else 
{ y7=2030+theta; } //16

if(araha==1)
{ y5=2047+theta; } //17
else
{ y5=2047-theta; } //18

if(araha==1)
{ tdd=2300-keseimbangan; } //4
else
{ tdd=1747+keseimbangan; } //3
		/////////////// Fase 4 /////////////////////
		printf ("Fase 4 \n");
		scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1+50,0); //10
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); // 17
		scm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,y7+40,0); //16
		scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,tdd,0);// 3
		scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2+100,0); //12
		scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,y3+100,0); // 14

	usleep(2000000);
	count =4;
	kiri1();			
}

//////////////////////// tendangan kiri/////////////
void kick::kiri1()
{
	int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;
	
	for(int id=JointData::ID_R_HIP_ROLL; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = InitPose[id] + MotionManager::GetInstance()->m_Offset[id];
 
 if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;
		
        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
		printf ("wDistance = %d \n \r", wDistance);
    }

    scm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);
		
if(araha==1)
{ y1=2017+theta; } //7
else
{ y1=2047-theta; } //8

if(araha==1)
{ a1=2047-theta; } //9
else
{ a1=2047+theta; } //10

if(araha==1)
{ y2=2013+theta; } //11
else
{ y2=2080+theta; } //12

if(araha==1)
{ y3=2047+theta; } //13
else 
{ y3=2047-theta; } //14 

if(araha==1)
{ y7=2063+theta; } //15
else 
{ y7=2030+theta; } //16

if(araha==1)
{ y5=2047+theta; } //17
else
{ y5=2047-theta; } //18
 
if(araha==1)
{ tdd=2300-keseimbangan; } //4
else
{ tdd=1747+keseimbangan; } //3
		/////////////// Fase 5 /////////////////////
		printf ("Fase 5 \n");
		scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,a1+50,0); //10
		scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,y5,0); // 17
		scm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,y7,0); //16
		scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,tdd,0);// 3
		scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,y2,0); //12
		scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,y3,0); // 14	
		usleep(2000000);
		/////////////// Fase 6 /////////////////////		
		printf ("Fase 6 \n");
			///////////////////////////////ID 3 dan ID 4 //////////////////////////// 
			scm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,1747,0);
			scm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L,2300,0);
			///////////////////////////////ID 9 dan ID 10 //////////////////////////// 
			scm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L,2047,0);	
		////////////// ID 11 dan 12 /////////////////////////////////
			scm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L,2013,0);
			scm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L,2080,0);
		/////////////////// ID 13 dan 14 ////////////////////////////
			scm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L,2047,0);
		/////////////////// ID 15 dan 16 /////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2083,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L,2030,0);
		/////////////////// ID 17 dan 18 //////////////////////////////
			scm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L,2047,0);
		//////////////////// ID 19 dan 20 /////////////////////////////
			scm730.WriteWord(JointData::ID_HEAD_PAN, MX28::P_GOAL_POSITION_L,2047,0);
			scm730.WriteWord(JointData::ID_HEAD_TILT, MX28::P_GOAL_POSITION_L,2263,0);

}
 



