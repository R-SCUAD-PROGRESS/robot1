/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>

#include "BallFollower.h"


#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace std;
using namespace cv;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();
	int NoBallCount;
	static const int NoBallMaxCount = 15;
	int height,width,step,x,y;  //parameters of the image we are working on
      int t1min=0,t1max=0,t2min=0,t2max=0,t3min=0,t3max=0; // other variables used
/////////////////////////////punya ballfollower///////////////////
int m_NoBallMaxCount;
		int m_NoBallCount;
		int m_KickBallMaxCount;
		int m_KickBallCount;

		double m_MaxFBStep;
		double m_MaxRLStep;
		double m_MaxDirAngle;

		double m_KickTopAngle;
		double m_KickRightAngle;
		double m_KickLeftAngle;

		double m_FollowMaxFBStep;
        double m_FollowMinFBStep;
		double m_FollowMaxRLTurn;
        double m_FitFBStep;
		double m_FitMaxRLTurn;
		double m_UnitFBStep;
		double m_UnitRLTurn;
		
		double m_GoalFBStep;
		double m_GoalRLTurn;
		double m_FBStep;
		double m_RLTurn;
bool DEBUG_PRINT;
		int KickBall;
///////////////////////////////////////////////////////////////////
	

    minIni* ini = new minIni(INI_FILE_PATH);
   /* Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

    ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
    red_finder->LoadINISettings(ini, "RED");
    httpd::red_finder = red_finder;

    ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
    yellow_finder->LoadINISettings(ini, "YELLOW");
    httpd::yellow_finder = yellow_finder;

    ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    blue_finder->LoadINISettings(ini, "BLUE");
    httpd::blue_finder = blue_finder;
*/
    httpd::ini = ini;
	
	// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    CvCapture* capture = cvCaptureFromCAM( 0 );
    
    if( !capture )
    {
            fprintf( stderr, "ERROR: capture is NULL \n" );
            getchar();
            return -1;
    }
    // grab an image from the capture
    IplImage* frame = cvQueryFrame( capture );
    
    // Create a window in which the captured images will be presented
    cvNamedWindow( "Camera", (160,120));//CV_WINDOW_AUTOSIZE );
cvNamedWindow( "After Color Filtering", (160,120));
    //cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
    
 
   // Load threshold from the slider bars in these 2 parameters
    CvScalar hsv_min = cvScalar(t1min, t2min, t3min, 0);
    CvScalar hsv_max = cvScalar(t1max, t2max ,t3max, 0);
    
    // get the image data
      height    = frame->height;
      width     = frame->width;
      step      = frame->widthStep;

CvSize size = cvSize(640,480);
	
    IplImage*  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
    IplImage*  thresholded1   = cvCreateImage(size, IPL_DEPTH_8U, 1); // Component image threshold
    IplImage*  thresholded2   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  thresholded3   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  filtered   = cvCreateImage(size, IPL_DEPTH_8U, 1);  //smoothed image

	BallTracker tracker = BallTracker();
	BallFollower follower = BallFollower();

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    Action::GetInstance()->Start(9);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

    while(1)
    {
        StatusCheck::Check(cm730);

       // Point2D ball_pos, red_pos, yellow_pos, blue_pos;

        //LinuxCamera::GetInstance()->CaptureFrame();
       // memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        if(StatusCheck::m_cur_mode == READY || StatusCheck::m_cur_mode == VISION)
        {
           /* ball_pos = ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            red_pos = red_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            yellow_pos = yellow_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            blue_pos = blue_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

            unsigned char r, g, b;
            for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
                r = 0; g = 0; b = 0;
                if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                    r = 255;
                    g = 128;
                    b = 0;
                }
                if(red_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 255;
                        g = 0;
                        b = 0;
                    }
                }
                if(yellow_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 255;
                        g = 255;
                        b = 0;
                    }
                }
                if(blue_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 0;
                        g = 0;
                        b = 255;
                    }
                }

                if(r > 0 || g > 0 || b > 0)
                {
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
                }
            }*/
        }
        else if(StatusCheck::m_cur_mode == SOCCER||StatusCheck::m_cur_mode == MOTION)
        {
            /*tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

            for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
                if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
            }*/
        }

       // streamer->send_image(rgb_output);

        if(StatusCheck::m_is_started == 0)
            continue;

        switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
        case SOCCER:
            if(Action::GetInstance()->IsRunning() == 0)
            {
		//follower.arah =1; //arah gawang lawan di utara
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);


               /* follower.Process(tracker.ball_position);

                if(follower.KickBall != 0)
                {
                    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);*/

                    if(follower.KickBall == -1)
                    {
                        Action::GetInstance()->Start(12);   // RIGHT KICK
                        fprintf(stderr, "RightKick! \n");
                    }
                    else if(follower.KickBall == 1)
                    {
                        Action::GetInstance()->Start(13);   // LEFT KICK
                        fprintf(stderr, "LeftKick! \n");
                    }
		    else if(follower.KickBall == -2)
                    {
                        Action::GetInstance()->Start(111);   // SIDE RIGHT KICK
                        fprintf(stderr, "SideRighttKick! \n");
                    }
		    else if(follower.KickBall == 2)
                    {
                        Action::GetInstance()->Start(110);   // SIDE LEFT KICK
                        fprintf(stderr, "SideLeftKick! \n");
                    }
		    else if(follower.KickBall == -3)
                    {
                        Action::GetInstance()->Start(106);   // PUTAR KANAN DALAM
                        fprintf(stderr, "PutarKananDalam! \n");
                    }
		    else if(follower.KickBall == 3)
                    {
                        Action::GetInstance()->Start(105);   // PUTAR KIRI DALAM
                        fprintf(stderr, "PutarKiriDalam! \n");
                    }
		    else if(follower.KickBall == -4)
                    {
                        Action::GetInstance()->Start(114);   // GESER KANAN
                        fprintf(stderr, "PutarKananDalam! \n");
                    }
		    else if(follower.KickBall == 4)
                    {
                        Action::GetInstance()->Start(113);   // GESER KIRI
                        fprintf(stderr, "PutarKiriDalam! \n");
                    }
                //}
            }
            break;
        case MOTION:
	    if(Action::GetInstance()->IsRunning() == 0)
            {
		/*follower.arah=2;//arah gawang lawan di selatan
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                follower.Process(tracker.ball_position);

                if(follower.KickBall != 0)
                {
                    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                    if(follower.KickBall == -1)
                    {
                        Action::GetInstance()->Start(12);   // RIGHT KICK
                        fprintf(stderr, "RightKick! \n");
                    }
                    else if(follower.KickBall == 1)
                    {
                        Action::GetInstance()->Start(13);   // LEFT KICK
                        fprintf(stderr, "LeftKick! \n");
                    }
		    else if(follower.KickBall == -2)
                    {
                        Action::GetInstance()->Start(111);   // SIDE RIGHT KICK
                        fprintf(stderr, "SideRighttKick! \n");
                    }
		    else if(follower.KickBall == 2)
                    {
                        Action::GetInstance()->Start(110);   // SIDE LEFT KICK
                        fprintf(stderr, "SideLeftKick! \n");
                    }
		    else if(follower.KickBall == -3)
                    {
                        Action::GetInstance()->Start(106);   // PUTAR KANAN DALAM
                        fprintf(stderr, "PutarKananDalam! \n");
                    }
		    else if(follower.KickBall == 3)
                    {
                        Action::GetInstance()->Start(105);   // PUTAR KIRI DALAM
                        fprintf(stderr, "PutarKiriDalam! \n");
                    }
		    else if(follower.KickBall == -4)
                    {
                        Action::GetInstance()->Start(114);   // GESER KANAN
                        fprintf(stderr, "PutarKananDalam! \n");
                    }
		    else if(follower.KickBall == 4)
                    {
                        Action::GetInstance()->Start(113);   // GESER KIRI
                        fprintf(stderr, "PutarKiriDalam! \n");
                    }
                }*/
            }
            //if(LinuxActionScript::m_is_running == 0)
              //  LinuxActionScript::ScriptStart(SCRIPT_FILE_PATH);
            break;
        case VISION:
           /* int detected_color = 0;
            detected_color |= (red_pos.X == -1)? 0 : VisionMode::RED;
            detected_color |= (yellow_pos.X == -1)? 0 : VisionMode::YELLOW;
            detected_color |= (blue_pos.X == -1)? 0 : VisionMode::BLUE;

            if(Action::GetInstance()->IsRunning() == 0)
                VisionMode::Play(detected_color);*/
            break;
        }
       hsv_min = cvScalar(54, 0, 162);//(16, 0, 232, 0);		// orange (10, 199, 158, 0)
       hsv_max = cvScalar(256,13,256);//(71, 23, 256, 0);		// orange (26, 256, 256, 0)
	
	//hsv_min = cvScalar(7, 232, 231, 0);
       //hsv_max = cvScalar(40, 256 ,256, 0);

	frame = cvQueryFrame( capture );
        
        if( !frame )
        {
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }

cvCvtColor(frame,hsv_frame,CV_BGR2HSV);
        
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
        
        // the below lines of code is for visual purpose only remove after calibration 
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        cvSplit( hsv_frame,thresholded1, thresholded2, thresholded3, NULL );

// Filter out colors which are out of range.
        cvInRangeS(thresholded1,cvScalar(10,0,0,0) ,cvScalar(26,0,0,0) ,thresholded1);
        cvInRangeS(thresholded2,cvScalar(199,0,0,0) ,cvScalar(256,0,0,0) ,thresholded2);
        cvInRangeS(thresholded3,cvScalar(158,0,0,0) ,cvScalar(256,0,0,0) ,thresholded3);
        
        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------
    
    
        
        
        // hough detector works better with some smoothing of the image
	cvDilate(thresholded,thresholded,0,4.5);
	cvErode(thresholded,thresholded,0,2);	

        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 15, 15 );
	//cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
	
	//cvBlur(thresholded,thresholded, CV_GAUSSIAN, 3, 3);
	//cvCanny(thresholded, thresholded,250, 250*5,5);
	//GaussianBlur(gray, thresholded,Size(9, 9),2,2);
	
	
	//cvDilate(thresholded,thresholded,0,5);       
	//cvCanny(thresholded, thresholded,100, 110,3);
            
	CvMemStorage* storage = cvCreateMemStorage(0);

        //hough transform to detect circle
	
        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
                                        thresholded->height/4, 100, 50, 20, 50);  ///4, 100, 50, 10, 70)

	for (int i = 0; i < circles->total; i++)
        {   //get the parameters of circles detected
            float* p = (float*)cvGetSeqElem( circles, i );
            printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
//if((p[0]>300&&p[0]<400)&&(p[1]>150&&p[1]<300)&&(p[2]>80))
//{
          
            // draw a circle with the centre and the radius obtained from the hough transform
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot centre
                                    2, CV_RGB(255,255,255),-1, 8, 0 );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot circle
                                    cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0 );
x = (double)p[0];
y = (double)p[1];
Action::GetInstance()->m_Joint.SetEnableBody(false);

		//Head::GetInstance()->MoveByAngle(x,y);
		Point2D     ball_position;
		Point2D pos;
		pos.X=x;
		pos.Y=y;

		if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		int NoBallCount;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->MoveTracking();
			NoBallCount++;
		}
		else
			Head::GetInstance()->InitTracking();
	}
	else
	{
		NoBallCount = 0;
		/*Point2D center = Point2D(240, 320);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= ((181 / 90.0)*0.088); // pixel per angle
		offset.Y *= ((140 / 90.0)*0.088); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTracking(ball_position);*/
		
					Point2D center = Point2D (160, 120);
					Point2D offset = pos - center;
					offset *= -1; // Inverse X-axis, Y-axis
					offset.X *= ((160 / 120)*0.09); // pixel per angle
					offset.Y *= ((160 / 120)*0.09); // pixel per angle
					//offset.X *= ((240 / 150)*0.3); // pixel per angle
					//offset.Y *= ((320 / 150)*0.3); // pixel per angle
					//offset.X *= ((181 / 90.0)*0.078); // pixel per angle
					//offset.Y *= ((140 / 90.0)*0.088); // pixel per angle
					ball_position = offset;
					Head::GetInstance()->MoveTracking(ball_position);


m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
	m_KickBallMaxCount = 5;
	m_KickBallCount = 0;

	m_KickTopAngle = -5.0;
	m_KickRightAngle = -30.0;
	m_KickLeftAngle = 30.0;

	m_FollowMaxFBStep = 12.0;
    m_FollowMinFBStep = 5.0;
	m_FollowMaxRLTurn = 35.0;
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 0.3;
	m_UnitRLTurn = 1.0;
int d=-30;int t=0;
//////////////////////////////////////awal ballfollower//////////////////////////////////////
/*if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

    if(ball_position.X == -1.0 || ball_position.Y == -1.0)
    {
		KickBall = 0;

		if(m_NoBallCount > m_NoBallMaxCount)
		{
			int lagi=1;
			// can not find a ball
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			//Head::GetInstance()->MoveToHome();

			//if((t>=-70)&&(t<=70)){t++;}
		
			//if(t==70){t=-70; d+=10;}
			//if(d==10){d=-25;lagi++;}
			if((d>=-30)&&(d<=10)){d++;}
		
			if(d==10||d>10){d=-30; t+=10;}
			if(t==70||t>70){t=-70;lagi++;}
			
			Head::GetInstance()->MoveByAngle(t,d);
			
			if(lagi==2){
				for(int i=0;i<5;i++)
				{
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
				Walking::GetInstance()->Start();
				lagi++;
				}
			}
			if(lagi==3){
				for(int i=0;i<5;i++)
				{
				Walking::GetInstance()->A_MOVE_AMPLITUDE = 8;
				Walking::GetInstance()->Start();
				lagi=1;
				}
			}

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL]");
		}
		else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}
    }
    else
    {
	
		m_NoBallCount = 0;		

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		int kompas = MotionStatus::kompas;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(tilt-5 <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{
				if(ball_position.Y < m_KickTopAngle+10)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;						
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");
					   // if((kompas>=800&&kompas<=900)||(kompas>=0&&kompas<=80))//utara
					  //{
						if(pan > -11&&pan<15)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						if(pan>=-20 && pan<-11)
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}
					  //}
					  
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f C:%3d)]", pan, ball_position.Y, m_KickTopAngle,kompas);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
		}


	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
	{
		if(Walking::GetInstance()->IsRunning() == true)
			Walking::GetInstance()->Stop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			Walking::GetInstance()->Start();			
		}
		else
		{
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		}
	}	
}*/
//////////////////////////////////////akhir ballfollower/////////////////////////////////////

	}
		/*if((x>235&&x<290)&&(y>270&&y<300))
		{
		Walking::GetInstance()->Stop();
		usleep(10000);
int a=0;
a++;
if(a==1){
    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
    //Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
	Action::GetInstance()->Start(13);   // LEFT KICK 13
	while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//Action::GetInstance()->Brake(); 
x=0;
y=0;
a=2;
if(a==2){a=3;}
if(a==3){a=0;}}

                        fprintf(stderr, "LeftKick! \n");
		}
		//else if((x>270&&x<320)&&(y>45&&y<120))
		else if((x>320&&x<360)&&(y>270&&y<300))
		{
		Walking::GetInstance()->Stop();
		usleep(10000);
int a=0;
a++;
if(a==1){
    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
    //Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
		Action::GetInstance()->Start(12);   // right KICK 13		
		while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//usleep(10000);
//Action::GetInstance()->Stop(); 
//usleep(10000);
                        fprintf(stderr, "RightKick! \n");
//Action::GetInstance()->Brake();
x=0;
y=0;
a=2;
if(a==2){a=3;}
if(a==3){a=0;}}


		}*/
		/*else if(((x>300)&&(x<340))&&((y>0)&&(y<120)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=8;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=2;
		Walking::GetInstance()->Start();
                        printf("Tengah Jauh!");
		}
		else if(((x>300)&&(x<340))&&((y>120)&&(y<320)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=8;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=2;
		//Head::GetInstance()->MoveByAngle(0,-20);
		Walking::GetInstance()->Start();
                        printf("Tengah dekat!");
		}
		else if(((x>300)&&(x<340))&&((y>320)&&(y<480)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=5;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=2;
		//Head::GetInstance()->MoveByAngle(0,-20);
		Walking::GetInstance()->Start();
                        printf("Tengah dekat Sekali!");
		}
		else if(((x>150)&&(x<300))&&((y>0)&&(y<480)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=8;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=8;
		Walking::GetInstance()->Start();
                        printf("Kiri deket! ");
		}
		else if(((x>0)&&(x<=150))&&((y>0)&&(y<480)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=2;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=12;
		Walking::GetInstance()->Start();
                        printf("Kiri Jauh!");
		}
		else if(((x>340)&&(x<=490))&&((y>0)&&(y<480)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=8;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=-5;
		Walking::GetInstance()->Start();
                        printf("Kanan deket!");
		}
		else if(((x>490)&&(x<=640))&&((y>0)&&(y<480)))
		{
		Walking::GetInstance()->X_MOVE_AMPLITUDE=2;
		Walking::GetInstance()->A_MOVE_AMPLITUDE=-10;
		Walking::GetInstance()->Start();
                        printf("Kanan Jauh!");
		}*/
}
	
        
	cvShowImage( "Camera", frame ); // Original stream with detected ball overlay
        // cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
         cvShowImage( "After Color Filtering", thresholded ); // The stream after color filtering
	
      
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cvWaitKey(10) & 255) == 27 ) break;
    }
	 
     
     
     // Release the capture device housekeeping
     cvReleaseCapture( &capture );
     cvDestroyWindow( "mywindow" );

    return 0;
		
//}


	
   // }

    return 0;
}
