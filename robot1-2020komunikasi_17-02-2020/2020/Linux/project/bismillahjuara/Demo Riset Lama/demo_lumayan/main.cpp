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
#include "Head.h"

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <ncurses.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <arpa/inet.h>
//#include "port.h"

#define BUFSIZE 2048
#define SERVICE_PORT 3838
#define POLL_TIMEOUT 20



#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

//int arah;
/*
int last=0;
struct sockaddr_in myaddr;	// our address 
	struct sockaddr_in remaddr;	// remote address 
	socklen_t addrlen = sizeof(remaddr);		// length of addresses 
	int recvlen;			// # bytes received 
	int fd;				// our socket 
	unsigned char buf[BUFSIZE];	// receive buffer 

int initreferee(){
// create a UDP socket 

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
	}

	// bind the socket to any valid IP address and a specific port 

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(SERVICE_PORT);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return 0;
	}


}
int referee ()
{

 

// Setup for polling
   struct pollfd ufds[1];
   ufds[0].fd = fd;
   ufds[0].events = POLLIN;            // For incoming packets
	
	
	
	// Congested WiFi: Try to grab several packets in one tick
   // to clear buffers and lower effective latency
   for (int i = 0; i < 5; i++) {
      int rv = poll(ufds, 1, POLL_TIMEOUT);  // Wait up to POLL_TIMEOUT ms
     
      // Check to see if we've received a packet
      if (rv > 0) {
         recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
         if (recvlen ==116) {
            //parseData((RoboCupGameControlData*)buffer);
			buf[recvlen] = 0;
			//printf("received message: \"%s\"\n", buf);
			//last=buf[9];
			switch (buf[9]){
			case 0:	{//printf("initial\n");
				last=0;}
			break;
			case 1: {//printf("ready\n");
				last=1;}
			break;
			case 2:{//printf("set\n");
				last=2;}
			break;
			case 3:{//printf("play\n");
				last=3;}
			break;
			case 4:{//printf("finish\n");
				last=4;}
			break;
			
         }
      }
	else{
	//printf("masih sama\n");
	//printf(" %d \n", last);
       }
	return last;
   }
	}
	
	
		//printf("wasit ok\n");
		//cukup=1;


return last;

   

}

*/
using namespace cv;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

int cx=0,cy=0,x,y;
int init_soccer=0;
Mat src; Mat src_gray;
 int thresh = 100;
 int th2 = 255, th1 =8;
 RNG rng(12345);
 
//========================= variabel lapangan ===================================//
 
 int h_min = 40, h_max = 85, s_min = 37, s_max = 255, v_min = 57, v_max = 255,conture = 100;
 
//========================= variabel bola ===================================//

int h_min2 = 189, h_max2 = 246, s_min2 = 175, s_max2 = 241, v_min2 = 215, v_max2 = 250;
int conture2 = 8,aconture2 = 153,th2_max = 255; 
Mat threshold_output;

//==================================//
      
//these will be changed using trackbars

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
vector<Vec3f> circles;

string intToString(int number){
 
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void createTrackbars(){

//============================= trackbar lapangan ============================//	   
   namedWindow("Trackbar Lapangan", CV_WINDOW_NORMAL);
   		createTrackbar("R1_MIN: ","Trackbar Lapangan",&h_min,255);
		createTrackbar("R1_MAX: ","Trackbar Lapangan",&h_max,255);
		createTrackbar("R2_MIN: ","Trackbar Lapangan",&s_min,255);
		createTrackbar("R2_MAX: ","Trackbar Lapangan",&s_max,255);
		createTrackbar("R3_MIN: ","Trackbar Lapangan",&v_min,255);
		createTrackbar("R3_MAX: ","Trackbar Lapangan",&v_max,255);
		createTrackbar("Conture: ","Trackbar Lapangan",&conture,255);

//============================= trackbar bola ============================//	
    namedWindow("Trackbar Bola", CV_WINDOW_NORMAL);
		createTrackbar("R1_MIN2: ","Trackbar Bola",&h_min2,255);
		createTrackbar("R1_MAX2: ","Trackbar Bola",&h_max2,255);
		createTrackbar("R2_MIN2: ","Trackbar Bola",&s_min2,255);
		createTrackbar("R2_MAX2: ","Trackbar Bola",&s_max2,255);
		createTrackbar("R3_MIN2: ","Trackbar Bola",&v_min2,255);
		createTrackbar("R3_MAX2: ","Trackbar Bola",&v_max2,255);
		createTrackbar("Conture2: ","Trackbar Bola",&conture2,255);
		createTrackbar("aConture2: ","Trackbar Bola",&aconture2,255);
		createTrackbar("th1: ","Trackbar Bola",&th1,255);
		createTrackbar("th2: ","Trackbar Bola",&th2,255);

//=============================================//
 
}


void sighandler(int sig)
{
    exit(0);
}

int main(int argc, char* argv[])//void)
{
//	initreferee(); //reffree
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
   
    httpd::ini = ini;
	//int cx=-5,cy=-5;
	// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
	//some boolean variables for different functionality within this
    bool recap = true;
    bool useMorphOps = true;
    //Matrix to store each frame of the webcam feed
		Mat imgOriginal;
        Mat filter;
        Mat filterr;
        Mat out;   
        Mat imgHSV;
        Mat imgThresholded;
        Mat grayimg;
        Mat res;
    //x and y values for the location of the object
    //create slider bars for HSV filtering
    createTrackbars();
    //video capture object to acquire webcam feed
    VideoCapture cap;
    //open capture object at location zero (default location for webcam)
    cap.open(0);
    //set height and width of capture frame
	cap.set(CV_CAP_PROP_FRAME_WIDTH,420);//420);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,360);//340);
    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop

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

        if(StatusCheck::m_cur_mode == READY || StatusCheck::m_cur_mode == VISION)
        {
           
        }
        else if(StatusCheck::m_cur_mode == SOCCER||StatusCheck::m_cur_mode == MOTION)
        {
           
        }

       
        if(StatusCheck::m_is_started == 0)
            continue;

        switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
        case SOCCER:
            if(Action::GetInstance()->IsRunning() == 0)
            {
                  //referee ();
			/*	while(referee()!=3){
				printf(" %d \n", referee());
				 Walking::GetInstance()->Stop();
				}
				/*else{*/
				follower.arah=1;
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);//}
		if(init_soccer =0)
		{
			Walking::GetInstance()->X_MOVE_AMPLITUDE =10;
			Walking::GetInstance()->Start();
		}
		init_soccer++;

                if(follower.KickBall != 0)
                {
                    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
                    Walking::GetInstance()->m_Joint.SetEnableBody(false);
                    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);

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
                }
            }
            break;
        case MOTION:
	    if(Action::GetInstance()->IsRunning() == 0)
            {

            }
            break;
        case VISION:
            break;
        }
		lagi:
		Action::GetInstance()->m_Joint.SetEnableBody(false);
		//MotionManager::GetInstance()->SetEnable(true);
		//Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
		//store image to matrix
//        Mat sample;
		cap.read(src);

//=============== lapangan =======================//

   cvtColor( src, src_gray, CV_BGR2HSV );
   inRange(src_gray,Scalar(h_min,s_min,v_min),Scalar(h_max, s_max, v_max),threshold_output);
   erode( threshold_output, threshold_output, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
   dilate( threshold_output, threshold_output, getStructuringElement(MORPH_ELLIPSE, Size(12, 12)) );
 
  //    imshow( "Lapangan", threshold_output );
 //===============================================//		
   Mat src_copy = src.clone();
 
   vector<vector<Point> > contours;
   vector<Vec4i> hierarchy;
   
   // mencari controur lapangan
   findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

   vector<vector<Point> >hull( contours.size() );
 
   
//=============== mencari luas lapangan =====================//
   for( int i = 0; i < contours.size(); i++ )
      {  convexHull( Mat(contours[i]), hull[i], false ); 
      }
//======================================================//

   Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC1 );
   for( int i = 0; i< contours.size(); i++ )
      {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	if (contourArea(contours[i]) > conture*40)
	{
        drawContours( src, contours, i, Scalar(0,0,0), 3, 8, vector<Vec4i>(), 0, Point() );
        drawContours(drawing, contours, i, Scalar(255, 255, 255), CV_FILLED); 	
    }
      }
      
	Mat mask, mask2, bl;
	bitwise_and(src,src,mask,drawing = drawing);
	//imshow("Batas",mask);
	inRange(mask,Scalar(h_min2,s_min2,v_min2),Scalar(h_max2, s_max2, v_max2),mask2);
	erode( mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );
	threshold(mask2,mask2,th1,th2,THRESH_BINARY+THRESH_OTSU);
	bitwise_and(src,src,bl,mask2 = mask2);
	//imshow("Bola",bl);

   vector<vector<Point> > contours2;
   vector<Vec4i> hierarchy2;
   // mencari controur bola
   findContours( mask2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
   vector<vector<Point> > contours_poly2( contours2.size() );
   vector<Rect> boundRect2( contours2.size() );
   vector<Point2f>center( contours2.size() );
   vector<float>radius( contours2.size() );
  
        
		
		if(useMorphOps)
		{

					m_NoBallMaxCount = 10;
					m_NoBallCount = m_NoBallMaxCount;
					m_KickBallMaxCount = 5;
					m_KickBallCount = 0;

					m_FollowMaxFBStep = 15.0;
					m_FollowMinFBStep = 10.0;
					m_FollowMaxRLTurn = 15.0;
					m_FitFBStep = 3.0;
					m_FitMaxRLTurn = 15;//35.0;
					m_UnitFBStep = 0.3;
					m_UnitRLTurn = 1.0;
					DEBUG_PRINT = true;
					int NoBallMaxCount = 15;
					int NoBallCount=0;
		//=============== koordinat bola =====================//

for( int i = 0; i < contours2.size(); i++ )
     { approxPolyDP( Mat(contours2[i]), contours_poly2[i], 3, true );
       boundRect2[i] = boundingRect( Mat(contours_poly2[i]) );
       minEnclosingCircle( (Mat)contours_poly2[i], center[i], radius[i] );
     }

//======================================================//

 //int cx=0,cy=0,x,y;	  
	  
for( int p = 0; p< contours2.size(); p++ )
{
//line( src, center[p], Point(center[p].x+40, center[p].y-50), Scalar(0,54,255), 2, CV_AA);
//line( src, Point(center[p].x+40, center[p].y-50),Point(center[p].x+90, center[p].y-50), Scalar(255,255,255), 2, CV_AA);
circle( src, center[p], 20, Scalar(255, 255, 255), 2, CV_AA  );

if (contourArea(contours2[p]) > conture2*20 && contourArea(contours2[p]) < aconture2*50)
       {
	cx = center[p].x;
	cy = center[p].y;
	x=cx;
	y=cy;
       }
 }     
	circle( src, Point(x,y), 9, Scalar(40, 80, 255), CV_FILLED, CV_AA  );
	line( src, Point(x,y), Point(x+40, y-50), Scalar(0,54,255), 2, CV_AA);
	line( src, Point(x+40, y-50),Point(x+90, y-50), Scalar(255,255,255), 2, CV_AA);
	putText(src, "Bola", Point(x+40, y-30), FONT_HERSHEY_COMPLEX_SMALL, 1,  Scalar(0,0,255,255));

//printf("[masuk]");		
                
					Point2D     ball_position;
				
												
						if(( x <= 0 && y <=0))
						{printf("[X=%d Y:%d\n\r]",x,y);
							ball_position.X = -1;
							ball_position.Y = -1;
							if(NoBallCount < NoBallMaxCount)
							{
								Head::GetInstance()->MoveTracking();
								NoBallCount++;
							}
							//else
							//Head::GetInstance()->InitTracking();
						}
						else
						{
							//printf("[tracker]");
		
							//Point2D     ball_position;
							Point2D pos;
							pos.X=x;//-25;
							pos.Y=y;//-25;
							Point2D center = Point2D (210, 180);
							Point2D offset = pos - center;

							offset *= -1; // Inverse X-axis, Y-axis
							offset.X *= ((210 / 180)*0.09); // pixel per angle
							offset.Y *= ((210 / 180)*0.09); // pixel per angle
							//offset.X *= ((240 / 150)*0.3); // pixel per angle

							//offset.Y *= ((320 / 150)*0.3); // pixel per angle
							//offset.X *= ((181 / 90.0)*0.078); // pixel per angle

							//offset.Y *= ((140 / 90.0)*0.088); // pixel per angle
							ball_position = offset;

							Head::GetInstance()->MoveTracking(ball_position);
	
		
						}
		
																
//////////////////////////////////////awal ballfollower//////////////////////////////////////
					if(DEBUG_PRINT == true)
						fprintf(stderr, "\r                                                                               \r");

						if((ball_position.X == -1.0 || ball_position.Y == -1.0))//||((px<=0)&&(px>hx_max))&&((py<=0)&&(py>hy_max))) ///-1.0
						{						
							//printf("[TIDAK LIAT]");
							int kompas = MotionStatus::kompas;
							printf("kompas=%f",kompas);
							//if(m_NoBallCount > m_NoBallMaxCount)
							//{
					int d;
					int t;
							int lagi=1;
							// can not find a ball
							m_GoalFBStep = 0;
							m_GoalRLTurn = 0;
							
							//if((d>=-30)&&(d<=10)){d++;}
		
							//if(d==10||d>10){d=-30; t+=10;}
							//if(t==70||t>70){t=-70;lagi++;}
							if((t>=-70)&&(t<=70)){t++;}
		
							if(t==70){t=-70; d+=10;}
							if(d==10){d=-25;lagi++;}
			
							Head::GetInstance()->MoveByAngle(t,d);
							//Action::GetInstance()->m_Joint.SetEnableHeadOnly(true);
							if(lagi==2)
							{
								for(int i=0;i<5;i++)
								{
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
								Walking::GetInstance()->Start();
								lagi++;
								}
							}
							if(lagi==3)
							{
								for(int i=0;i<5;i++)
								{
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 5;
								Walking::GetInstance()->Start();
								lagi=1;
								}
							}

					//		if(DEBUG_PRINT == true)
						//	fprintf(stderr, "[NO BALL]");
				//			}
			/*				else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}*/
						
						}
						else
						{
////////////////////////////////////////////////////////////////////////////////////////////////ketika ada bola(mencapai target/////////////
							m_KickTopAngle = -5.0;
							m_KickRightAngle = -30.0;
							m_KickLeftAngle = 30.0;
							m_NoBallCount = 0;		
							double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
							double pan_range = Head::GetInstance()->GetLeftLimitAngle();
							double pan_percent = pan / pan_range;

							int kompas = MotionStatus::kompas;

							double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
							double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
							double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
							double tilt_percent = (tilt - tilt_min) / tilt_range;
							//if(tilt_percent < 0)
								//tilt_percent = -tilt_percent;
						
								if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
								{
									printf("kompas=%f",kompas);
									if(tilt <= -20.0)//-20.0//(tilt_min + MX28::RATIO_VALUE2ANGLE))
									{//Walking::GetInstance()->Stop();
										if(ball_position.Y < m_KickTopAngle)
										{
											m_GoalFBStep = 0;
											m_GoalRLTurn = 0;
									usleep(100);
Walking::GetInstance()->Stop();
usleep(1000);											//if(m_KickBallCount >= m_KickBallMaxCount)
											//{// awal nendang
												m_FBStep = 0;
												m_RLTurn = 0;						
												if(DEBUG_PRINT == true)
													fprintf(stderr, "[KICK]");
												Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
												MotionManager::GetInstance()->SetEnable(true);

												if((pan > 15)&&(pan < 20)||pan > 0)// -11&&pan<15)
												{printf("PAN=%f",pan);
											

													Action::GetInstance()->Start(13);
													printf(" Left");
													//if(Action::GetInstance()->IsRunning() == 0)	
													while(Action::GetInstance()->IsRunning()) {usleep(0.00002*1000);Head::GetInstance()->MoveByAngle(0,30);}				// LEFT KICK 13
													//(Action::GetInstance()->IsRunning()) usleep(0.002*1000);
													
													

												}
												//while(noball);
												if((pan < -8)&&(pan > -16)||pan < 0)
												{
											
													Action::GetInstance()->Start(12);   // RIGHT KICK 12
													printf(" Right");
													//if(Action::GetInstance()->IsRunning() == 0)	
													while(Action::GetInstance()->IsRunning()){ usleep(0.00002*1000);Head::GetInstance()->MoveByAngle(0,30);}				// LEFT KICK 13
													//Action::GetInstance()->IsRunning()) usleep(0.002*1000);
													//while(Action::GetInstance()->IsRunning()) usleep(0.002*1000);
													
													
				
												}
												//while(useMorphOps);
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
								/////////////////////////////////////////////////////////////////////////////////////////////////////
								else
								{
									m_KickBallCount = 0;
									KickBall = 0;
									m_GoalFBStep = 0;
									m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
									if(DEBUG_PRINT == true)
									fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
								}

							
							//else Walking::GetInstance()->Stop();
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
									Walking::GetInstance()->A_MOVE_AIM_ON = true;
									m_FBStep = 0;
									m_RLTurn = 0;
									m_KickBallCount = 0;
									KickBall = 0;
									Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
									Walking::GetInstance()->A_MOVE_AMPLITUDE = (-1)*m_RLTurn;
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
									Walking::GetInstance()->A_MOVE_AMPLITUDE = (-1)*m_RLTurn;

									if(DEBUG_PRINT == true)
										fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
								}
							}
					
	
						}
		}
	
//moveWindow("Batas", 10, 1150);
// moveWindow("Trackbar Bola", 700, 0);
  
//   imshow( "cari", threshold_output );
   imshow( "Source", src );
   //moveWindow("Bola", 370, 450);
   
 //moveWindow("detected lines", 10, 50);
// moveWindow("Source", 10, 50);
 moveWindow("Trackbar Lapangan", 370, 0);
 //imshow("detected lines", drawing);
 
   
   if(waitKey(30)== 27)break;        
    }
	 
   return 0;
}

