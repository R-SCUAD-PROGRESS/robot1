/*
 * BISMILLAH JUARA 
 * ROAD TO ROBO CUP
 *fokus pada tujuan utama
 * main.cpp
 *  RSCUAD ROBOT AHMAD DAHLAN
 *  Created on: 2017. 24. 4.
 *  Revision : 2018. 16. 03
 *  Revision : 2019
 *      Author: robotis, Suhu RSCUAD, Jihad Rahmawan, Syahid Al Irfan, muhammad Annas
 */

//#include <opencv2/highgui/highgui.hpp>
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

#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>


#include <opencv2/video/video.hpp>

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

#include <vector>
#include <stdio.h>
#include <stdlib.h>
//#include "port.h"

#define BUFSIZE 2048
#define SERVICE_PORT 3838
#define POLL_TIMEOUT 20
using namespace std;
using namespace cv;


#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"
///////////////////////////////////////////////////////
int arah;
int lihat=0;
int tendang=0;
int kembali=0;
int tahan=0;
int jalan=10;
int tambah=0;
int lurus=0;
int keluar=0;
int kanan=0;
int muter=0;
int sampingaccel=0,sampinggyro=0;
int waktu=0,counter=0;

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

static const double VIEW_V_ANGLE = 46.0; //degree
static const double VIEW_H_ANGLE = 58.0; //degree

int last=0;
int ww = 0;
int kaki_kanan, kaki_kiri,bahu_kiri,bahu_kanan;
struct sockaddr_in myaddr;	/* our address */
	struct sockaddr_in remaddr;	/* remote address */
	socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
	int recvlen;			/* # bytes received */
	int fd;				/* our socket */
	unsigned char buf[BUFSIZE];	/* receive buffer */
	
	int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    
   // cout << "\n Hit 'q' to exit...\n";

    //char ch = 0;

    double ticks = 0;
    bool found = false;

double precTick,dT;

int initreferee()
{
/* create a UDP socket */

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
	{
		
		perror("cannot create socket\n");
		return 0;
	}

	/* bind the socket to any valid IP address and a specific port */

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(SERVICE_PORT);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) 
	{
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
   for (int i = 0; i < 5; i++)
	   {
      int rv = poll(ufds, 1, POLL_TIMEOUT);  // Wait up to POLL_TIMEOUT ms
     
      // Check to see if we've received a packet
		if (rv > 0) 
		{
         recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
			if (recvlen ==116) 
			{
            //parseData((RoboCupGameControlData*)buffer);
			buf[recvlen] = 0;
			//printf("received message: \"%s\"\n", buf);
			//last=buf[9];
				switch (buf[9])
				{
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

	
	//////////////////////////////////////////////////////////////////////////////////
using namespace cv;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);
		
Mat src; Mat src_gray;
 int th2 = 255, th1 =8;
 RNG rng(12345);
 
//========================= variabel lapangan ===================================//
 
 int h_min = 34, h_max = 82, s_min = 36, s_max = 255, v_min = 29, v_max = 255,conture = 100;
//int h_min = 255, h_max = 255, s_min = 255, s_max = 255, v_min = 255, v_max = 255,conture = 100;
 Mat drawing, drawing2;

//========================= variabel bola ===================================//

int h_min1 = 0, h_max1 = 255, s_min1 =153, s_max1 = 255, v_min1 = 123, v_max1 = 255;
Mat threshold_output;
Mat mask, mask2, bl, mn, gbl;
int x = -5,y = -5;	  
//==================================//
//========================= variabel gawang ===================================//
 Mat gh;
int gh_min2 = 0, gh_max2 = 255, gs_min2 = 0, gs_max2 = 255, gv_min2 = 136, gv_max2 = 255;
int gconture2 = 13,agconture2 = 153,gth2_max = 255, gth1 = 255, th23 = 183, th13 = 8; 
Mat gthreshold_output;
Mat src_copy;

 int gthresh = 100;
int blockSize = 2;
int apertureSize = 3;
double kk = 0.04;

Point vb  = Point(-6, -6), ff = Point(-6, -6);
Mat ssg, sg;
Mat dst, cdst, ual, ul;
Point gw = Point(0,0);
int tgh, kh = 0;
//==================================//


int a=10;


bool recap = true;
bool useMorphOps = true;
bool noball;
int noballcount;
int max_nobalcount;


Point2D ball_position;
Point2D pos;


double pan;
double tilt;
double pan_range;
double pan_percent;
double tilt_min;
double tilt_range;
double tilt_percent;

int ty,td, lagi;
int u;
bool socer_mode; //===================swicth soker mode atau vision mode = false
bool cari_ball;
char geser;
//===========kecepatan perpindahan
int sprinmax_speed, slow_speed, belokmax_speed, belokmid_speed, belokmin_speed; 
int scan_ball_speed;

int periode_time1, periode_time2;
int orientasi;
 int mode_rotasi;
 int s,k,last_acc;
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



void sighandler(int sig)
{
    exit(0);
}

 void motion(int indek){
	                    
						Walking::GetInstance()->Stop();
sleep(1);
			Action::GetInstance()->m_Joint.SetEnableBody(true, true);
                        MotionManager::GetInstance()->SetEnable(true);
			usleep(200);
                        Action::GetInstance()->Start(indek);
			usleep(300);
                        while(Action::GetInstance()->IsRunning());
						//usleep(8*1000);
						//tilt=0;
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Walking::GetInstance()->Stop();
						
						Head::GetInstance()->MoveToHome();
						//usleep(8*1000);
						Head::GetInstance()->MoveTracking(ball_position);	 
                        pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			            tilt=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
sleep(1);
}			   



void tblap()
{
		//============================= trackbar lapangan ============================//	   
		namedWindow("Trackbar Lapangan", CV_WINDOW_NORMAL);
   		createTrackbar("R1_MIN: ","Trackbar Lapangan",&h_min,255);
		createTrackbar("R1_MAX: ","Trackbar Lapangan",&h_max,255);
		createTrackbar("R2_MIN: ","Trackbar Lapangan",&s_min,255);
		createTrackbar("R2_MAX: ","Trackbar Lapangan",&s_max,255);
		createTrackbar("R3_MIN: ","Trackbar Lapangan",&v_min,255);
		createTrackbar("R3_MAX: ","Trackbar Lapangan",&v_max,255);
		createTrackbar("Conture: ","Trackbar Lapangan",&conture,255);
	moveWindow("Trackbar Lapangan", 370, 0);
	
}

void tbbol()
{
//============================= trackbar bola ============================//	
		 namedWindow("Trackbar Bola", CV_WINDOW_NORMAL);
   		createTrackbar("H_MIN: ","Trackbar Bola",&h_min1,255);
		createTrackbar("H_MAX: ","Trackbar Bola",&h_max1,255);
		createTrackbar("S_MIN: ","Trackbar Bola",&s_min1,255);
		createTrackbar("S_MAX: ","Trackbar Bola",&s_max1,255);
		createTrackbar("V_MIN: ","Trackbar Bola",&v_min1,255);
		createTrackbar("V_MAX: ","Trackbar Bola",&v_max1,255);
//=============================================//	
}

void tbgaw()
{
//============================= trackbar gawang ============================//	
		namedWindow("Trackbar gawang", CV_WINDOW_NORMAL);
		createTrackbar("R1_MIN2: ","Trackbar gawang",&gh_min2,255);
		createTrackbar("R1_MAX2: ","Trackbar gawang",&gh_max2,255);
		createTrackbar("R2_MIN2: ","Trackbar gawang",&gs_min2,255);
		createTrackbar("R2_MAX2: ","Trackbar gawang",&gs_max2,255);
		createTrackbar("R3_MIN2: ","Trackbar gawang",&gv_min2,255);
		createTrackbar("R3_MAX2: ","Trackbar gawang",&gv_max2,255);
		createTrackbar("gconture2: ","Trackbar gawang",&gconture2,255);
		createTrackbar("agconture2: ","Trackbar gawang",&agconture2,255);
		createTrackbar("TH1   : ","Trackbar gawang",&th13,255);
		createTrackbar("TH2   : ","Trackbar gawang",&th23,255);
		
//=============================================//	
}



void lap(Mat src_s, int jl)
{
	
	//=============== lapangan =======================//

   cvtColor( src_s, src_gray, CV_BGR2HSV );
   inRange(src_gray,Scalar(h_min,s_min,v_min),Scalar(h_max, s_max, v_max),threshold_output);
  // erode( threshold_output, threshold_output, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)) );
   //dilate( threshold_output, threshold_output, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );
 
  //    imshow( "Lapangan", threshold_output );
 //===============================================//		
   src_copy = src_s.clone();
 
   vector<vector<Point> > contours;
   vector<Vec4i> hierarchy;
   
   // mencari controur lapangan
   findContours( threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

   vector<vector<Point> >hull( contours.size() );
 
   
//=============== mencari luas lapangan =====================//
   for( int i = 0; i < contours.size(); i++ )
      {  convexHull( Mat(contours[i]), hull[i], false ); 
      }
//======================================================//

   drawing = Mat::zeros( src.size(), CV_8UC1 );
   drawing2 = Mat::zeros( src.size(), CV_8UC1 );
   sg = Mat::zeros( src.size(), CV_8UC1 );
   for( int i = 0; i< contours.size(); i++ )
      {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	if (contourArea(contours[i]) > conture*40)
	{
        drawContours( src, hull, i, Scalar(11,0,0), 2, 8, vector<Vec4i>(), 0, Point(0,0));
        drawContours(drawing, hull, i, Scalar(255, 255, 255), CV_FILLED,  8, vector<Vec4i>(), 0, Point(0,0));	
		drawContours( sg, hull, i, Scalar(255,255,255),  CV_FILLED, 8, vector<Vec4i>(), 0, Point(0,0) );
		drawContours(drawing2, hull, i, Scalar(255, 255, 255), CV_FILLED); 		
    }
      }
		
		bitwise_and(src_copy,src_copy,ssg,sg = sg);
		imshow("ssg", ssg);
		imshow("sg", sg);
		circle(src, Point(150,190 ), 3, Scalar(23,35,255), -1);
}
		





void gaw(Mat sdrawing2, Mat ssrcc)
{
	
	//==================================================================================== pemilihan metode ====================================================================//
	  
	  int uo =  threshold_output.at<uchar>(120,150 );// batas bounding rect kanan
	  int uos =  threshold_output.at<uchar>(120,25 );// batas bounding rect kiri
	  tgh =  threshold_output.at<uchar>(190,150 );// batas bounding rect kiri
		//circle(src, Point(150,170 ), 3, Scalar(23,35,255), -1);
		
//*		
		if (tgh > 0)/// kondisi metode boundingrect
		{
			circle(src, Point(10,10 ), 10, Scalar(23,35,255), -1);
//	*/	
		
		
//====================================================================================== boo ===================================================//
	  
 
 cvtColor(ssrcc, ul, CV_RGB2HSV);
 imshow("ssrcc", ssrcc);
 inRange(ul,Scalar(gh_min2,gs_min2,gv_min2),Scalar(gh_max2, gs_max2, gv_max2),ul);
 erode( ul, ul, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
 dilate( ul, ul, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

 imshow("ul", ul);
 
 threshold(ul,ual,th13,th23,THRESH_BINARY+THRESH_OTSU);
 Canny(ual, dst, 5, 5, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);
 Mat rio;
  cvtColor(cdst,rio, CV_BGR2HSV);

  vector<vector<Point> > contours3;
  vector<Vec4i> hierarchy3;

  findContours( ual, contours3, hierarchy3, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  //====================== Deteksi conture menjadi persegi ============//
  vector<vector<Point> > contours_poly3( contours3.size() );
  vector<Rect> boundRect3( contours3.size() );
  vector<Point2f>center3( contours3.size() );
  vector<float>radius3( contours3.size() );

  for( int i = 0; i < contours3.size(); i++ )
     { approxPolyDP( Mat(contours3[i]), contours_poly3[i], 3, true );
       boundRect3[i] = boundingRect( Mat(contours_poly3[i]) );
       minEnclosingCircle( (Mat)contours_poly3[i], center3[i], radius3[i] );
     }


 //================ membuat persegi, bonding rect dan titik tengah========////
 
  for( int i = 0; i< contours3.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       //drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
if (contourArea(contours3[i]) > gconture2*20 )
			{
	rectangle(src, boundRect3[i].tl(), boundRect3[i].br(), (0,200,300), 5);
	circle( src, center3[i], 3, Scalar(0, 0, 255), CV_FILLED, CV_AA  );
	//line( src, boundRect[i].tl(), center[i], Scalar(1,100,0), 8, CV_AA);
	gw.x = (center3[0].x+center3[1].x)*0.5;
	
			}
     //  circle( src, center[i], (int)radius[i], color, 2, 8, 0 );
     }
//================================================ akhir boo ===================================================//
    }
		else  /// kondisi metode koordinat sudut
		{
			//drawing2 = sdrawing2;
	for (int i = 0; i < sdrawing2.cols; i++) {
    for (int j = 0; j < sdrawing2.rows; j++) {

		int hj =  sdrawing2.at<uchar>(j, i);
		if (hj > 0)
		{
		sdrawing2.at<uchar>(j, i) = 0;
		}
		else
		{
		sdrawing2.at<uchar>(j, i) = 255;
		}
	  //  cout<<pixValue<<endl;
        
     }
}
		Mat gmask, gmask2;
		bitwise_and(src,src,gmask,sdrawing2 = sdrawing2);
	
		inRange(gmask,Scalar(gh_min2,gs_min2,gv_min2),Scalar(gh_max2, gs_max2, gv_max2),gmask);
		erode( gmask, gmask, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
		//dilate( gmask, gmask, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );
		imshow("gmask",gmask);
		//bitwise_and(gmask,gmask,gbl,drawing2 = drawing2);
		
		vector<Vec4i> lines;
		Vec4i l;
		Mat ze = Mat::zeros( src.size(), CV_32FC1 );
		Canny(gmask, gmask, 5, 5, 3);
		imshow("canny",gmask);
		HoughLinesP(gmask, lines, 5, CV_PI/180, 50, 100, 150 );
		int x1,x2,y1,y2, xa,xd,ya,yd, x3,y3,xk,yk;
   for( size_t i = 0; i < lines.size(); i++ )
  {
    l = lines[i];
    //line( src_s, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(100,0,0), 4, CV_AA);
    
    if (l[1] <= l[3] && l[1]+l[1]*0.6 >= l[3])	// Garis ke pinggir pos kanan
   {
    line( ze, Point(0, l[1]), Point(400, l[1]), Scalar(100,0,0), 4, CV_AA);
//line( src_s, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(100,0,0), 4, CV_AA);
//circle( src_s, Point( l[0], l[1]), 9,  Scalar(21,0,99), 2, 8, 0 );
    x1 = l[0];
    x2 = l[2];
    }
    else if (l[3] <= l[1] && l[3]+l[3]*0.3 >= l[1])	// Garis ke pinggir pos kiri
   {
    line( ze, Point(0, l[3]), Point(400, l[3]), Scalar(100,21,0), 4, CV_AA);
//line( src_s, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(100,0,0), 4, CV_AA);
//circle( src_s, Point( l[0], l[1]), 9,  Scalar(255,255,255), 2, 8, 0 );
    xa = l[0];
    xd = l[2];
    }
    if (l[0] <= l[2] && l[0] >= l[2]-l[2]*0.6)	// Garis ke atas
   {
    line( ze, Point(l[2], 0), Point(l[2], 400), Scalar(100,0,0), 4, CV_AA);   
    }
    if (l[0] >= l[2] && l[0] <= l[2]+l[2]*0.6)	// Garis ke atas
   {
    line( ze, Point(l[2], 0), Point(l[2], 400), Scalar(100,0,0), 4, CV_AA);   
    }
    //line( ze, Point(xk, 0), Point(xk, 400), Scalar(100,0,0), 4, CV_AA);
    //line( ze, Point(x1, l[1]), Point(x3, l[3]), Scalar(100,0,0), 4, CV_AA);
    //line( ze, Point(l[0], 0), Point(l[2], 400), Scalar(100,0,0), 4, CV_AA);
   }


cornerHarris( ze, gh, blockSize, apertureSize, kk, BORDER_DEFAULT );

  for( int m = 0; m < gh.rows ; m++ )
     { for( int n = 0; n < gh.cols; n++ )
          {
            if( (int) gh.at<float>(m,n) > gthresh )
              {
	    /*  for (int c = 0; c < 4; c++)
	       {
	       vb[c] = Point(n,m);
	       }*/
               //circle( src_s, Point( n, m), 9,  Scalar(21,255,99), 2, 8, 0 );
	       if (n > x1 && n >x2)
{
	       ff = Point( n, m);
}	       //line( src_s, Point(vb[0].x, 0), Point(vb[0].x, 400), Scalar(100,0,0), 4, CV_AA);
		if (n < x1 && n <x2)
		{
		vb = Point( n, m);
		}
	       //line( src_s, Point(vb[].x, 0), Point(vb[0].x, 400), Scalar(100,0,0), 4, CV_AA);
		//cout<<vb[0]<<"==="<<vb[1]<<"==="<<vb[2]<<"==="<<vb[3]<<endl;
	      // cout<<n<<"==="<<x1<<"||"<<x2<<endl;
	      }
	      
          }
     }  
     circle( src, ff, 9,  Scalar(21,255,99), 2, 8, 0 );
     circle( src, vb, 9,  Scalar(21,0,99), 2, 8, 0 );
	 if (vb.x>0 && ff.x>0)
     {
     line( src, vb, ff, Scalar(54,8,80), 4, CV_AA);
     
     }
     line( src, vb, Point(vb.x,400), Scalar(0,54,255), 4, CV_AA);
     line( src, ff, Point(ff.x,400), Scalar(0,255,20), 4, CV_AA);
     circle( src, ff, 9,  Scalar(21,200,99), CV_FILLED, 8, 0 );//titik kanan dri layar
     circle( src, vb, 9,  Scalar(21,0,99), CV_FILLED, 8, 0 );//titik kiri dri layar
	 gw.x =(ff.x+vb.x)*0.5;
//	 has(ff.x, ff.y);
    //  cout<<l[0]<<"==="<<l[1]<<"||"<<l[2]<<endl;
   imshow( "zeros", ze );
		}
//================================================ akhir pemilihan metode ===================================================//	
	
	
      	
        
     }

	//===========================================================//







 //======================inisialisasi bola===========================//
void inisialisasibola()
{
cv::setIdentity(kf.transitionMatrix);
 kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
}

void habis_nendang()
{
//Walking::GetInstance()->HIP_PITCH_OFFSET=31.0;
Walking::GetInstance()->P_OFFSET=-4.1;
tambah=0;
//kembali=0;
ty=0;

if ( ty ==0 )
{

td +=5;
if (td >=75 &&td<=110)
{
td =0;
}

}
kembali ++;

//cout <<kembali <<endl;
if (kembali >=100)
{tendang =0;
}
//cout<<"tendang=" <<tendang<<endl;							
//cout <<"masuk kok"<<endl;
Head::GetInstance()->MoveByAngle(ty,td);
//Head::GetInstance()->MoveByAngle(pan,tilt);
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
}
void caribola()
{
tambah=0;
Walking::GetInstance()->P_OFFSET=-4.1;

ty=0;

if ( ty ==0 )
{

td +=5;
if (td >=75 &&td<=110)
{
td =0;
}

}
kembali ++;

//cout <<kembali <<endl;
if (kembali ==300)
{tendang =0;
}
//cout<<"tendang=" <<tendang<<endl;							
//cout <<"masuk kok"<<endl;
Head::GetInstance()->MoveByAngle(ty,td);
//Head::GetInstance()->MoveByAngle(pan,tilt);
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
}


 void bola(Mat vdrawing2, int es)
 {
	 m_NoBallMaxCount = 10;
					m_NoBallCount = m_NoBallMaxCount;
					m_KickBallMaxCount = 5;
					m_KickBallCount = 0;

					m_FollowMaxFBStep = 25.0;
					m_FollowMinFBStep = 5.0;
					m_FollowMaxRLTurn = 20.0;
					m_FitFBStep = 3.0;
					m_FitMaxRLTurn = 20;//35.0;
					m_UnitFBStep = 0.3;
					m_UnitRLTurn = 1.0;
					int d=-30;int t=0;
					int NoBallMaxCount = 15;
					int NoBallCount=0;
	       //=========== tmbhn =====================//
      	
	
precTick = ticks;
        ticks = (double) cv::getTickCount();

         dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
       

        
        

        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A
 
            //cout << "MASUK CES" << endl ;

            state = kf.predict();
            //cout << "State post:" << endl << state << endl;

            cv::Rect predRect;
            predRect.width = state.at<float>(7);
            predRect.height = state.at<float>(9);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            cv::circle(src, center, 1, CV_RGB(10,0,0), -1);

            cv::rectangle(src, predRect, CV_RGB(10,0,0), 2);
        }

        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(ssg, blur, cv::Size(5,5), 5.0, 5.0);
        // <<<<< Noise smoothing

       
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
       
        cv::Mat rangeRes = cv::Mat::zeros(ssg.size(), CV_8UC1);
        cv::inRange(frmHsv, cv::Scalar(h_min1, s_min1, v_min1),
                    cv::Scalar(h_max1,s_max1, v_max1), rangeRes);
        // <<<<< Color Thresholding

        // >>>>> Improving the result
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(1,1), 2);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(1,1), 2);

        cv::imshow("bola", rangeRes);

        // >>>>> Contours detection
        vector<vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // <<<<< Contours detection

        // >>>>> Filtering
        vector<vector<cv::Point> > balls;
        vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
           if (ratio > 0.6f && bBox.area() >= 40)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
		
            }
        }
        // <<<<< Filtering

       // cout << "bola:" << ballsBox.size() << endl;

        // >>>>> Detection result
        for (size_t i = 0; i < balls.size(); i++)
        {
            cv::drawContours(src, balls, i, CV_RGB(255,0,0), 5);
           // cv::rectangle(src, ballsBox[i], CV_RGB(0,255,0), 2);
cv::rectangle(src, ballsBox[i], CV_RGB(0,255,0), 3);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(src, center, 2, CV_RGB(20,150,20), -1);

            /*stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);*/
x=center.x;
y=center.y;
        }
//*/
// cout << "=============================" << endl;

 							
							if(( x <= 0 && y <=0)) {
							ball_position.X = -1.5;//-1;
							ball_position.Y = -1.5;//-1;
							Head::GetInstance()->MoveTracking();
							noballcount++;
							if(noballcount>max_nobalcount){
								Head::GetInstance()->MoveToHome(); //rst sudut leher
								noball=true;
								
							} else {
								noball=false;
							}
							  
							} 
							else
							{
                          	noballcount=0;					
							pos.X=x;//-25;
							pos.Y=y;//-25;
							Point2D center = Point2D(170,150);//(85, 70);
							Point2D offset = pos - center;

							offset *= -1;//-0.9;//0.5//1 // Inverse X-axis, Y-axis
							//offset.X *= ((VIEW_H_ANGLE /340)); //310)); // pixel per angle
							//offset.Y *= ((VIEW_V_ANGLE /300)); //230)); // pixel per angle
							
							offset.X *= ((170 / 150)*0.11); // pixel per angle
							offset.Y *= ((170 / 150)*0.11); // pixel per angle
							ball_position = offset;

// cout << "aktif " << pos.X << " ====== " << pos.Y << endl;
							Head::GetInstance()->MoveTracking(ball_position);
 //cout << "aktif lagi " << offset.X << " ---- " << ball_position.X << endl;
 //cout << "pan"<<pan <<endl;
							noball=false;
							
							m_KickTopAngle = -5.0;
						m_KickRightAngle = -30.0;
						m_KickLeftAngle = 30.0;
						m_NoBallCount = 0;
							}
 	//circle(src, Point(170,150 ), 14, Scalar(255,255,255), -1);
 }



void motion_cariball(){ 
//cout << "masuk sini juga coiiiii"<<endl; 
Walking::GetInstance()->P_OFFSET=-4.1; 
tambah=0; 
//cout << td << " td---ty " << ty << endl;
	                       			if(td>0)
						   {
						   //if((ty>=-95)&&(ty<=75))
if((ty>=-95)&&(ty<=90))
							{ty += 5;}
						   //if(ty==75)
if(ty==90)
							{ty=-95; td+=20;}
						  if(td>110)
						//if(td>150)
						    {
							   td=-5;  
							   lagi++;
                              			 if(lagi==3)
						     {
								   lagi=0;
								   ty=-70;
						     }
						    }
						   }						   
						 else
						   {
					        	 //if(((ty>=-95))&&(ty<=75))
if(((ty>=-95))&&(ty<=90))
								{ty += 5;}
						     	 if(ty==35)
								{ty=-60; td += 15;}
						   }
//sleep(1);
						        //Walking::GetInstance()->PERIOD_TIME=600;//650;
		                        //Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
								//if(lagi<=15&&lihat==0)
								/*if (lagi<=2 &&lihat==0)
								{
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
								Walking::GetInstance()->Start();
								Walking::GetInstance()->X_MOVE_AMPLITUDE =5;// scan_ball_speed;;
								Walking::GetInstance()->Start();
cout<<"cari bola gan\n"<<endl;
								
							    }*/
								//if(lagi>16&&lihat==0)
								
						
							
				/*if(lihat==100)
				{ty=0;td=80;
					for(int i=0;i<10000;i++)
					{
						Walking::GetInstance()->PERIOD_TIME=600;//650;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = -3;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 5; 
						Walking::GetInstance()->Start();
						//cout<<"JALAN DARI PICK-UP"<<endl;
cout<<"bola dapat gan"<<endl;
					}
				}*/
					Head::GetInstance()->MoveByAngle(ty,td);
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
// cout << pan << " p---t " << tilt << endl;
			
				
	               
}
 /*void followball_engine(){
//	 gaw(drawing2, ssg);
//	 if (tgh > 0)
//	 {
		     Head::GetInstance()->MoveTracking(ball_position);	 
		     pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			 tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
			 //===========limit leher agar tidak liat tangannya sendiri :D =============
			 if((pan>56 || pan<-56) && tilt<=-20){
				 Head::GetInstance()->MoveByAngleOffset(pan,tilt);
			 }
		      Walking::GetInstance()->PERIOD_TIME=650;
		      Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			   
			 if (tilt>5){
					Walking::GetInstance()->X_MOVE_AMPLITUDE =20; //sprinmax_speed; 
					Walking::GetInstance()->Start();
						if(pan>50 && pan<55){  // BELOK TAJAM
							Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmax_speed; 
							Walking::GetInstance()->Start();
							}
						else if(pan<=50 && pan>25){ // BELOK MENENGAH
							Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmid_speed;
							Walking::GetInstance()->Start();
							} 
						else if (pan<=25  && pan >5 ){
							Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmin_speed;
							Walking::GetInstance()->Start();
							}
						else if(pan<=5 && pan>-5){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =0;
							Walking::GetInstance()->Start();
							}
						else if(pan>-25 && pan<=-5){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmin_speed*-1);
							Walking::GetInstance()->Start();
							}
						else if(pan>-50 && pan<=-25){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmid_speed*-1);
							Walking::GetInstance()->Start();
							} 
						else if(pan>=-55 && pan<=50){
						Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmax_speed*-1);
						Walking::GetInstance()->Start(); 
							}
				}
			    else if(tilt<=4){
					Walking::GetInstance()->X_MOVE_AMPLITUDE = slow_speed; 
					Walking::GetInstance()->Start();
			            if(pan>50 && pan<55){  // BELOK TAJAM
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 20;//belokmax_speed; 
							Walking::GetInstance()->Start();
							}
						else if(pan<=50 && pan>25){ // BELOK MENENGAH
							Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmid_speed;
							Walking::GetInstance()->Start();
							} 
						else if (pan<=25  && pan >5 ){
							Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmin_speed;
							Walking::GetInstance()->Start();
							}
						else if(pan<=5 && pan>-5){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =0;
							Walking::GetInstance()->Start();
							}
						else if(pan>-25 && pan<=-5){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmin_speed*-1);
							Walking::GetInstance()->Start();
							}
						else if(pan>-50 && pan<=-25){
							Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmid_speed*-1);
							Walking::GetInstance()->Start();
							} 
						else if(pan>=-55 && pan<=-50){
						Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmax_speed*-1);
						Walking::GetInstance()->Start(); 
							}
				}
	/*
	}
	else
	{
		
						Walking::GetInstance()->PERIOD_TIME=600; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_MOVE_AMPLITUDE= 20;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->A_MOVE_AMPLITUDE = -40;
						Walking::GetInstance()->Start();
						usleep(8*1000);
	} //
 }*/
void head_trackonly(){
 Head::GetInstance()->MoveTracking(ball_position);	 
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 //tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
 printf("tilt: %d \t pan: %d \r", tilt, pan);
}			 	

 void geser_bola(int mode_ges){
	  
 }
 
 
	int oh = 0, fg = 0, mb = 0, jj = 0, hk = 0, bc = 0;
	
void bangun(){
sampingaccel=MotionStatus::RL_ACCEL;
sampinggyro=MotionStatus::RL_GYRO;
last_acc=MotionStatus::FB_GYRO;
cm730.ReadWord(1, MX28::P_PRESENT_POSITION_L, &kaki_kiri, 0);
cm730.ReadWord(2, MX28::P_PRESENT_POSITION_L, &kaki_kanan, 0);

			cout << "besar present loadnya 18 & 17 = " << kaki_kiri << " & " << kaki_kanan << endl;
			//cout << "GYRO & ACCEL = " << sampinggyro << " & " << sampingaccel<< endl;
			//cout << "fbaccel" <<last_acc<<endl;
			
	 if( kaki_kiri > 2750 || kaki_kiri < 1100  )
	 {
		// k=1;
	 }
	   else k=0;k =1;
	  if(k == 1){
		  
		  //last_acc=MotionStatus::FB_ACCEL;
		  if((kaki_kiri < 1100 && last_acc < 5)){
//cout << "dpn " << last_acc << endl;MotionManager::GetInstance()->SetEnable(false);
		  sleep(1);
			MotionManager::GetInstance()->SetEnable(true);
		//				motion(9);
						motion(196);
						Walking::GetInstance()->HIP_PITCH_OFFSET=31.5;//30.4;
						Walking::GetInstance()->P_OFFSET=-4.1;
						usleep(2500);
			
		  } 
		 if(kaki_kiri  > 2750 && sampingaccel >460 && sampingaccel < 560){MotionManager::GetInstance()->SetEnable(false);
		  sleep(1);
cout << "blkng " << last_acc << endl;
			  MotionManager::GetInstance()->SetEnable(true);
			//    		        motion(9);
					motion(195);
					Walking::GetInstance()->HIP_PITCH_OFFSET=31.5;//30.4;
					Walking::GetInstance()->P_OFFSET=-4.1;
					usleep(200);
			  
		  }
if((sampingaccel < 460)){
	waktu++;
//cout <<"waktu=" <<waktu <<endl; 

	if (waktu>=10)
		{
//cout <<"waktu=" <<waktu <<endl;        		
//cout << "KIRI " << last_acc << endl;MotionManager::GetInstance()->SetEnable(false);
		 	 sleep(1);
			MotionManager::GetInstance()->SetEnable(true);
					motion(61);
						//motion(195);
					Walking::GetInstance()->HIP_PITCH_OFFSET=31.4;//30.4;
					Walking::GetInstance()->P_OFFSET=-4.1;
					usleep(200);
			waktu=0;
		  } 
}

		 if((sampingaccel > 590)){
	waktu++;
//cout <<"waktu=" <<waktu <<endl; 

	if (waktu>=10)
		{
//cout <<"waktu=" <<waktu <<endl;        		
//cout << "kanan " << last_acc << endl;MotionManager::GetInstance()->SetEnable(false);
		 	 sleep(1);
			MotionManager::GetInstance()->SetEnable(true);
					motion(60);
						//motion(195);
						Walking::GetInstance()->HIP_PITCH_OFFSET=31.4;//30.4;
						Walking::GetInstance()->P_OFFSET=-4.1;
						usleep(200);
			waktu=0;
		  } 
}
if (sampingaccel > 460 && sampingaccel <590)
{waktu=0;}
	  }
	  
	  
}
	
 void shoot_to_goals(int &ey, int &yy){
			   orientasi = MotionStatus::kompas;
			   //cout << "orientasi = " << orientasi << endl;
			   /*if(orientasi>=0 && orientasi<=75){
				 mode_rotasi=1;  
			   } else if(orientasi>75 && orientasi <= 150){ 
				 mode_rotasi=0; 
			   }*/
			
if(  orientasi>=6 && orientasi<=250  ){//(orientasi>=0 && orientasi<=75){
				 mode_rotasi=1;  
			   } else if(  orientasi>250 && orientasi <485){//< 465){//399){//(orientasi>75 && orientasi <= 150){ 
				 mode_rotasi=0; 
			   } 
			   /*if((orientasi>=0 && orientasi<=1) || (orientasi>=399&& orientasi<=400))
			   {tt = 1;
				   break;
				} */
			   if((orientasi>=0 && orientasi<=5) || (orientasi>=485&& orientasi<=600))ey = 1;
			   //=======================motion keliling bola======================
					if( mode_rotasi==0  ){
						
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					
						//Walking::GetInstance()->HIP_PITCH_OFFSET=30.0;//30.4;
						//Walking::GetInstance()->P_OFFSET=-2.0;
						Walking::GetInstance()->P_OFFSET=-3.1;
						Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
						Walking::GetInstance()->PERIOD_TIME=600;//552;//600; 
						Walking::GetInstance()->X_MOVE_AMPLITUDE =1;//-3;// -3;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE= 41;//54;//41;//52;//54;//70;
						
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 10;//-12;//-20;
						Walking::GetInstance()->Z_MOVE_AMPLITUDE= 46;//38;//45;
						Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.10;
						//Walking::GetInstance()->Start();
						Walking::GetInstance()->Start();
						//cout <<pan<<endl;
						
						//cout << "Muter kiri" << endl;
						//yy = 0;
						
					}
					else if( mode_rotasi==1    )
					{
					
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						

						//Walking::GetInstance()->HIP_PITCH_OFFSET=25.1;
						//Walking::GetInstance()->P_OFFSET=-2.0;
						Walking::GetInstance()->P_OFFSET=-3.1;
						Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
						Walking::GetInstance()->PERIOD_TIME=600;//552;//600; 
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 1;//-3;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE= -41;//-45;//-52;//-70;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = -12;//-11;//20;
						Walking::GetInstance()->Z_MOVE_AMPLITUDE= 46;//45;
						Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.10;
						Walking::GetInstance()->Start();
						
						//cout << "Muter Kanan" << endl;
						//yy = 100;
					}
					
					/*else if(mode_rotasi==10 )
					{
						
						gaw(drawing2, ssg);
						ball_position.X = gw.x;
						ball_position.Y = gw.y;
						Walking::GetInstance()->Stop();
						usleep(8*1000);
						    Head::GetInstance()->MoveTracking(ball_position);	 
		                    pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			                tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
							
							Walking::GetInstance()->PERIOD_TIME=600;//650;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			               
								
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 2;//2; 
								Walking::GetInstance()->Start();
								
								if(pan>50){  // BELOK TAJAM
								Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmax_speed; 
								Walking::GetInstance()->Start();
								}
								else if(pan<=50 && pan>25){ // BELOK MENENGAH
								Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmid_speed;
								Walking::GetInstance()->Start();
								} 
								else if (pan<=25  && pan >5 ){
								Walking::GetInstance()->A_MOVE_AMPLITUDE = belokmin_speed;
								Walking::GetInstance()->Start();
								}
								else if(pan<=5 && pan>-5){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =0;
								Walking::GetInstance()->Start();
								}
								else if(pan>-25 && pan<=-5){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmin_speed*-1);
								Walking::GetInstance()->Start();
								}
								else if(pan>-50 && pan<=-25){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmid_speed*-1);
								Walking::GetInstance()->Start();
								} 
								else if(pan<=-50){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmax_speed*-1);
								Walking::GetInstance()->Start(); 
								}
								//

							
							cout << "sudah brooo" << endl;
					}*/
}

int main(int argc, char* argv[])//void)
{
	int trg = 0;
	initreferee();
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();
    minIni* ini = new minIni(INI_FILE_PATH);
    httpd::ini = ini;

    tblap();
	tbbol();
	tbgaw();
	inisialisasibola();
    VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,340);//420);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,300);//340);
   
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
	//Action::GetInstance()->m_Joint.SetEnableBody(false,false);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    Action::GetInstance()->Start(5);//Start(9);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	//Head::GetInstance()->m_Joint.SetEnableHeadOnly(false,false);
   	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	//Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(false,false);
    	//Action::GetInstance()->Start(9);
	//=====================================================================
	//               PROGRAM UTAMA 
	//====================================================================
	while(1)
	{
		int pl = 0, tt = 0;
		while(1)
		{
			//Walking::GetInstance()->Stop();
			StatusCheck::Check(cm730);
			if(StatusCheck::m_cur_mode == READY)
			{
				cout << "Plan 1 : dari kick OFF " << endl;
				pl = 1;
			}
			if(StatusCheck::m_cur_mode == VISION)
			{
				cout << "Plan 2 : dari Pick Up " << endl;
				pl = 2;
				//lihat=100;
			}
			if(StatusCheck::m_cur_mode == MOTION)
			{
				cout << "Plan 3 : Kick OFF lawan " << endl;
				pl = 3;
			}
			if(StatusCheck::m_cur_mode == SOCCER)
			{
				cout << "Plan 4 : drop ball" << endl;
				pl = 4;
			}
			if(StatusCheck::mulai == 1)break;
		}
		socer_mode=true;
		if (socer_mode==true)
		{
			/*	
			referee ();
			while(referee()!=3){
			printf(" %d \n", referee());
			Walking::GetInstance()->Stop();
			}
			//	*/
			//tendang=1;
			int gg = 0, waw = 0, kj = 0;	
			//sleep(10);
			/*
			while(1)
			{
				bangun();
				cap.read(src);
				lap(src, 1);
				bola(drawing, 1);
				imshow("src", src);
				if( ( waw >= 600)  )break;
				if(x<=0)
				motion_cariball();	
				x = -5;
				y = -5;
			
				//
				waw++;
				kj++;
				cout << "cooo == " << waw << endl;
		
				Walking::GetInstance()->PERIOD_TIME=600;//650;
				Walking::GetInstance()->A_MOVE_AMPLITUDE = -4;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 15; 
				Walking::GetInstance()->Start();
				//sleep(1);
				///

			}*/
			if(pl == 3) //tendang=2;sleep(10);
			{
				
			int gg = 0, waw = 0, kj = 0;	
				sleep(10);
				while(1)
					
				{	
					tendang=2;
					orientasi = MotionStatus::kompas;
					//cout << "rientasi = " << orientasi << endl;
					bangun();
					//habis_nendang();
					if(waw <=400)
					{
						cap.read(src);
						lap(src, 1);
						bola(drawing, 1);
						imshow("src", src);
					}
					if( ( waw >= 10) )break;
					if(x<=0)
					//motion_cariball();	
					//habis_nendang();
					x = -5;
					y = -5;
			
					//* 
					waw++;
					kj++;
					//cout << "cooo == " << waw << endl;
		
					Walking::GetInstance()->PERIOD_TIME=600;//650;
					//Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;//13; 
					Walking::GetInstance()->Start();
					//sleep(1);
					//*/
				}
			}

			if(pl == 2)
			{
				int gg = 0, waw = 0, kj = 0;	
				//sleep(10);
				while(1)
					
				{	
					tendang=2;
					orientasi = MotionStatus::kompas;
					//cout << "rientasi = " << orientasi << endl;
					
					bangun();
					//habis_nendang();
					if(waw <=400)
					{
						cap.read(src);
						lap(src, 1);
						bola(drawing, 1);
						imshow("src", src);
					}
					if( ( waw >= 10) )break;
					if(x<=0)
					//motion_cariball();	
					//habis_nendang();
					x = -5;
					y = -5;
			
					//* 
					waw++;
					kj++;
					//cout << "cooo == " << waw << endl;
		
					Walking::GetInstance()->PERIOD_TIME=600;//650;
					//Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;//13; 
					Walking::GetInstance()->Start();
					//sleep(1);
					//*/
				}
			}
			if(pl == 4)
			{
				int gg = 0, waw = 0, kj = 0;	
				//sleep(10);
				while(1)
					
				{	
					tendang=2;
					orientasi = MotionStatus::kompas;
					//cout << "rientasi = " << orientasi << endl;
					//cout << "kembali=" <<kembali <<endl;
					bangun();
					//habis_nendang();
					if(waw <=400)
					{
						cap.read(src);
						lap(src, 1);
						bola(drawing, 1);
						imshow("src", src);
					}
					if( ( waw >= 10) )break;
					if(x<=0)
					//motion_cariball();	
					//habis_nendang();
					x = -5;
					y = -5;
			
					//* 
					waw++;
					kj++;
					//cout << "cooo == " << waw << endl;
		
					Walking::GetInstance()->PERIOD_TIME=600;//650;
					//Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;//13; 
					Walking::GetInstance()->Start();
					//sleep(1);
					//*/
				}
				/*int gg = 0, waw = 0, kj = 0;	
				sleep(4);//*
				while(1)
				{
					bangun();
					cap.read(src);
					lap(src, 1);
					bola(drawing, 1);
					imshow("src", src);
					if( ( waw >= 1000)  )break;
					//nambah1
					if(x<=0&& tendang ==0)
					motion_cariball();	
					x = -5;
					y = -5;
					//* 
					waw++;
					kj++;
					cout << "cooo == " << waw << endl;
		
					Walking::GetInstance()->PERIOD_TIME=600;//650;
					Walking::GetInstance()->A_MOVE_AMPLITUDE = -5;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 3;//13; 
					Walking::GetInstance()->Start();
					//sleep(1);
					///
				}*/
				//*/
				/*while(1)
				{
					//Walking::GetInstance()->Stop();
					//usleep(100);
					orientasi = MotionStatus::kompas;
					cout << "orientasi = " << orientasi << endl;
					if(orientasi>=0 && orientasi<=250){//(orientasi>=0 && orientasi<=75){
					mode_rotasi=1;  
					} else if(orientasi>250 && orientasi < 399){//(orientasi>75 && orientasi <= 150){ 
					mode_rotasi=0; 
				} 
				if((orientasi>=0 && orientasi<=1) || (orientasi>=399&& orientasi<=400))
				{tt = 1;
				   break;
				}
				//=======================motion keliling bola======================
					if( mode_rotasi==0){
						Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						//Walking::GetInstance()->A_MOVE_AIM_ON == true;
						//Walking::GetInstance()->Z_OFFSET = 30;
						//Walking::GetInstance()->HIP_PITCH_OFFSET=24;//24.3;
						//Walking::GetInstance()->PERIOD_TIME=552;//600; 
						//Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE =0;//0;// -3;
						//Walking::GetInstance()->Start();
						//Walking::GetInstance()->Y_MOVE_AMPLITUDE= 40;//70;
						//Walking::GetInstance()->Z_MOVE_AMPLITUDE= 50;
						//Walking::GetInstance()->Start();
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 30;//-20;
						Walking::GetInstance()->Start();
						//Walking::GetInstance()->A_MOVE_AIM_ON == true;
						cout << "Muter kiri" << endl;
						//yy = 100;
						
					}
					else if( mode_rotasi==1)
					{Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						//Walking::GetInstance()->A_MOVE_AIM_ON == true;
						//Walking::GetInstance()->Z_OFFSET = 30;
						//Walking::GetInstance()->HIP_PITCH_OFFSET=24;//24.3;
						//Walking::GetInstance()->PERIOD_TIME=552;//600; 
						//Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE =0;// -3;
						//Walking::GetInstance()->Start();
						//Walking::GetInstance()->Y_MOVE_AMPLITUDE= 40;//70;
						//Walking::GetInstance()->Z_MOVE_AMPLITUDE= 50;
						//Walking::GetInstance()->Start();
						Walking::GetInstance()->A_MOVE_AMPLITUDE = -30;//-20;
						Walking::GetInstance()->Start();
						//Walking::GetInstance()->A_MOVE_AIM_ON == true;
						cout << "Muter kiri" << endl;
						
					}
	}*/
			}
			int frame_width=   cap.get(CV_CAP_PROP_FRAME_WIDTH);
			int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
			// VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);
			while(1)	
			{ 
				Mat frame;
				cap >> frame;
				//video.write(frame);
				StatusCheck::Check(cm730);
				if(StatusCheck::mulai == 1)break;
				Walking::GetInstance()->A_MOVE_AIM_ON == true;
				bangun();		
				orientasi = MotionStatus::kompas;
				//cout << pan << "]]]]]]" << tilt << endl;
				//drawing = Mat::zeros( src.size(), CV_8UC1 );
				ssg = Mat::zeros(src.size(), CV_8UC1);
				vb  = Point(-6, -6); ff = Point(-6, -6);
				//int x = -5;y = -5;	  
				max_nobalcount=5; 
				cap.read(src);
	    
				if(socer_mode)
				{
					//tendang=1;
					lap(src, 1);
					bola(drawing, 1);
					//circle(src, Point(170,135 ), 7, Scalar(0,0,0), -1);
					imshow("src", src);
					if(DEBUG_PRINT == true)
					fprintf(stderr, "\r                                                                               \r");
					/*if(noball){
					//=======================speed movemnt ball scan=================
				
					scan_ball_speed=3;//3;
					motion_cariball();	 
					}*/

					if (noball && tt == 1&& tendang==1)
					{
						tambah=0;
						lurus++;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7;
					//cout<<"cariiii"<<lihat << endl;
						habis_nendang();
					}
					else if(noball && tendang==1)
					{
						//=======================speed movemnt ball scan=================
						tambah=0;
						lurus++;
						Walking::GetInstance()->PERIOD_TIME=600;//650;
						//Walking::GetInstance()->A_MOVE_AMPLITUDE = -4;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_OFFSET=23;
						Walking::GetInstance()->Z_OFFSET=35;
						Walking::GetInstance()->A_OFFSET=0.0;
						Walking::GetInstance()->HIP_PITCH_OFFSET=30.4;
						Walking::GetInstance()->Z_MOVE_AMPLITUDE= 43;
						Walking::GetInstance()->BALANCE_KNEE_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.02;
						Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.02;
						//cout<<"cariiii"<<lihat << endl;
						orientasi = MotionStatus::kompas;
						//cout << "orientasi = " << orientasi << endl; 
						habis_nendang();	 
					} 
					if (noball && tt == 1&& tendang==2)
					{
						tambah=0;
						//lurus++;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7;
					//cout<<"cariiii"<<lihat << endl;
						caribola();
						
					}
					else if(noball && tendang==2)
					{
						//=======================speed movemnt ball scan=================
						tambah=0;
						lurus++;
						Walking::GetInstance()->PERIOD_TIME=600;//650;
						//Walking::GetInstance()->A_MOVE_AMPLITUDE = -4;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_OFFSET=23;
						Walking::GetInstance()->Z_OFFSET=35;
						Walking::GetInstance()->A_OFFSET=0.0;
						Walking::GetInstance()->HIP_PITCH_OFFSET=30.4;
						Walking::GetInstance()->Z_MOVE_AMPLITUDE= 43;
						Walking::GetInstance()->BALANCE_KNEE_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.02;
						Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.02;
						//cout<<"cariiii"<<lihat << endl;
						orientasi = MotionStatus::kompas;
						//cout << "orientasi = " << orientasi << endl; 
						caribola();	 
					} 
					//nambah2

					if(noball && tt == 1&& tendang ==0)
					{
						//=======================speed movemnt ball scan=================
						tambah=0;
						lurus++;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7;
						
					} 
					else if(noball && tendang==0)
					{
						//=======================speed movemnt ball scan=================
						tambah=0;
						lurus++;
						//cout << "tt=0="<<lurus << endl;
						
						Walking::GetInstance()->PERIOD_TIME=600;//650;	
						//Walking::GetInstance()->A_MOVE_AMPLITUDE = -4;
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 7; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_OFFSET=23;
						Walking::GetInstance()->Z_OFFSET=35;
						Walking::GetInstance()->A_OFFSET=0.0;
						Walking::GetInstance()->HIP_PITCH_OFFSET=30.4;
						Walking::GetInstance()->Z_MOVE_AMPLITUDE= 43;
						Walking::GetInstance()->BALANCE_KNEE_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.02;
						Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.02;
						Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.02;
						//cout<<"cariiii"  <<lihat<< endl;
						orientasi = MotionStatus::kompas;
						//cout << "orientasi = " << orientasi << endl;
						motion_cariball();	 
					} 
					else if(!noball)
					{
						//=====================atur speed nya disini==============================
						sprinmax_speed=8;//8;
						slow_speed=5;
						belokmax_speed=13;//25;
						belokmid_speed=10;//20;
						belokmin_speed=8;
						lihat++;
						if(lihat==110)lihat=0;
			
						//=======================cek kondisi posisi bola=====================
						//Head::GetInstance()->MoveTracking(ball_position);	 
						pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
						pan_range = Head::GetInstance()->GetLeftLimitAngle();
						pan_percent = pan / pan_range;
			
						tilt=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
						tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
						tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
						tilt_percent = (tilt - tilt_min) / tilt_range;
						if(tilt_percent < 0)
							tilt_percent = -tilt_percent;
					//cout<<"sudah\n"<<endl;
						//cout<<"pan & tilt = "<< tilt << " = " << pan << " " <<endl;

						if(pan > -50 && pan < 50 )
						{
							if (tilt >=25 && (pan <=15 && pan >=-15))
							{
								tahan=0;
								Walking::GetInstance()->P_OFFSET=-4.1;
								Walking::GetInstance()->HIP_PITCH_OFFSET=31.5;//30.4;
								
								if (kaki_kiri >1420 && kaki_kiri <1800 )
								{
									Walking::GetInstance()->X_MOVE_AMPLITUDE =jalan;//27;
									//cout << "sampai sini"<<endl;
									
									tambah++;
									if (tambah >=8)
										{
											cout << "jalan cepat"<<endl;
											jalan = jalan +5;
											if (jalan >=28)
												{

													jalan=28;

												} 
											Walking::GetInstance()->X_MOVE_AMPLITUDE =jalan;//27;
cout<<"masuk coiii 28"<<endl;	
										}
								}
								if((kaki_kiri>=1800)&&(kaki_kiri<=2600))
								{
										
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 10;
Walking::GetInstance()->P_OFFSET=-4.1;
Walking::GetInstance()->HIP_PITCH_OFFSET=32.0;//30.4;
tambah=0;

									//usleep(990000);
								}
								if(kaki_kiri <=1420)
								{
									
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 7 ;
										//Walking::GetInstance()->P_OFFSET=-3.0;
										Walking::GetInstance()->HIP_PITCH_OFFSET=30.0;//30.4;
										tambah=0;
										jalan=10;
										
									cout << "jalan ="<< jalan <<endl;
									//usleep(990000);
								}
								
								//Walking::GetInstance()->X_MOVE_AMPLITUDE =Walking::GetInstance()->X_MOVE_AMPLITUDE +24;

								//cout << "masuk coiiii" <<endl;
								//cout << "pan=" << pan <<endl;
							}
							if (tilt >=25 && (pan >15 && pan <-15))
							{
								//tahan=0;
								tambah=0;
								Walking::GetInstance()->HIP_PITCH_OFFSET=30.0;
								Walking::GetInstance()->X_MOVE_AMPLITUDE=6;//27;
								//Walking::GetInstance()->X_MOVE_AMPLITUDE =Walking::GetInstance()->X_MOVE_AMPLITUDE +24;
								//cout << "pan=" << pan <<endl;
							}
							
							//if (tilt > 1 && tilt <=24 )//&& muter==0)
							if (tilt <=27 && tilt >24 )
							{
								tahan=0;
								tambah=0;
								//cout << "masuk coiiii66" <<endl;
								//Walking::GetInstance()->Stop();
								//usleep(1000);
								Walking::GetInstance()->P_OFFSET=-2.8;
								Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
								
								Walking::GetInstance()->X_MOVE_AMPLITUDE =15;
								//shoot_to_goals(kh, x);
								//Head::GetInstance()->MoveByAngle(0,40);
								//muter=1;
								if(kaki_kiri <=1420)
								{
									tambah=0;	
									
									Walking::GetInstance()->P_OFFSET=-1.0;
									Walking::GetInstance()->HIP_PITCH_OFFSET=29.0;//30.4;
									Walking::GetInstance()->X_MOVE_AMPLITUDE = 1 ;
									//usleep(990000);
								}
							}
							//if (tilt >=-2 &&tilt <=24 )
							if (tilt <=24 )
							{
								tahan=0;
								tambah=0;
								//cout << "masuk coiiii66" <<endl;
								//Walking::GetInstance()->Stop();
								//usleep(1000);
								Walking::GetInstance()->P_OFFSET=-2.8;
								Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
								Walking::GetInstance()->A_MOVE_AMPLITUDE =0;
								Walking::GetInstance()->X_MOVE_AMPLITUDE =6;
								if(kaki_kiri <=1420)
								{
									tambah=0;
									
Walking::GetInstance()->P_OFFSET=-1.0;
Walking::GetInstance()->HIP_PITCH_OFFSET=29.0;//30.4;
Walking::GetInstance()->X_MOVE_AMPLITUDE = 1 ;
									//usleep(990000);
								}
								//shoot_to_goals(kh, x);
								//Head::GetInstance()->MoveByAngle(0,40);
								//muter=1;
							}
					
							//cout<<"sudah masuk = "<<tilt <<endl;//<< tilt << " = " << pan  <<endl;
							//cout << "pan=" << pan <<endl;
							/*if (tilt > -2 && tilt <=1)
							{
								shoot_to_goals(kh, x);
							}*/
							//if (tilt >-2) {keluar=0;}
							
							if(tilt <=-2)
							//if (tilt <=20 )//&& (pan <=10 && pan >=-10))
							{
								//keluar=1;
								//shoot_to_goals(kh, x);
								//cout<<"sudah masuk lagi = "<< m_KickTopAngle << "y=" << y <<endl;
								if(y >= 50)
								{
									//keluar=1;
									Walking::GetInstance()->X_MOVE_AMPLITUDE =2;
									
									//cout<<"sud11111111111"<<endl;
									m_GoalFBStep = 0;
									m_GoalRLTurn = 0;
									///========================didepan kaki atau tidak====================
									//if(tilt<=-2 && (pan>=-50 && pan<=50) && y >= 90) //kanan
									if (tilt<= -2 && pan > 20)
											{
											Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
											Walking::GetInstance()->P_OFFSET=-2.7;
											Walking::GetInstance()->X_MOVE_AMPLITUDE = -5;
											Walking::GetInstance()->A_MOVE_AMPLITUDE = -12;
											Walking::GetInstance()->Start();
											//cout<<"Mundur belok kiri"<<endl;
											}
									else if (tilt<=-2 && pan < -20)
											{
											Walking::GetInstance()->HIP_PITCH_OFFSET=28.0;
											Walking::GetInstance()->P_OFFSET=-2.7;
											Walking::GetInstance()->X_MOVE_AMPLITUDE = -5;
											Walking::GetInstance()->A_MOVE_AMPLITUDE = 12;
											Walking::GetInstance()->Start();
											//cout<<"Mundur belok kanan"<<endl;
											}

									//if(tilt<=-2 && (pan>=-20 && pan<=20) && y >= 90) //kanan
									if (tilt <=-2 && (pan>=-20 && pan<=20) && y >= 90)
									{
										//keluar=1;
										
											shoot_to_goals(kh, x);
										if( kh == 1 ){ww = 1;kh = 0;}
										//cout<< "tendang >> y=" << y <<endl;	
										if(ww == 1)
										{
											//if( x >= 170)//85) //kanan
											if( pan <= 0)//85)
											{
												cout<<"Tendang kanan " <<endl;
												
												Walking::GetInstance()->Stop();
												MotionManager::GetInstance()->StopLogging();
												Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
												Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
												Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;//-3;*/ 
usleep(300);
												
												//Action::GetInstance()->Start();
												motion(202);//(200);//(155);kiri=200, kanan=202
												
												//muter=0;
												Walking::GetInstance()->X_OFFSET=-3;
												Walking::GetInstance()->Y_OFFSET=23;	
												Walking::GetInstance()->Z_OFFSET=35;
												Walking::GetInstance()->A_OFFSET=0.0;
Walking::GetInstance()->P_OFFSET=-4.1;
												Walking::GetInstance()->HIP_PITCH_OFFSET=31.4;//30.4;
												Walking::GetInstance()->Z_MOVE_AMPLITUDE= 43;
												Walking::GetInstance()->BALANCE_KNEE_GAIN=0.02;
												Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.02;
												Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.02;
												Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.02;
usleep(300);
												tendang =1;
												kembali=0;
												tambah=0;
		
                            			
												cout << "x && y" << x << " && " << y << endl;
												//cari_ball=false;
											}
											//else if(x <170) //kiri
											else if(pan >0)
											{
												cout<<"Tendang kiri " <<endl;
												
												Walking::GetInstance()->Stop();
												MotionManager::GetInstance()->StopLogging();
												Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
												Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
												Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;//-3;*/
usleep(300);
												
												//Action::GetInstance()->Start(200);
												motion(200);//(168);kiri=200, kanan=202
												//muter=0;
												Walking::GetInstance()->Y_OFFSET=23;
												Walking::GetInstance()->Z_OFFSET=35;
												Walking::GetInstance()->A_OFFSET=0.0;
Walking::GetInstance()->P_OFFSET=-4.1;
												Walking::GetInstance()->HIP_PITCH_OFFSET=31.4;//30.4;
												Walking::GetInstance()->Z_MOVE_AMPLITUDE= 43;//38;
												Walking::GetInstance()->BALANCE_KNEE_GAIN=0.02;
												Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.02;
												Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.02;
												Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.02;
usleep(300);
												
												tendang =1;
												kembali=0;
												tambah=0;
		
												
											}
											ww = 0;
										}
									}
										//========================shoot to target=========================/
										/*		if(cari_ball){
										followball_engine();
										}else if(!cari_ball)
										{ 
											shoot_to_goals();
										}	*/
								}
								else 
								{
									m_KickBallCount = 0;
									KickBall = 0;
									m_GoalFBStep = m_FitFBStep;
									m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
									//if(DEBUG_PRINT == true)
								//mode_rotasi=50;
									fprintf(stderr, "KEJAR(P:%.2f T:%.2f>%.2f C:%3d X:%3d)]", pan, y, m_KickTopAngle,orientasi,x);
								}
							}
							else
							{
								cm730.ReadWord(1, MX28::P_PRESENT_POSITION_L, &kaki_kiri, 0);
								m_KickBallCount = 0;
								KickBall = 0;
								m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
								if(m_GoalFBStep < m_FollowMinFBStep)
									m_GoalFBStep = m_FollowMinFBStep;
								MotionManager::GetInstance()->StartLogging();
								Walking::GetInstance()->Start();
								
								//m_GoalFBStep = a ;///*m_FollowMaxFBStep*/ * tilt_percent;
								m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
								m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
								//cout << "sini gan" <<endl;
								
								//Walking::GetInstance()->X_MOVE_AMPLITUDE = 3 ;
								//if(DEBUG_PRINT == true)
								fprintf(stderr, "[LARI(P:%.2f T:%.2f>%.2f C:%3d TANG:%3d X:%3d]", pan, tilt, tilt_min, orientasi,kaki_kiri,x);
							}
						}
						else
						{
							m_KickBallCount = 0;
							KickBall = 0;
							m_GoalFBStep = 0;
							m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
							//if(DEBUG_PRINT == true)
							fprintf(stderr, "[FOLLOW bola 1:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, orientasi);
								}		
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
						Walking::GetInstance()->A_MOVE_AIM_ON = true;
						if(DEBUG_PRINT == true)
						fprintf(stderr, " START");

						if(Walking::GetInstance()->IsRunning() == false)
						{
							m_FBStep = 0;
							m_RLTurn = 0;
							m_KickBallCount = 0;
							KickBall = 0;
							//komen1
							/*if (tilt >=25 && (pan <=10 && pan >=-10))
							{
								Walking::GetInstance()->X_MOVE_AMPLITUDE =3;//27;
								Walking::GetInstance()->X_MOVE_AMPLITUDE =Walking::GetInstance()->X_MOVE_AMPLITUDE +24;

								cout << "masuk coiiii" <<endl;
								cout << "pan=" << pan <<endl;
							}

							if (tilt <=24&& (pan <=10 && pan >=-10))
							{
								cout << "masuk coiiii66" <<endl;
								Walking::GetInstance()->X_MOVE_AMPLITUDE =3;
							}*/
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
							//cout <<"masuk sini juga" <<endl;
							//Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
							//Walking::GetInstance()->X_MOVE_AMPLITUDE =3;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = -1*m_RLTurn;
							Walking::GetInstance()->Start();			
						}
						else
						{
							if(m_FBStep < m_GoalFBStep)
								m_FBStep += m_UnitFBStep;
							else if(m_FBStep > m_GoalFBStep)
								m_FBStep = m_GoalFBStep;
							//komen2
							

							//Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
							//cout <<"sini ding" <<endl;

							if(m_RLTurn < m_GoalRLTurn)
								m_RLTurn += m_UnitRLTurn;
							else if(m_RLTurn > m_GoalRLTurn)
								m_RLTurn -= m_UnitRLTurn;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = -1*m_RLTurn;

							if(DEBUG_PRINT == true)
							fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
						}
					}
				}
			x = -5;
			y = -5;
			//	*/

			if(waitKey(30)== 27){trg = 1;break;}    
			}
			if (trg == 1)break;
		}
		if (trg == 1)break;
	}
	return 0;
}	
