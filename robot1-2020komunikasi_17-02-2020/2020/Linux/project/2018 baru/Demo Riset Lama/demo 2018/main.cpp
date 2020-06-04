/*
 * main.cpp
 *  RSCUAD ROBOT AHMAD DAHLAN
 *  Created on: 2017. 24. 4.
 *  Revision : 2018. 16. 03
 *      Author: robotis, Jihad Rahmawan, Syahid Al Irfan
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
using namespace std;


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

static const double VIEW_V_ANGLE = 46.0; //degree
static const double VIEW_H_ANGLE = 58.0; //degree

int last=0;
struct sockaddr_in myaddr;	/* our address */
	struct sockaddr_in remaddr;	/* remote address */
	socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
	int recvlen;			/* # bytes received */
	int fd;				/* our socket */
	unsigned char buf[BUFSIZE];	/* receive buffer */

int initreferee(){
/* create a UDP socket */

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
	}

	/* bind the socket to any valid IP address and a specific port */

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

	
	//////////////////////////////////////////////////////////////////////////////////
using namespace cv;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);
		
Mat src; Mat src_gray;
 int th2 = 255, th1 =8, ind = 0;
 RNG rng(12345);
 
//========================= variabel lapangan ===================================//
 
 int h_min = 34, h_max = 87, s_min = 37, s_max = 255, v_min = 57, v_max = 255,conture = 100;
 Mat drawing, drawing2;
//========================= variabel bola ===================================//

int h_min2 = 199, h_max2 = 255, s_min2 = 216, s_max2 = 255, v_min2 = 170, v_max2 = 255;
int conture2 = 25,aconture2 = 153,th2_max = 255, ratio=3; 
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
int tgh;
//==================================//





bool recap = true;
bool useMorphOps = true;
bool noball;
int noballcount;
int max_nobalcount;


Point2D ball_position;
Point2D pos;


int pan;
int tilt;
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
                        Action::GetInstance()->Start(indek);
                        while(Action::GetInstance()->IsRunning());
						usleep(8*1000);
						//tilt=0;
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
						Walking::GetInstance()->Stop();
						
						Head::GetInstance()->MoveToHome();
						usleep(8*1000);
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
//		createTrackbar("HoughR", "Trackbar Bola", &ratio, 20);
	moveWindow("Trackbar Bola", 700, 0);
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
        drawContours( src, hull, i, Scalar(11,0,0), 2, 8, vector<Vec4i>(), 0, Point(0,10));
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







 void bola(Mat vdrawing2, int es)
 {
	       //=========== tmbhn =====================//
      	
	for (int i = 0; i < vdrawing2.cols; i++) {
    for (int j = 0; j < vdrawing2.rows; j++) {

		int hj =  vdrawing2.at<uchar>(j, i);
		if (hj > 0)
		{
		vdrawing2.at<uchar>(j, i) = 0;
		}
		else
		{
		vdrawing2.at<uchar>(j, i) = 255;
		}
	  //  cout<<pixValue<<endl;
        
     }
}
	Mat mask, mask2, agbl;
	bitwise_and(src,src,mask,vdrawing2 = vdrawing2);
	
		inRange(mask,Scalar(h_min2,s_min2,v_min2),Scalar(h_max2, s_max2, v_max2),mask);
		erode( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(10, 40)) );
		imshow("gmaska",mask);
		bitwise_and(mask,mask,agbl,vdrawing2 = vdrawing2);
		agbl = mask + vdrawing2;
		imshow("agbl",agbl);
		
		for (int i = 0; i < agbl.cols; i++) {
		for (int j = 0; j < agbl.rows; j++) {

		int hj =  agbl.at<uchar>(j, i);
		if (hj > 0)
		{
		agbl.at<uchar>(j, i) = 0;
		}
		else
		{
		agbl.at<uchar>(j, i) = 255;
		}
	  //  cout<<pixValue<<endl;
        
     }
}
	//===========================================================//
	 Mat smask, bola;
	// x = -5;y = -5;	  
	//sgbl = gbl;
	bitwise_and(src,src,smask,agbl = agbl);
	imshow("Batas",smask);
	inRange(smask,Scalar(h_min2,s_min2,v_min2),Scalar(h_max2, s_max2, v_max2),mask2);
	erode( mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
	dilate( mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );
	threshold(mask2,mn,th1,th2,THRESH_BINARY+THRESH_OTSU);
	bitwise_and(src,src,bola,mn = mn);
	imshow("Bolaaa",bola);
	

   vector<vector<Point> > contours2;
   vector<Vec4i> hierarchy2;
   // mencari controur bola
   findContours( mask2, contours2, hierarchy2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
   vector<vector<Point> > contours_poly2( contours2.size() );
   vector<Rect> boundRect2( contours2.size() );
   vector<Point2f>center( contours2.size() );
   vector<float>radius( contours2.size() );
   
//=============== koordinat bola =====================//

for( int i = 0; i < contours2.size(); i++ )
     { approxPolyDP( Mat(contours2[i]), contours_poly2[i], 3, true );
       boundRect2[i] = boundingRect( Mat(contours_poly2[i]) );
       minEnclosingCircle( (Mat)contours_poly2[i], center[i], radius[i] );
     }

//======================================================//

 
	  
for( int p = 0; p< contours2.size(); p++ )
{
//line( src, center[p], Point(center[p].x+40, center[p].y-50), Scalar(0,54,255), 2, CV_AA);
//line( src, Point(center[p].x+40, center[p].y-50),Point(center[p].x+90, center[p].y-50), Scalar(255,255,255), 2, CV_AA);

for(size_t j=0;j<contours2.size();j++)  
                {
            	  Point center2(cvRound(center[j].x),cvRound(center[j].y));
                  //cx = cvRound(circles[j][0]);
                  //cy = cvRound(circles[j][1]);
            	  circle( src, center2, 20, Scalar(255, 255, 255), 2, CV_AA  );
		 // op = center.x;
		  //ip = center.y;
        	}

if (contourArea(contours2[p]) > conture2*20 && contourArea(contours2[p]) < aconture2*50)
       {
	x = center[p].x;
	y = center[p].y;
       }
	
 } 

 
	circle( src, Point(x,y), 9, Scalar(40, 80, 255), CV_FILLED, CV_AA  );
	line( src, Point(x,y), Point(x+40, y-50), Scalar(0,54,255), 2, CV_AA);
	line( src, Point(x+40, y-50),Point(x+90, y-50), Scalar(255,255,255), 2, CV_AA);
	putText(src, "Bola", Point(x+40, y-30), FONT_HERSHEY_COMPLEX_SMALL, 1,  Scalar(0,0,255,255));
//*/
// cout << "=============================" << endl;
 							
							if(( x <= 0 && y <=0)) {
							ball_position.X = -1;
							ball_position.Y = -1;
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
							Point2D center = Point2D(85, 115);
							Point2D offset = pos - center;

							offset *= -1; // Inverse X-axis, Y-axis
							offset.X *= ((VIEW_H_ANGLE / 310)); // pixel per angle
							offset.Y *= ((VIEW_V_ANGLE / 230)); // pixel per angle
							ball_position = offset;

 cout << "aktif " << pos.X << " ====== " << pos.Y << endl;
							Head::GetInstance()->MoveTracking(ball_position);
 cout << "aktif lagi " << offset.X << " ---- " << ball_position.X << endl;
							noball=false;
							}
 
 }



void motion_cariball(){    
cout << td << " td---ty " << ty << endl;
	                       			if(td>0)
						   {
						   if((ty>=-95)&&(ty<=75))
							{ty += 5;}
						   if(ty==75)
							{ty=-95; td+=20;}
						   if(td>110)
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
					        	 if(((ty>=-95))&&(ty<=75))
								{ty += 5;}
						     	 if(ty==35)
								{ty=-60; td += 15;}
						   }

						        Walking::GetInstance()->PERIOD_TIME=650;
		                        Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
								if(lagi<=1)
								{
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
								Walking::GetInstance()->Start();
								Walking::GetInstance()->X_MOVE_AMPLITUDE = scan_ball_speed;;
								Walking::GetInstance()->Start();
								
							    }
								if(lagi>1)
							{
								Walking::GetInstance()->A_MOVE_AMPLITUDE = 8;
								Walking::GetInstance()->Start();
								Walking::GetInstance()->X_MOVE_AMPLITUDE = -1;
								Walking::GetInstance()->Start();	
							}
					Head::GetInstance()->MoveByAngle(ty,td);
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
// cout << pan << " p---t " << tilt << endl;
			
				
	               
}
 void followball_engine(){
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
					Walking::GetInstance()->X_MOVE_AMPLITUDE = sprinmax_speed; 
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
	}*/
 }
void head_trackonly(){
 Head::GetInstance()->MoveTracking(ball_position);	 
 pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
 //tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);	
 printf("tilt: %d \t pan: %d \r", tilt, pan);
}			 	

 void geser_bola(int mode_ges){
	  
 }
	
void getup_auto(){
	  s=MotionStatus::FB_ACCEL;
cout << "satu" << s << endl;
	 if((s>150 && s<350) || s>670){
	//	 k=1;
	   } else k=0;
	  if(k==1){
		  MotionManager::GetInstance()->SetEnable(false);
		  sleep(1);
		  last_acc=MotionStatus::FB_ACCEL;
		  if((last_acc>150 && last_acc<380)){
cout << "dpn " << last_acc << endl;
			MotionManager::GetInstance()->SetEnable(true);
						motion(9);
						motion(174);
		  } 
		  if(last_acc>480){
cout << "blkng " << last_acc << endl;
			  MotionManager::GetInstance()->SetEnable(true);
			    		        motion(9);
						motion(171);
			  
		  }
		  
	  }
}
	

 void shoot_to_goals(){
			   orientasi = MotionStatus::kompas;
			   if((orientasi>=0 && orientasi<=50) || (orientasi>=100&& orientasi<=150)){
			//	 mode_rotasi=1;  
			   } else if(orientasi>50 && orientasi<=75){ 
				 mode_rotasi=1; 
			   } else if(orientasi>75 && orientasi<100){ 
				 mode_rotasi=1;
			   }
			  cout << orientasi << " kmps" << endl; 
			   //=======================motion keliling bola======================
					if( mode_rotasi==1){
						Walking::GetInstance()->PERIOD_TIME=600; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_MOVE_AMPLITUDE= 70;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->A_MOVE_AMPLITUDE = -20;
						Walking::GetInstance()->Start();
					}
					else if( mode_rotasi==2)
					{
						Walking::GetInstance()->PERIOD_TIME=600; 
						Walking::GetInstance()->Start();
						Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->Y_MOVE_AMPLITUDE= -70;
						Walking::GetInstance()->Start();
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 20;
						Walking::GetInstance()->Start();
					}
					else if(mode_rotasi==0)
					{
						
				/*		gaw(drawing2, ssg);
						ball_position.X = gw.x;
						ball_position.Y = gw.y;
						Walking::GetInstance()->Stop();
						usleep(8*1000);
				*/		  //  Head::GetInstance()->MoveTracking(ball_position);	 
		                    pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			                tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
							
							Walking::GetInstance()->PERIOD_TIME=650;
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			               
							if (tilt>0){
								Walking::GetInstance()->X_MOVE_AMPLITUDE = 4; 
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
								//motion(12);
								}
							else if(pan>-10 && pan<=-5){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmin_speed*-1);
								Walking::GetInstance()->Start();
								}
							else if(pan>-30 && pan<=-10){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmid_speed*-1);
								Walking::GetInstance()->Start();
								} 
							else if(pan<=30){
								Walking::GetInstance()->A_MOVE_AMPLITUDE =(belokmax_speed*-1);
								Walking::GetInstance()->Start(); 
								}
							}
							else if(tilt<=-4)
							{//*	
							Walking::GetInstance()->X_MOVE_AMPLITUDE = 2; 
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
								//*/
							}
					}

}

int main(int argc, char* argv[])//void)
{
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
    VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,310);//420);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,230);//340);
   
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
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    Action::GetInstance()->Start(9);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//=====================================================================
//               PROGRAM UTAMA 
//====================================================================

    socer_mode=true;
	if (socer_mode=true)
	{
/*	
referee ();
				while(referee()!=3){
				printf(" %d \n", referee());
				 Walking::GetInstance()->Stop();
				}
//	*/
    while(1)	
    { 

				getup_auto();		
		orientasi = MotionStatus::kompas;
		//cout << pan << "]]]]]]" << tilt << endl;
		//drawing = Mat::zeros( src.size(), CV_8UC1 );
        //ssg = Mat::zeros(src.size(), CV_8UC1);
		//vb  = Point(-6, -6); ff = Point(-6, -6);
		//int x = -5;y = -5;	  
	    max_nobalcount=5; 
		cap.read(src);
	    
		if(socer_mode){
			lap(src, 1);
		bola(drawing, 1);
	circle(src, Point(170,135 ), 7, Scalar(0,0,0), -1);
		imshow("src", src);
			if(noball){
				//=======================speed movemnt ball scan=================
				
				scan_ball_speed=3;
				motion_cariball();	 
			} 
			else if(!noball)
			{
			//=====================atur speed nya disini==============================
			sprinmax_speed=8;
			slow_speed=4;
			belokmax_speed=25;
			belokmid_speed=20;
			belokmin_speed=8;
			
			//=======================cek kondisi posisi bola=====================
			//Head::GetInstance()->MoveTracking(ball_position);	 
            pan=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			tilt=MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
			///========================didepan kaki atau tidak====================
					if(tilt<=-5 && (pan>=-50 && pan<=50) && y >= 100)
					{
						
cari_ball=false;
					}
					else
					{
						cari_ball=true;
					}
			 //========================shoot to target=========================/
						if(cari_ball == true){
						followball_engine();
						}else
						{ 
							shoot_to_goals();
						if(ind == 1)
							{
							motion(9);
							motion(155);
							}

						}
			}
		}
		
	x = -5;
	y = -5;
//	*/

	if(waitKey(30)== 27)break;
    
}
}
return 0;
}	

