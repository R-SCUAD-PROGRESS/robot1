//=============== Vision Bola, Gawang dan Lapangan ======================//
//========================= Syahid Al Irfan ====================================//
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


 using namespace cv;
 using namespace std;
 
 Mat src; Mat src_gray;
 int th2 = 255, th1 =8;
 RNG rng(12345);
 
//========================= variabel lapangan ===================================//
 
 int h_min = 34, h_max = 45, s_min = 80, s_max = 255, v_min = 75, v_max = 255,conture = 100;
 Mat drawing, drawing2;
//========================= variabel bola ===================================//

int h_min2 = 199, h_max2 = 255, s_min2 = 216, s_max2 = 255, v_min2 = 218, v_max2 = 255;
int conture2 = 15,aconture2 = 153,th2_max = 255, ratio=3; 
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
double k = 0.04;

Point vb  = Point(-6, -6), ff = Point(400, -6);
Mat ssg, sg;
Mat dst, cdst, ual, ul;
Point gw = Point(0,0);
//==================================//

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
   findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

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
        drawContours(drawing, hull, i, Scalar(255, 255, 255), CV_FILLED,  8, vector<Vec4i>(), 0, Point(0,10));	
		drawContours( sg, hull, i, Scalar(255,255,255),  CV_FILLED, 8, vector<Vec4i>(), 0, Point(0,-50) );
		drawContours(drawing2, hull, i, Scalar(255, 255, 255), CV_FILLED); 	
    }
      }
		
		bitwise_and(src_copy,src_copy,ssg,sg = sg);
		imshow("ssg", ssg);
//		imshow("sg", sg);
		circle(src, Point(150,190 ), 3, Scalar(23,35,255), -1);
}
		





void gaw(Mat sdrawing2, Mat ssrcc)
{
	
	//==================================================================================== pemilihan metode ====================================================================//
	  
	/*  int uo =  threshold_output.at<uchar>(120,150 );// batas bounding rect kanan
	  int uos =  threshold_output.at<uchar>(120,25 );// batas bounding rect kiri
	  int tgh =  threshold_output.at<uchar>(190,150 );// batas bounding rect kiri
		//circle(src, Point(150,170 ), 3, Scalar(23,35,255), -1);
		
//*		
		if (tgh > 0)/// kondisi metode boundingrect
		{
			circle(src, Point(10,10 ), 10, Scalar(23,35,255), -1);
//	/	
		
		
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

  findContours( ual, contours3, hierarchy3, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

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
		{*/
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
//		imshow("gmask",gmask);
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


cornerHarris( ze, gh, blockSize, apertureSize, k, BORDER_DEFAULT );

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
//   imshow( "zeros", ze );
		
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
		//erode( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		//dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(10, 90)) );
		//imshow("gmaska",mask);
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
	 x = -5;y = -5;	  
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
   findContours( mask2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
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
 }
 
 
 
 
 
 
 
 
int main()
 {
VideoCapture cap(0); 

	if(!cap.isOpened())
	{
			return -1;
	 }
	//namedWindow("Trackbar", CV_WINDOW_NORMAL);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);

	
	
	
while(1)
{
	//Mat ssg, sg;
	ssg = Mat::zeros(src.size(), CV_8UC1);
	vb  = Point(-6, -6); ff = Point(-6, -6);
	int x = -5;y = -5;	  
	tblap();
	tbbol();
	tbgaw();
	///.
	cap.read(src);
		lap(src, 1);
		//imshow( "drawing2drawing2", ssg );
		gaw(drawing2, ssg);
		   
		bola(drawing, 1);
      cout<<x<<"==="<<y<<endl;
	  cout<<ff.x<<"==="<<ff.y<<"]]]]]"<<ff.x<<"==="<<ff.y<<endl;
	  circle( src, gw, 9,  Scalar(255,255,255), CV_FILLED, 8, 0 );

// moveWindow("garis", 1110, 50);
 //moveWindow("cari", 10, 50);
 //moveWindow("Batas", 10, 1150);
 
  
//   imshow( "cari", threshold_output );
   imshow( "Source", src );
   //moveWindow("Bola", 370, 450);
   
 //moveWindow("detected lines", 10, 50);
 //moveWindow("Source", 10, 50);
 
 //imshow("detected lines", drawing);
 
   
   if(waitKey(30)== 27)break;
   }
   return 0;
 }
