
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



#include <iostream>
#include "Tserial.h"


using namespace std;
using namespace cv;



void detectAndDisplay( Mat frame );


String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection - Remixed by TechBitar";


  int arduino_command;
  Tserial *arduino_com;
  short MSBLSB = 0;
  unsigned char MSB = 0;
  unsigned char LSB = 0;

  unsigned char faceDetectB1 = 1;
  unsigned char faceDetectB2 = 1;


int main( int argc, const char** argv )
{
  //CvCapture* capture;
  //Mat frame;
 Mat frame;
 Mat clearFrame;
String url = "http://admin:3558242111@192.168.1.1:61323/mjpeg.cgi?user=admin&=3558242111&channel=0&.mjpg";

   
  arduino_com = new Tserial();
  if (arduino_com!=0) {
       arduino_com->connect("COM7", 57600, spNONE); } 
  
  
  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
  if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
 
  //-- 2. Read the video stream
 

  VideoCapture cap(url);
   //capture = cvCaptureFromCAM( 3 );
   for(int i=0; i<3; i++) { cap >> clearFrame; }
//cap.set(CV_CAP_PROP_FRAME_COUNT, 13);
    while( true )
    {
		
     //frame = cvQueryFrame( capture );
		
	     cap >> frame;
		
     
      if( !frame.empty() )
       { detectAndDisplay( frame ); }
      else
       { printf(" --(!) No captured frame -- Break!"); break; }
      
      int c = waitKey(10);
      if( (char)c == 'c' ) { break; } 
    }

  

     arduino_com->disconnect();
     delete arduino_com;
     arduino_com = 0;

  return 0;
}


void detectAndDisplay( Mat frame )
{
 
   std::vector<Rect> faces;
   Mat frame_gray;

   cvtColor( frame, frame_gray, CV_BGR2GRAY );
   equalizeHist( frame_gray, frame_gray );
   //-- Detect faces
   face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

   for( int i = 0; i < faces.size(); i++ )
    {
      Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
      ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
	 cout << "X:" << faces[i].x  <<  "  y:" << faces[i].y  << endl;

		

    
     // send X axis
	
     LSB = faces[i].x & 0xff;
     MSB = (faces[i].x >> 8) & 0xff;
	 arduino_com->sendChar( MSB );
	 arduino_com->sendChar( LSB );


    // Send Y axis
	LSB = faces[i].y & 0xff;
	MSB = (faces[i].y >> 8) & 0xff;
	arduino_com->sendChar( MSB );
	arduino_com->sendChar( LSB );


      Mat faceROI = frame_gray( faces[i] );
      std::vector<Rect> eyes;
    } 

   //-- Show what you got
  
   
   imshow( window_name, frame );
   

}
