
//////////////////////////////////////////////////////
//Raspian version of Red & Green Light identification 
//Range of RED: 170~180
//Range of GREEN: 38~75
/////////////////////////////////////////////////////
#include <wiringPi.h>
#include <stdio.h>
#include "opencv2/opencv.hpp" 
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include "linefinder.h"
#include "PCA9685.h"
#include "Ultrasonic.h"
#include <signal.h>

PCA9685 pca9685;
Ultrasonic ultra;

#define DIR_DISTANCE_ALERT  30
#define ULTRASONIC_TRIG     3
#define ULTRASONIC_ECHO     23
#define PI 3.1415926

static volatile int keepRunning = 1;
using namespace cv;
using namespace st
static unsigned char OUT[] = {5, 0, 1, 2, 3};
static unsigned char IN[] = {21, 22, 26, 23};

void checkObstackle();
void setup();
void intHandler(int dummy);

void setup()
{
	wiringPiSetup();
	int i;
	
	for(i=0; i<sizeof(OUT); i++){
		pinMode(OUT[i], OUTPUT);
		digitalWrite(OUT[i],LOW);
	}
	for (i=0; i<sizeof(IN); i++){
		pinMode(IN[i],INPUT);
	}
	ultra.begin(ULTRASONIC_TRIG,ULTRASONIC_ECHO);
}


void checkObstackle()
{

		// frount right and left
		digitalWrite(OUT[1], HIGH);
		digitalWrite(OUT[0], HIGH);
		delay(100);
		digitalWrite(OUT[1], LOW);
		digitalWrite(OUT[0], LOW);
		delay(100);
		digitalWrite(OUT[1], HIGH);
		digitalWrite(OUT[0], HIGH);	
		delay(100);
		digitalWrite(OUT[1], LOW);
		digitalWrite(OUT[0], LOW);	
		delay(100);
		// rear left and right
		digitalWrite(OUT[3], HIGH);
		digitalWrite(OUT[2], HIGH);
		delay(100);
		digitalWrite(OUT[3], LOW);
		digitalWrite(OUT[2], LOW);
		delay(100);
		digitalWrite(OUT[3], HIGH);
		digitalWrite(OUT[2], HIGH);
		delay(100);
		digitalWrite(OUT[3], LOW);
		digitalWrite(OUT[2], LOW);
		
}


void intHandler(int dummy){
	delay(10);
	keepRunning = 0;
}


int main(int argc, char**argv)
{
	int CountTimes=0;
	int count =0;
			
	setup();
	signal(SIGINT, intHandler);
	  // Set-Up
    int houghVote = 200;
    
  
    string arg = "";
    // Set up windows
    bool showCanny = 1;
    bool showHough = 1;
    bool showHoughP = 1;
	Mat3b image;
	raspicam::RaspiCam_Cv capture;
	capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	capture.open();

	while (keepRunning)
	{
		
		capture.grab();
		capture.retrieve(image);
		 cv::Mat resized;
	  cv::resize(image, resized, cv::Size(320, 240));
	  
	  
	  float disValue = ultra.ReadDistanceCentimeter();
			cout<<"DISTANCE:   "<<disValue<<endl;
		
        int x = 0;
        int y = 170;
        int width = 320;
        int height = 70;

        Rect roi(x,y,width,height);
        Mat bgr = image(roi);
                     
        Scalar val = Scalar(0, 0, 0);
       
        Mat contours;
        Canny(bgr,contours,100,200);
                
        Mat contoursInv;
        threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
                
		std::vector<Vec2f> lines;
        if (houghVote < 1 or lines.size() > 2) { // we lost all lines. reset
            houghVote = 300;
        }
        else{ houghVote += 25;}
        
        while(lines.size() < 4 && houghVote > 0){
            HoughLines(contours,lines,1,PI/180, houghVote);
            houghVote -= 5;
        }
        std::cout << houghVote << "HOUGHVOTE\n";
        Mat result(bgr.size(),CV_8U,Scalar(0));
        Mat hough(bgr.size(),CV_8U,Scalar(0));
        bgr.copyTo(result);

        // Draw the lines
        std::vector<Vec2f>::const_iterator it= lines.begin();     
        
        while (it!=lines.end()) {	
			
            float rho= (*it)[0];   // first element is distance rho
            float theta= (*it)[1]; // second element is angle theta
			float tan;
			
            if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ) { // filter to remove vertical and horizontal lines
				  // point of intersection of the line with first row
                Point pt1(rho/cos(theta),0);
                // point of intersection of the line with last row
                Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
                //std::cout << " rho and theta ("<< rho <<"," << theta << ")\n";
                line( result, pt1, pt2, Scalar(255,255), 1);
                  line( hough, pt1, pt2, Scalar(255), 1);
               
			}
            ++it;
        }

		        // Create LineFinder instance
        LineFinder ld;
		// -------------------DETECT LIGHT ------------------------------------------------
		Mat3b hsv;
		cvtColor(resized, hsv, COLOR_BGR2HSV);

		Mat1b mask1, mask2, mask3;
		inRange(hsv, Scalar(173, 70, 50), Scalar(210, 255, 255), mask1);
		inRange(hsv, Scalar(38, 70, 50), Scalar(75, 255, 255), mask2);
		
		 vector<vector<Point> > reddd;
		vector<Vec4i> hierarchy;
		findContours( mask1, reddd, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		int CountRed=0;
		for(unsigned int i=0;i<reddd.size();i++){

			for(unsigned int j=0;j<reddd[i].size();j++)
			{
			 //cout<< "Point(x,y)="<< reddd[i][j] << endl;
			 CountRed++;
			 
			 }
		}	
		imshow("RED", mask1);	
		cout<<"CountRed:   "<<CountRed<<endl;
		
		
		
		// -------------------DETECT LIGHT ------------------------------------------------
        // Set probabilistic Hough parameters
        ld.setLineLengthAndGap(8,50); // min accepted length and gap
        ld.setMinVote(10); // sit > 3 to get rid of "spiderweb"

        // Detect lines
        std::vector<Vec4i> li= ld.findLines(contours);
        cout<<"\nNumber of Lines:  "<<li.size()<<"\n"<<endl;
        int totalLeft =0;
        int totalRight =0;
        int totalLeft2 = 0;
        int totalRight2 = 0;
        int input2;
        int input;
        int CounterLeft = 0;
        int CounterRight = 0;
        int CounterLeft2 = 0;
        int CounterRight2 = 0;
        /*if the y1 and y2 data of vectors is needed
         * int inputY1=0;
        int inputY2=0;
        int totalLY1 =0;
        int totalRY1 =0;
        int totalLY2 =0;
        int totalRY2 =0;*/
        for (int i = 0; i < li.size(); i++)
			{
					//cout << li[i][0];
					input = li[i][0];
					//inputY1 = li[i][1];
					
					if(input<160){
					totalLeft = totalLeft + input;
					//totalLY1=totalLY1+inputY1;
					CounterLeft++;
					}
					else{totalRight=totalRight + input;
						//totalRY1=totalRY1+inputY1;
						CounterRight++;
						}
					
					input2 = li[i][2];
					//inputY2 = li[i][3];
					
					if(input2<160){
					totalLeft2 = totalLeft2 + input2;
					//totalLY2=totalLY2+inputY2;
					CounterLeft2++;
					}
					else{totalRight2=totalRight2 + input2;
						//totalRY2=totalRY2+inputY2;
						CounterRight2++;
						}
					
			}
			
			int averageLeft = 0;
			int averageLeft2 = 0;
			int averageRight = 0;
			int averageRight2 = 0;
			int forSpeed = 70;
			int bakSpeed = 90;
			int ovlav = 0;
			int ovrav = 0;
			/*int avLY1=0;
			int avLY2=0;
			int avRY1=0;
			int avRY2=0;*/
			
			
			if(CounterLeft!=0)
			averageLeft = totalLeft/CounterLeft;
			//avLY1 = totalLY1/CounterLeft;}
			if(CounterLeft2!=0)
			averageLeft2 = totalLeft2/CounterLeft2;
			//avLY2 = totalLY2/CounterLeft2;}
			if(CounterRight!=0)
			averageRight = totalRight/CounterRight;
			//avRY1 = totalRY1/CounterRight;}
			if(CounterRight2!=0)
			averageRight2 = totalRight2/CounterRight2;
			//avRY2 = totalRY2/CounterRight2;}
			ovlav = (averageLeft + averageLeft2)/2;
			ovrav = (averageRight + averageRight2)/2;
		
			cout<<"ovleftX1:   "<<averageLeft<<endl;
			cout<<"ovleftX2:   "<<averageLeft2<<endl;
			cout<<"ovrightX1:   "<<averageRight<<endl;
			cout<<"ovrightX2:   "<<averageRight2<<endl;
			
			/*cout<<"ovleftY1:   "<<avLY1<<endl;
			cout<<"ovleftY2:   "<<avLY2<<endl;
			cout<<"ovrightY1:   "<<avRY1<<endl;
			cout<<"ovrightY2:   "<<avRY2<<endl;*/
			
			cout<<"ovlav:   "<<ovlav<<endl;
			cout<<"ovrav:   "<<ovrav<<endl;
			
			int right=890, left=1350, forward=1950;
			if(CountRed == 0){
			/*if(count==0)
			{
				pca9685.setModSpeed(80,80);
				pca9685.goForward();
				delay(forward);
			s	pca9685.setModSpeed(100,10);
				pca9685.goRight();
				delay(right);
				count++;
				cout<<"asasdaasdgasdfafasfaSFASDFASGASGSFDGASGDsdfgsdfgadfga"<<endl;
			}
			if(CountTimes==0)
			{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					{	
					pca9685.stop();
					delay(20);
					checkObstackle();
					pca9685.setModSpeed(100,10);
					pca9685.goRight();
					delay(right);
					pca9685.setModSpeed(80,80);
					pca9685.goForward();
					delay(100);
					
					pca9685.setModSpeed(10,100);
					pca9685.goLeft();
					delay(left);
					pca9685.setModSpeed(80,80);
					pca9685.goForward();
					delay(forward);
					pca9685.setModSpeed(10,100);
					pca9685.goLeft();
					delay(left);
					pca9685.setModSpeed(80,80);
					pca9685.goForward();
					delay(500);
					
					pca9685.setModSpeed(100,10);
					pca9685.goRight();
					delay(1000);
					CountTimes++;
				}
			}*/
			if(ovlav!=0 && ovrav != 0)
			{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					{	
						   cout<<"2"<<endl;					
							pca9685.stop();
							checkObstackle();
						}
				else{
					
				cout<<"Forward Both"<<endl;
				forSpeed = 60;
				bakSpeed = 60;
				pca9685.setModSpeed(forSpeed, bakSpeed);
				pca9685.stop();
				delay(80);
				pca9685.goForward();
				
			}
			
				}	
			else if(ovlav!=0 && ovrav == 0)
			{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					   {
						   cout<<"1"<<endl;
							pca9685.stop();
							checkObstackle();
						}
				else{
				cout<<"RIGHT"<<endl;
				pca9685.setModSpeed(90, 12);
				pca9685.stop();
				delay(25);
				pca9685.goRight();
		
			}
			}				
			else if(ovlav==0 && ovrav==0)
			{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					{
						   cout<<"3"<<endl;
							pca9685.stop();
							checkObstackle();
					}
				else{
				forSpeed=60;
				bakSpeed =60;
				cout<<"booth zero"<<endl;
				pca9685.setModSpeed(forSpeed, bakSpeed);
				pca9685.stop();
				delay(80);
				pca9685.goForward();
						
			}
			} 
			else if(ovlav==0 && ovrav!=0)
			{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					{
						   cout<<"4"<<endl;
							pca9685.stop();
							checkObstackle();
					}
				else{				
					forSpeed=7;
					bakSpeed =100;
					cout<<"LEFT"<<endl;
					pca9685.setModSpeed(forSpeed, bakSpeed);
					pca9685.stop();
					delay(30);
					pca9685.goLeft();
				
				
					}
				}			  		
			else{
				if(disValue <= DIR_DISTANCE_ALERT && disValue>10)
					{
						   cout<<"5"<<endl;
							pca9685.stop();
							checkObstackle();
					}		
					else{
				cout<< "AfterAll: "<<endl;
				pca9685.setModSpeed(55, 55);
				pca9685.stop();
				delay(70);
				pca9685.goForward();
				
			}
		}
		}else
		{
			pca9685.stop();
		}
			
        Mat houghP(bgr.size(),CV_8U,Scalar(0));
        
        ld.setShift(0,0);
        ld.drawDetectedLines(houghP);
        imshow("HoughP",houghP);
		
		if (cvWaitKey(20) == 'q')
			break;
	}
	capture.release();
	return 0;
}
