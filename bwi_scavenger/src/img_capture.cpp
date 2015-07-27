#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cstdio>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"

using namespace cv;
using namespace std;


Mat frame,img,ROI;
Rect cropRect(0,0,0,0);
Point P1(0,0);
Point P2(0,0);

const char* winName="Crop Image";
bool clicked=false;
int i=0;
char imgName[15];
std::string path_to_template = "/home/bwi/shiqi/template.jpg";


void checkBoundary(){
    //check croping rectangle exceed image boundary
    if(cropRect.width>img.cols-cropRect.x)
        cropRect.width=img.cols-cropRect.x;
    
    if(cropRect.height>img.rows-cropRect.y)
        cropRect.height=img.rows-cropRect.y;
    
    if(cropRect.x<0)
        cropRect.x=0;
    
    if(cropRect.y<0)
        cropRect.height=0;
}

void showImage(){
    img=frame.clone();
    checkBoundary();
    if(cropRect.width>0&&cropRect.height>0){
        ROI=frame(cropRect);
        imshow("cropped",ROI);
    }

    rectangle(img, cropRect, Scalar(0,255,0), 1, 8, 0 );
    imshow(winName,img);
}


void onMouse( int event, int x, int y, int f, void* ){

    switch(event){

        case  CV_EVENT_LBUTTONDOWN  :
                                        clicked=true;

                                        P1.x=x;
                                        P1.y=y;
                                        P2.x=x;
                                        P2.y=y;
                                        break;

        case  CV_EVENT_LBUTTONUP    :
                                        P2.x=x;
                                        P2.y=y;
                                        clicked=false;
                                        break;

        case  CV_EVENT_MOUSEMOVE    :
                                        if(clicked){
                                        P2.x=x;
                                        P2.y=y;
                                        }
                                        break;

        default                     :   break;


    }


    if(clicked){

        if(P1.x>P2.x){ 
            cropRect.x=P2.x;
            cropRect.width=P1.x-P2.x; 
        }
        else {
            cropRect.x=P1.x;
            cropRect.width=P2.x-P1.x; 
        }

        if(P1.y>P2.y){ 
            cropRect.y=P2.y;
            cropRect.height=P1.y-P2.y; 
        }
        else {         
            cropRect.y=P1.y;
            cropRect.height=P2.y-P1.y; 
        }
    }


    showImage();

}


int main( int argc, char** argv )
{
    if (argc != 1)
        return -1;
  
    CvCapture* capture; 
  
    capture = cvCaptureFromCAM(0); 
  
    if (!capture) {
        ROS_ERROR("No camera detected! \n");
        return -1;
    }
  
    namedWindow(winName, CV_WINDOW_AUTOSIZE);
    // Mat frame;
  
    while (true) {
  
        frame = cvQueryFrame(capture);
  
        if (frame.empty()) {
          ROS_WARN("No captured frame -- Break! \n");
          break;
        }
  
        imshow( winName, frame);
        char input = waitKey(10);
  
        if (input == 's') {
            std::time_t rawtime;
            std::tm* timeinfo;
            char buffer [80];
            
            cout<<"Click and drag for Selection"<<endl<<endl;
            cout<<"------> Press 's' to save"<<endl<<endl;
  
            cout<<"------> Press '8' to move up"<<endl;
            cout<<"------> Press '2' to move down"<<endl;
            cout<<"------> Press '6' to move right"<<endl;
            cout<<"------> Press '4' to move left"<<endl<<endl;
  
            cout<<"------> Press 'w' increas top"<<endl;
            cout<<"------> Press 'x' increas bottom"<<endl;
            cout<<"------> Press 'd' increas right"<<endl;
            cout<<"------> Press 'a' increas left"<<endl<<endl;
  
            cout<<"------> Press 't' decrease top"<<endl;
            cout<<"------> Press 'b' decrease bottom"<<endl;
            cout<<"------> Press 'h' decrease right"<<endl;
            cout<<"------> Press 'f' decrease left"<<endl<<endl;
  
            cout<<"------> Press 'r' to reset"<<endl;
            cout<<"------> Press 'Esc' to quit"<<endl<<endl;
  
  
            // src=imread("src.png",1);
  
            setMouseCallback(winName,onMouse,NULL );
            imshow(winName,frame);
  
            while(1){

                char c=waitKey();
                if(c=='s'&&ROI.data)
                {
                  //sprintf(imgName,"%d.jpg",i++);
                  //imwrite(imgName,ROI);
                  imwrite(path_to_template, ROI); 
                  ROS_INFO(" template saved: %s\n", path_to_template.c_str());
                  break;
                }
                if(c=='6') cropRect.x++;
                if(c=='4') cropRect.x--;
                if(c=='8') cropRect.y--;
                if(c=='2') cropRect.y++;
  
                if(c=='w') { cropRect.y--; cropRect.height++;}
                if(c=='d') cropRect.width++;
                if(c=='x') cropRect.height++;
                if(c=='a') { cropRect.x--; cropRect.width++;}
  
                if(c=='t') { cropRect.y++; cropRect.height--;}
                if(c=='h') cropRect.width--;
                if(c=='b') cropRect.height--;
                if(c=='f') { cropRect.x++; cropRect.width--;}
  
                if(c==27) break;
                if(c=='r') {
                    cropRect.x=0;cropRect.y=0;cropRect.width=0;
                    cropRect.height=0;
                }

                showImage();
            }
        }
    }
  
    return 0;
}

