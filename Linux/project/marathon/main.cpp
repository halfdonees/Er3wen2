#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define exp 2.718281828


#include <cv.h>
#include <dlfcn.h>
#include <highgui.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <limits.h>
#include <string.h>
#include <string>
#include <libgen.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include "Walking.h"
#include "MotionStatus.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "StatusCheck.h"
#include "VisionMode.h"
#include <sstream>
//#include <aruco.h>

#include "gyro.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"


using namespace cv;
using namespace std;


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
int Change(int x, int y);
void drawAllLinesP(Mat &input, const std::vector<Vec4i> &lines);

float distanceToLine(int x1,int y1,int x2,int y2,int x0,int y0) {
    float e = fabs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
    float den = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    return e / den;
}
int lineSegmentLength2(int x1, int y1, int x2, int y2) {
    return (x2 - x1) *(x2 - x1) + (y2 - y1) *(y2 - y1);
}
void applyGaussianBlur(Mat &frame,Mat &out, int width) {
    GaussianBlur(frame, out, Size(width, width),0,0);
}
void applyCannyEdge(Mat &frame, Mat &out, int low, int high) {
    Canny(frame, out, low, high);
}
void applyHoughLinesP(Mat &frame,float rho,float  theta, int threshold, int min_line_length, int max_line_gap){
}

void calcLinesP(const Mat &input, std::vector<Vec4i> &lines);
void drawLinesP(Mat &input, const std::vector<Vec4i> &lines);

string IntToString (int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}


vector<Vec4i> mergeLines(vector<Vec4i> linesP , int dist) {
    vector<Vec4i> olines;
    vector<Vec4i> nlines;
    bool merged;
    int Psize = linesP.size();
    for (int s = 0; s < Psize; s = s+1) {    //Scan line in lineP
        int x1 = linesP[s][0];
        int y1 = linesP[s][1];
        int x2 = linesP[s][2];
        int y2 = linesP[s][3];
        int dy = y2 - y1;
        int dx = x2 - x1;
        
        if ((dy < 0) || ((dy == 0) && (dx < 0))) {
            int temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        if (dy < 0) {
            dy = -dy;
        }
        if (dx < 0) {
            dx = -dx;
        }
        double theta = atan2(dy, dx);
        merged = false;
        for (int i = 0; i < olines.size(); i = i+1) { //Compare line with olines
            
            int ox1 = olines[i][0];
            int oy1 = olines[i][1];
            int ox2 = olines[i][2];
            int oy2 = olines[i][3];
            float thetao = atan2(oy2-oy1,ox2-ox1);
            
            if (fabs(theta - thetao) < 5 * CV_PI / 240 ) { //find parallel 0.34
                if (distanceToLine(ox1, oy1, ox2, oy2, x1, y1) <= 10 && distanceToLine(ox1, oy1, ox2, oy2, x2, y2) <= 10) {
                    //cout << "a\n";
                    if (y1 < oy1) {
                        oy1 = y1;
                        ox1 = x1;
                    }
                    if (y2 < oy1) {
                        oy1 = y2;
                        ox1 = x2;
                    }
                    if (y2 > oy2) {
                        oy2 = y2;
                        ox2 = x2;
                    }
                    if (y1 > oy2) {
                        oy2 = y1;
                        ox2 = x1;
                    }
                    
                    nlines.push_back({ox1,oy1,ox2,oy2});
                    
                    double nlength = lineSegmentLength2(ox1, oy1, ox2, oy2);
                    for (int j = 0; j < i+1 ; j++) {
                        if (nlength >= lineSegmentLength2(olines[j][0], olines[j][1], olines[j][2], olines[j][3])) { // nlength >= slength
                            olines.erase(olines.begin() + i);
                            olines.insert(olines.begin() + j, nlines[nlines.size()-1]);
                        }
                    }
                    merged = true;
                    break;
                }
            }//end find parallel
        }//end compare linesP with olines
        if (merged == false) {
            nlines.push_back({x1,y1,x2,y2});
            double nlength = lineSegmentLength2(x1, y1, x2, y2);
            for (int j = 0; j < olines.size(); j++) {
                if (nlength > lineSegmentLength2(olines[j][0], olines[j][1], olines[j][2], olines[j][3])) {
                    //cout << "b\n";
                    olines.insert(olines.begin() + j, nlines[nlines.size()-1]);
                    nlines.clear();
                    merged = true;
                    break;
                }
            }
            if (merged == false) {
                //cout << "c\n";
                olines.push_back(nlines[nlines.size()-1]);
                nlines.clear();
            }
        }
    }//end scan lines
    return olines;
}

vector<Vec4i> mergedLinesInImage(Mat &input,string name) {// blur, canny, Hough, merge ,show
    Mat edge;
    //applyGaussianBlur(input,input,5);
    applyCannyEdge(input, edge, 80,160);
    //imshow(name, edge);
    vector<Vec4i> linesP;
    vector<Vec4i> linesM;
    
    HoughLinesP(input, linesP, 1, CV_PI / 360, 50, 70, 10);
    linesM = mergeLines(linesP,144);
    cvtColor(input,input,CV_GRAY2BGR);
    drawAllLinesP(input,linesM);
    imshow("roi",input);
    return linesM;
}

void resizeCam(Mat&,Mat&,float);
void adjustLines(vector<Vec4i> &lines, int cols,int rows );

void walking_set(double X,double Y,double A);
void walking_time(double X,double Y,double A,double time);


int  cam_brightness = 50;
int  cam_contrast = 12;
int  cam_saturation = 10;

//#include <aruco/aruco.h>

int main(void)
{



	clock_t clk_begin,clk_end;

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)       //?!!Oa????a???a???a!M|??a?, Dynamixela-????a?.
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());      //MotionManagera??a?a!!O?a?a!M|a?actione!?a???Â¢FFDe!L?a?.
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());        //MotionManager?-e?!Le!P?e!Mae!L?a??-e?!L??a-??Â¢FFDe!?a!L!c??Â¢FGg?
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());     //MotionManagere!L?a?a-??Â¢FFDe?a!L!c?

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

    LinuxActionScript::PlayMP3("../../../Data/mp3/start up.mp3");


/*
	end of initialize
 */

    double cam_B = (double)cam_brightness/100;
    double cam_C = (double)cam_contrast/100;
    double cam_S = (double)cam_saturation/100;

    namedWindow("cam control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    moveWindow( "cam control", 600, 20 );
    createTrackbar("brightness", "cam control", &cam_brightness, 100);
    createTrackbar("contrast", "cam control", &cam_contrast, 100);
    createTrackbar("saturation", "cam control", &cam_saturation, 100);
	Action::GetInstance()->Start(16);//stand
	
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);


    MotionManager::GetInstance()->SetEnable(true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
    
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
    Head::GetInstance()->MoveByAngle(0, -5);


    //states
    enum State {READY,
    			FIRST_STEP,
    			TRACKING,
    			BLINDWALK,
    			FIND_ARROW_L,
    			FIND_ARROW_R,
    			FIND_ARROW_S,
    			ORTH_L,
    			ORTH_R};

    int state = READY;

    CvCapture *capture;
    Mat frame;
    Mat hsvFrame;
    Mat cam;
    Mat gray;
    Mat yccFrame;
    

    VideoCapture video(0);
    video >> cam;
    //cout << "test\n";
    namedWindow("webcam",WINDOW_AUTOSIZE);
    
    Mat roi1;//track line near
    Mat roi2;
    //Mat frame2;

    int counterM=0;
    string savePath;
   
    int X = 15;
    int A = 0;
    int Y = 0;

    //find lines
    double theta;
    double offset;

    vector<Vec4i> linesM;
    vector<Vec4i> linesL;
    vector<Vec4i> linesR;
    
    //perspective
    Mat top_view;
   	Point2f srcQua[4];
	Point2f top_viewQua[4];
	int scale = 71;

    top_view = Mat::zeros(cam.rows * 1, cam.cols * 0.5, frame.type());

    Mat erodeStruct = getStructuringElement(MORPH_CROSS,Size(3,3));
 
    cam_B = (double)cam_brightness/100;
	cam_C = (double)cam_contrast/100;
	cam_S = (double)cam_saturation/100;

	cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, cam_B);
	cvSetCaptureProperty(capture, CV_CAP_PROP_CONTRAST, cam_C);
    cvSetCaptureProperty(capture, CV_CAP_PROP_SATURATION, cam_S);



	//imshow("qwe",tempL1);
    while(1)
    { 
    	StatusCheck::Check(cm730);

//        printGYRO(&cm730);

  
	    video >> cam;
	    video >> cam;
	    video >> cam;
	    video >> cam;
	    video >> cam;
	    video >> cam;
	    video >> cam;

        flip(cam,cam,-1);
        resizeCam(cam,frame,0.5);


        cvtColor(frame,yccFrame,COLOR_BGR2YCrCb);

        //eagle view
       	srcQua[0] = Point2f(0,              (frame.rows * 0.4) - 1);
		srcQua[1] = Point2f(frame.cols - 1, (frame.rows * 0.4) - 1);

		srcQua[2] = Point2f(0,              frame.rows - 1);
		srcQua[3] = Point2f(frame.cols - 1, frame.rows - 1);

		top_viewQua[0] = Point2f(0,                                               top_view.rows - (frame.rows * 0.6) - 1);
		top_viewQua[1] = Point2f(frame.cols - 1,                                  top_view.rows - (frame.rows * 0.6) - 1);

		top_viewQua[2] = Point2f((frame.cols * (1 - (float)scale / 100) / 2) - 1, top_view.rows - 1);
		top_viewQua[3] = Point2f((frame.cols * (1 + (float)scale / 100) / 2) - 1, top_view.rows - 1);

		Mat warp_mat = getPerspectiveTransform(srcQua, top_viewQua);

		warpPerspective(frame, top_view, warp_mat, top_view.size());


        Mat yccFrame_Cr(yccFrame.rows,yccFrame.cols,CV_8U);
        Mat hsv_top;
        cvtColor(top_view,hsv_top,COLOR_BGR2HSV);
        Mat h_top(top_view.rows,top_view.cols,CV_8U);

        //get channel 1 ,Hue
        for(int i =0;i<hsv_top.rows;i++){
            for(int j =0;j<hsv_top.cols;j++){
                h_top.at<uchar>(i,j) = hsv_top.at<Vec3b>(i,j)[0];
            }
        }

//        cvtColor(hsv_top,h_top,CV_BGR2GRAY);
        Mat top_gray;
        applyGaussianBlur(top_view,top_view,5);
        Canny(top_view,top_gray,2,120);
        imshow("top_gray",top_gray);
        //cout << "test2\n";
        imshow("top_hue",h_top);

        
       	erode(frame,frame,erodeStruct,Point(-1,-1));

       	cvtColor(frame,frame,CV_BGR2HSV);

        roi1 = top_gray(Rect(h_top.cols*0.15,h_top.rows*0.6,h_top.cols*0.7,h_top.rows *0.4));
//        Canny(h_top,h_top,120,200);

//        roi1 = h_top(Rect(h_top.cols*0.15,h_top.rows*0.65,h_top.cols*0.7,h_top.rows *0.35));

        rectangle(top_view, Point(top_view.cols*0.15,top_view.rows*0.6), 
        					Point(top_view.cols*0.85,top_view.rows*1-1), Scalar(255,0,0), 3); // draw near

       	//cout << "test3\n";

        //cout << "test3.1\n";

        linesM = mergedLinesInImage(roi1,"roi_near");
        //cout << "test3.2\n";

        adjustLines(linesM,hsv_top.cols*0.15,hsv_top.rows*0.6);

        if(linesM.size()!=0){
        	int x1 = linesM[0][0];
        	int y1 = linesM[0][1];
        	int x2 = linesM[0][2];
        	int y2 = linesM[0][3];

        	if(pow((x1-x2),2)+pow((y1-y2),2) >= 1600 ){
            	theta = atan2(y2 - y1, x2 - x1);
                offset = ( x1 + x2 ) / 2;

                if(theta >= 0)	theta = CV_PI/2 - theta;
                else	theta = - CV_PI/2 -theta;
                
                theta /= CV_PI;
                cout << "theta: " << theta << endl;
                Y = ((frame.cols / 2) - offset);

                if((theta >= 0.25)){
                	cout << "The angle is too big\n";
                	A = 5*Y;
                	X = 0;

                }

                else if(theta <= -0.25){
                	cout << "The angle is too big\n";
                	A = 5*Y;
                	X = 0;
                }


              	else{
	                A = 20 * theta;
	                //cout << "A: " << A << endl;
	               	A += (0.2*Y);	
	               	A += -3;
	               	X = 6;
              	}
        	}
           	//cout << offset << endl;
    	}//end if (linesM.size()!=0)

    	cout << "A_MOVE_AMPLITUDE: " << A << endl;


        drawLinesP(top_view,linesM);
//        drawLinesP(h_top,linesL);
        imshow("top view", top_view);
    	createTrackbar("scalar", "top view", &scale, 100);


        imshow("webcam",frame);

        cout << "time: " << time(0) << "-------------\n";


        waitKey(1);  

      
        if(StatusCheck::m_is_started == 0){
            clk_begin = clock();
            continue;
        }


        if(StatusCheck::m_cur_mode==SOCCER){
        	X = 6;
          
            //state////////////////////////////////////////////////
        	switch(state){
        	case READY:{ 
                cout << "Get Ready........\n";
                usleep(3000);
                state = FIRST_STEP;
                break;       		
        	}

        	case FIRST_STEP:{
                cout << "state:FirstStep\n";
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                walking_time(X,0,0,0.1);
                state = TRACKING;
                break;        		
        	}

        	case TRACKING:{
                cout << "state : tracking\n";
                if(linesM.size()!=0){
                	int x1 = linesM[0][0];
                	int y1 = linesM[0][1];
                	int x2 = linesM[0][2];
                	int y2 = linesM[0][3];
                	if(pow((x1-x2),2)+pow((y1-y2),2) >= 1600 ){
                    	theta = atan2(y2 - y1, x2 - x1);
                        offset = ( x1 + x2 ) / 2;

                        if(theta >= 0)	theta = CV_PI/2 - theta;
                        else	theta = - CV_PI/2 -theta;
                        
                        theta /= CV_PI;
                        cout << "theta: " << theta << endl;
                        Y = ((frame.cols / 2) - offset);

                        if((theta >= 0.6)){
                        	cout << "The angle is too big";
                        	A = 5*Y;
                        	X = 3;
                        }

                        else if(theta <= -0.6){
                        	cout << "The angle is too big";
                        	A = 5*Y;
                        	X = 3;
                        }


                      	else{
                        A = 15 * theta;
                        //cout << "A: " << A << endl;
                       	A += (0.2*Y);	
                       	X = 12;
                      	}
                	}
                   	//cout << offset << endl;
            	}//end 
            	//cout << "Y: " << Y << endl;
            	cout << "A_MOVE_AMPLITUDE: " << A << endl;
            	walking_set(X,0,A);        	
            	break;	
        	}//end case TRACKING

            case FIND_ARROW_S: {
                cout <<"state:find straight arrow.\n";
                walking_time(10,0,0,8);
                walking_set(12,0,0);
                state = FIRST_STEP;
                break;
            }

            case FIND_ARROW_R:{
                cout <<"state:find right arrow.\n";
                walking_time(0,0,-10,6);
                walking_time(12,0,0,5);
                walking_set(12,0,0);
                state = FIRST_STEP;
                break;
            }

            case FIND_ARROW_L:{
                cout <<"state:find left arrow.\n";
                walking_time(0,0,0,6);
                walking_time(12,0,0,5);
                walking_set(12,0,0);
                state = FIRST_STEP;
                break;
            }
            
            case ORTH_L:{
            	cout << "state = orthl\n";
                walking_time(0,20,20,2);
                walking_set(12,0,0);
                state = FIRST_STEP;
                break;
            }
            case ORTH_R:{
            	cout << "state = orthR\n";
                walking_time(0,20,-20,2);
                walking_set(12,0,0);
                state = FIRST_STEP;
                break;
            }


        	}//end switch(state)


		}// end if(cur_status==SOCCER)


    }//end while(1) 
    return 0;

}


void resizeCam(Mat& input,Mat& output,float scale){
    resize(input,output,
        Size(input.cols* scale,input.rows*scale),0,0);
}

void walking_set(double X,double Y,double A){
    Walking::GetInstance()->X_MOVE_AMPLITUDE=  X;
    Walking::GetInstance()->Y_MOVE_AMPLITUDE=  Y;
    Walking::GetInstance()->A_MOVE_AMPLITUDE=  A;
}

void walking_time(double X,double Y,double A,double time){
    walking_set(X,Y,A);
    Walking::GetInstance()->Start();
    cout << Walking::GetInstance()->A_MOVE_AMPLITUDE <<endl ;
    clock_t start, clk =0;
    start = clock();
    clk = clock();
    while( ( (double)(clk-start) / CLOCKS_PER_SEC ) < time){
        clk = clock();
    }
}

void calcLinesP(const Mat &input, std::vector<Vec4i> &lines) {
    Mat contours;
    Canny(input, contours, 10, 75);
    //imshow("c",contours);
    lines.clear();
    HoughLinesP(contours, lines, 1, CV_PI / 360, 80);
}
void drawLinesP(Mat &input, const std::vector<Vec4i> &lines){
    for (int i = 0; i<lines.size()&&i<1; i++) {
        line(input, Point(lines[i][0], lines[i][1]), 
        	Point(lines[i][2], lines[i][3]), Scalar(0,255,255), 3);
    }
}
void drawAllLinesP(Mat &input, const std::vector<Vec4i> &lines){
    for (int i = 0; i<lines.size(); i++) {
        line(input, Point(lines[i][0], lines[i][1]), 
        	Point(lines[i][2], lines[i][3]), Scalar(rand()%256,rand()%256,rand()%256), 2);
    }
}

void adjustLines(vector<Vec4i> &lines, int cols,int rows ){
	for(int i = 0; i < lines.size(); i++){
		lines[i][0] += cols;
		lines[i][2] += cols;
		lines[i][1] += rows;
		lines[i][3] += rows;
	}

}