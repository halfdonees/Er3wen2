#include <iostream>
#include <opencv/cv.h>
#include <highgui.h>

#include <unistd.h>
#include <vector>
#include <limits.h>
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


#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"

#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"
#define VIDEOSTREAM         video >> cam; \
							video >> cam; \
							video >> cam; \
                            video >> cam; \
                            video >> cam; \
                            video >> cam; \
                            video >> cam; \
							video >> cam; \
							video >> cam; 
							

using namespace Robot;
using namespace cv;
using namespace std;

double area_red, area_yellow;

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

void change_current_dir() {
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig) {
    exit(0);
}

void resizeCam(Mat& input,Mat& output,float scale) {
    resize(input,output,Size(input.cols* scale,input.rows*scale),0,0);
}

void walking_set(double X,double Y,double A) {
    Walking::GetInstance()->X_MOVE_AMPLITUDE=  X;
    Walking::GetInstance()->Y_MOVE_AMPLITUDE=  Y;
    Walking::GetInstance()->A_MOVE_AMPLITUDE=  A;
}

void walking_time(double X,double Y,double A,double time) {
    walking_set(X,Y,A);
    Walking::GetInstance()->Start();
    clock_t start, clk =0;
    start = clock();
    clk = clock();
    while( ( (double)(clk-start) / CLOCKS_PER_SEC ) < time) {
        clk = clock();
    }
}

/*
static void onMouse(int Event, int x, int y, int flags, void* param) {  
    Mat &img = *((Mat*)param);

    if (Event == CV_EVENT_LBUTTONDOWN) {
        cout << "x:" << x << "  y:" << y << endl;

        cout << "B: " << (int)img.at<Vec3b>(y, x)[0] << endl;
        cout << "G: " << (int)img.at<Vec3b>(y, x)[1] << endl;
        cout << "R: " << (int)img.at<Vec3b>(y, x)[2] << endl;
    }

}
*/
float whiteArea(Mat &img){
	int widthLimit = img.cols;
	float counter =0;
	for(int height=0; height<img.rows; height++){
	    uchar *data = img.ptr<uchar>(height);
    	for(int width=0; width<widthLimit ; width++){
        	if(data[width]!=0) counter+=1;
        }
	}
	float area = img.cols*img.rows;
	return counter/area;
}

void drawColor(Mat& frame, Mat& inRanged, Scalar color){
	for(int i=0;i<frame.rows;i++){
		for(int j=0;j<frame.cols;j++){
			if(inRanged.at<uchar>(i,j)){
				frame.at<Vec3b>(i,j)[0] = color[0];
				frame.at<Vec3b>(i,j)[1] = color[1];
				frame.at<Vec3b>(i,j)[2] = color[2];
			}
		}
	}

}

int main(void) {
    clock_t clk_begin,clk_end;
	
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false) {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false) {
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
    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS) {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(0 < firm_ver && firm_ver < 27) {
        #ifdef MX28_1024
            Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
        #else
            fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
            fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
            exit(0);
        #endif
    }
    else if(27 <= firm_ver) {
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
    /*
    end of initialize
    */
//    Action::GetInstance()->Start(75);//stand
    while(Action::GetInstance()->IsRunning()) usleep(5*1000);

    sleep(3);
    Action::GetInstance()->Start(16);//stand
    while(Action::GetInstance()->IsRunning()) usleep(5*1000);

//    MotionManager::GetInstance()->SetEnable(true);
    
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
    Head::GetInstance()->MoveByAngle(0, 15);

    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    

    VideoCapture video(0);

    Mat frame;
    Mat colored_frame;
    Mat frame_hsv;
    Mat cam;

    video >> cam;


    flip(cam,cam,-1);
    resizeCam(cam,frame,0.5);

    Mat maskBlue;
    Mat maskYellow;
    Mat mask;

    Mat maskRed;
    Mat maskRed1;
    Mat maskRed2;

    Mat lava;

    const Scalar blueLower = Scalar(90,80,40);
    const Scalar blueUpper = Scalar(120,255,255);

    const Scalar yellowLower = Scalar(20,80,80);
    const Scalar yellowUpper = Scalar(40,255,255);

    const Scalar redLower1 = Scalar(0,130,60);
    const Scalar redUpper1 = Scalar(10,255,255);
    const Scalar redLower2 = Scalar(170,130,60);
    const Scalar redUpper2 = Scalar(180,255,255);

    Mat erodeStruct = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat darwinStruct = getStructuringElement(MORPH_RECT, Size(190, 190));

    enum STATE{
    	READY           =0,
    	WALKING         =1,
    	LEFT_WALL       =2,
    	RIGHT_WALL      =3,
    	GATE		    =4,
    	LEFT_BORDER		=5,
    	RIGHT_BORDER	=6,
    	LOOK_LEFT,
    	LOOK_RIGHT,
    	GATE_SHIFT_LEFT,
    	GATE_SHIFT_RIGHT,
        YELLOW
    };

    int encounterWall=0;
    int encounterLeftBorder=0;
    int encounterRightBorder=0;
    int encounterGate=0;


    Mat roi_near;
    Mat roi_left;
    Mat roi_right;
    Mat roi_red;
    Mat roi_near_yellow;

    const float roi_near_yellow_height = 0.2;
    const float roi_near_yellow_width = 0.5;


    const float roi_near_height = 0.375;
    float left_wall_area=0;
    float right_wall_area=0;

    const float roi_left_height = 0.2;
    const float roi_left_width = 0.1;
    const float roi_right_height = roi_left_height;
    const float roi_right_width = roi_left_width;
    const float roi_side_margin = 0.3;

//    float yellow_area=0;

    int encounterYellow = 0;

    int state = WALKING;
//    int preState = READY;
    namedWindow("webcam",WINDOW_AUTOSIZE);
    namedWindow("hsv",WINDOW_AUTOSIZE);
    namedWindow("left",WINDOW_AUTOSIZE);
    namedWindow("near",WINDOW_AUTOSIZE);
    namedWindow("right",WINDOW_AUTOSIZE);
    namedWindow("mask",WINDOW_AUTOSIZE);
    namedWindow("Lava",WINDOW_AUTOSIZE);

	moveWindow("webcam",0,0);
	moveWindow("hsv",320,0);
	moveWindow("left",0,320);
	moveWindow("right",320,320);
	moveWindow("near",40,540);
	moveWindow("mask",640,0);
	moveWindow("Lava",640,400);

    while(1)
    { 

    	StatusCheck::Check(cm730);

        VIDEOSTREAM

    	flip(cam,cam,-1);
    	resizeCam(cam,frame,0.5);

    	colored_frame = frame.clone();

    	cvtColor(frame,frame_hsv,COLOR_BGR2HSV);

    	imshow("hsv",frame_hsv);


    	inRange(frame_hsv,blueLower,blueUpper,maskBlue);
        inRange(frame_hsv,yellowLower,yellowUpper,maskYellow);

        inRange(frame_hsv,redLower1,redUpper1,maskRed1);
        inRange(frame_hsv,redLower2,redUpper2,maskRed2);
        bitwise_or(maskRed1,maskRed2,maskRed);

	   	erode(maskYellow,maskYellow,erodeStruct);
		dilate(maskYellow,maskYellow,erodeStruct);
		dilate(maskYellow,maskYellow,erodeStruct);
	   	erode(maskYellow,maskYellow,erodeStruct);



	   	roi_near = maskBlue(Rect(0,maskBlue.rows*(1- roi_near_height ),maskBlue.cols,maskBlue.rows*roi_near_height));

        roi_left = maskYellow(Rect(0+maskYellow.cols*roi_side_margin,maskYellow.rows*(1-roi_left_height),
                        maskYellow.cols*roi_left_width,maskYellow.rows*roi_left_height));
        imshow("left",roi_left);

        roi_right = maskYellow(Rect(maskYellow.cols*(1-roi_right_width-roi_side_margin ),maskYellow.rows*(1-roi_right_height),
                        maskYellow.cols*roi_right_width,maskYellow.rows*roi_right_height));
        imshow("right",roi_right);

        roi_near_yellow = maskYellow(Rect(maskYellow.cols*(1-roi_near_yellow_width)/2,maskYellow.rows * (1- roi_near_yellow_height),
                                        maskYellow.cols*roi_near_yellow_width,maskYellow.rows*roi_near_yellow_height));
        imshow("yellowNear",roi_near_yellow);


/*
    	rectangle(frame,Point(frame.cols*0.48,frame.rows*0.08),Point(frame.cols*0.52,frame.rows*0.15),Scalar(255,255,255),1);

*/

        roi_red = maskRed(Rect(maskRed.cols*0.485,maskRed.rows*0.05,maskRed.cols*0.03,maskRed.rows*0.04));
        rectangle(frame,Point(frame.cols*0.485,frame.rows*0.05),Point(frame.cols*0.515,frame.rows*0.08),Scalar(255,255,255),1);

        imshow("webcam",frame);

        imshow("red",roi_red);
        imshow("red_mask",maskRed);





    	imshow("near",roi_near);

    	if(whiteArea(roi_near)>0.2) encounterWall = 1;
        else encounterWall = 0;

    	if(whiteArea(roi_left)>0.03) encounterLeftBorder = 1;
        else encounterLeftBorder = 0;
    	if(whiteArea(roi_right)>0.03) encounterRightBorder = 1;
        else encounterRightBorder = 0;
//    	if(whiteArea(roi_red)>0.7) encounterGate = 1;
 //       else encounterGate = 0;
        if(whiteArea(roi_near_yellow)>0.2) encounterYellow = 1;
    	else encounterYellow=0;

    	cout << "Gate: " << encounterGate << endl;


	   	erode(maskBlue,maskBlue,erodeStruct);
		dilate(maskBlue,maskBlue,erodeStruct);
		dilate(maskBlue,maskBlue,erodeStruct);
	   	erode(maskBlue,maskBlue,erodeStruct);
//    	imshow("mask",maskBlue);


        drawColor(colored_frame,maskRed,Scalar(0,0,255));
        drawColor(colored_frame,maskBlue,Scalar(255,0,0));
        drawColor(colored_frame,maskYellow,Scalar(0,255,255));

        imshow("colored_frame",colored_frame);
		dilate(maskBlue,maskBlue,darwinStruct);

		lava = maskBlue(Rect(0,maskBlue.rows*0.3,maskBlue.cols,maskBlue.rows*0.7));
	   	imshow("Lava",lava);




      	waitKey(33);
      
        if(StatusCheck::m_is_started == 0){
            clk_begin = clock();
            continue;
        }

        if(StatusCheck::m_cur_mode==SOCCER){

        	switch(state){
        		case READY:
        			Walking::GetInstance()->Stop();
        			cout << "Take a break.\n";
//                    sleep(1);
                    state = WALKING;
                    encounterWall = 0;
        			break;


	        	case WALKING:
                                    Head::GetInstance()->MoveByAngle(0,15);

	        		cout << "State: WALKING Straight\n";

	        		if(encounterGate==1){
	        			cout << "stop.\n";
	        			Walking::GetInstance()->Stop();
	        			state = GATE;
	        		}

	        		if(encounterRightBorder==1){
	        			state = RIGHT_BORDER;
	        			break;
	        		}


	        		else if(encounterLeftBorder==1){
	        			state = LEFT_BORDER;
	        			break;
	        		}

	        		else if(encounterWall==1){
	        			cout << "stop\n";
	        			Walking::GetInstance()->Stop();
                        sleep(1);
	        			Head::GetInstance()->MoveByAngle(35,15);
                        sleep(1);

	        			state = LOOK_LEFT;
	        			break;

	        		}
                    else if(encounterYellow==1){
                        cout << "Stop";
                        Walking::GetInstance()->Stop();
                        sleep(1);

                        state = YELLOW;
                    }
	        		else{
		        		walking_set(10,0,0);
	        			Walking::GetInstance()->Start();
	        			break;
	        		}

	        	case LOOK_LEFT:
	        		cout << "State: LOOK_LEFT";

//	        		imshow("Left",frame);
	        		left_wall_area = whiteArea(maskBlue);
	        		cout << left_wall_area << endl;

	        		Head::GetInstance()->MoveByAngle(-35,15);
	        		sleep(1);

	        		if(encounterLeftBorder==1)	state = LEFT_BORDER;
	        		else	state = LOOK_RIGHT;

	        		break;
	        		
	        	case LOOK_RIGHT:
	        		cout << "State: LOOK_ROGHT";
//	        		imshow("Right",frame);

	        		right_wall_area = whiteArea(maskBlue);
	        		cout << right_wall_area << endl;
	        		Head::GetInstance()->MoveByAngle(0,15);
	        		sleep(1);
	        		encounterWall = 0;

	        		if(encounterRightBorder==1)	state = RIGHT_BORDER;
	        		else	state = (left_wall_area>right_wall_area)? LEFT_WALL:RIGHT_WALL;

	        		left_wall_area=0;
	        		right_wall_area=0;

	        		break;

	        	case RIGHT_WALL:{
	        		cout << "State: RIGHT_WALL\n";
	        		cout << "turn left\n";
//	        		walking_set(-5,0,8);
	        		walking_set(-5,20,3);
	        		//-5,20,3

	        		Walking::GetInstance()->Start();
	        		int danger=0;
	        		int hei=frame.rows/2;
	        		for(int i=0;i<hei;i++){
	        			if(lava.at<uchar>(240-i,160))
	        				danger=1;
	        		}

	        		if(danger==0){
	        			state=READY;
//	        			preState=RIGHT_WALL;
	        		}

	        		break;
	        	}
	        	case LEFT_WALL:{

	        		cout << "State: LEFT_WALL\n";
	        		cout << "turn right\n";
//	        		walking_set(-5,0,-8);
	        		walking_set(-4,-16,-5);
	        		Walking::GetInstance()->Start();
	        		int danger=0;
	        		int hei=frame.rows/2;
	        		for(int i=0;i<hei;i++){
	        			if(lava.at<uchar>(240-i,160))
	        				danger=1;
	        		}

	        		if(danger==0){
	        			state=READY;
//	        			preState=LEFT_WALL;
	        		}
   
	        		break;
	        	}

	        	case LEFT_BORDER:{
	        		cout << "State: LEFT_BORDER\n";
	        		cout << "turn right.\n";
                    walking_time(-4,-16,-15,1);
//                    walking_time(-3,0,-15,1);

	        		state = WALKING;
	        		encounterLeftBorder=0;

	        		break;
	        	}
	        	case RIGHT_BORDER:{
	        		cout << "State: RIGHT_BORDER\n";
	        		cout << "turn left.\n";
                    walking_time(-5,20,15,1);

//	        		walking_time(-3,0,15,1);
	        		state = WALKING;
	        		encounterRightBorder=0;
	        		break;
	        	}

	        	case GATE:{
	        		cout << "State: GATE\n";
	        		Walking::GetInstance()->Stop();
	        		sleep(1);
	        		walking_time(-10,0,0,6);
                    sleep(1);
	       		    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
				    MotionManager::GetInstance()->SetEnable(true);

				    Action::GetInstance()->Start(61);
			        while(Action::GetInstance()->IsRunning()) usleep(5*1000);

                    Action::GetInstance()->Start(63);
                    while(Action::GetInstance()->IsRunning()) usleep(5*1000);
                    Action::GetInstance()->Start(63);
                    while(Action::GetInstance()->IsRunning()) usleep(5*1000);

                    Action::GetInstance()->Start(64);
                    while(Action::GetInstance()->IsRunning()) usleep(5*1000);


                    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
                    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

                    state = WALKING;
	        	}

                case YELLOW:{
                    cout << "State: YELLOW\n";
                    walking_set(-4,-16,-5);
                    Walking::GetInstance()->Start();
                    if(whiteArea(roi_near_yellow)<0.1){
                        state = WALKING;
                        break;
                    }
                    break;
                }



        	}//end switch

        	cout << endl;
        }//end if soccer


    }//end while   

    return 0;
}
