#include "NearPixel.h"
#include "opencv/cv.h"

#include <iostream>

using namespace cv;
using namespace std;

Mat applyDilatePixToMask(Mat &mask, Mat &img, int offset1,int offset2, int offset3){
    Mat output;
    img.copyTo(output);
    for(int i=0;i<mask.rows;i++){
        for(int j=0;j<mask.cols;j++){
            if(mask.at<uchar>(i,j)){
                lookNearByPixels(j, i, Scalar(output.at<Vec3b>(i,j)[0],output.at<Vec3b>(i,j)[1],output.at<Vec3b>(i,j)[2]), output,offset1,offset2,offset3);
            }
        }
    }
    cvtColor(output,output,COLOR_BGR2GRAY);
    return output;
}


void lookNearByPixels(int x, int y,Scalar pix, Mat& img, int offset1, int offset2, int offset3){
    //cout << "--\n";
    Scalar color = Scalar(img.at<Vec3b>(y,x)[0],img.at<Vec3b>(y,x)[1],img.at<Vec3b>(y,x)[2]);
    Scalar lowerRange = getRange(pix, -offset1,-offset2,-offset3);
    Scalar upperRange = getRange(pix, offset1,offset2,offset3);
    
    bool similiar = singleInRange(color, lowerRange,upperRange);
    
    if(x>img.cols||x<0||y>img.rows||y<0){
        //cout << "out of size .                         return\n";
        return;
    }
    if(singleInRange(color, Scalar(255,255,255), Scalar(255,255,255))){
        //cout << "pixels have been detected before.     return\n";
        return;
    }
    
    if (similiar){
        //cout << lowerRange << endl << upperRange << endl;
        img.at<Vec3b>(y,x)[0] = 255;
        img.at<Vec3b>(y,x)[1] = 255;
        img.at<Vec3b>(y,x)[2] = 255;
        //cout << "find similiar pixels!\n" << endl;
    }
    else{
//        cout << lowerRange << endl << color << endl << upperRange << endl;
//        cout << color[0] << " " << color[1] << " " << color[2] << endl;
        //cout << "color is too different.\n";
        return;
    }
    
    //cout << "move to right~ ";
    lookNearByPixels(x+1, y,color, img, offset1, offset2, offset3); // right
    //cout << "move to left~~ ";
    lookNearByPixels(x-1, y,color, img, offset1, offset2, offset3); // left
    //cout << "movet to up~~~ ";
    lookNearByPixels(x, y-1,color, img, offset1, offset2, offset3); // up
    //cout << "move to down~~ ";
    lookNearByPixels(x, y+1,color, img, offset1, offset2, offset3); // down
}

bool singleInRange(Scalar pixel,Scalar lower,Scalar upper){
    //cout << lower << endl << pixel << endl << upper << endl;
    if(pixel[0] >= lower[0] && pixel[0] <= upper[0]){
        if(pixel[1] >= lower[1] && pixel[1] <= upper[1]){
            if(pixel[2] >= lower[2] && pixel[2] <= upper[2]){
                return true;
            }
        }
    }
    return false;
}

Scalar getRange(Scalar pixel,int offset1,int offset2,int offset3){
    return Scalar(pixel[0]+offset1,pixel[1]+offset2,pixel[2]+offset3);
}


