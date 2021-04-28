/*
It's myself Image Recongnition
*/
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#define master_address 0x04

#define uchar unsigned char
using namespace cv;
using namespace std;

int main(int argc, char *argv[]){

    Mat img, grayimg, blurimg, cannyimg, imageroi;
    Mat textimg[100];
    Mat addimgall(480,640,CV_8U,Scalar(0)), addimgcl;

    double threshValue = 120;
    int edgethresh = 30, file_idx = 0, sendValue = 0;
    int edgedata[50] = {0};
    int flowerlocal[50] = {0};
    char filename[12], fileidx[3];                      //圖片名稱和編號
    char imagename[100];
    bool button = false;
/**********************************************************/
    int fd = wiringPiI2CSetup(master_address);
    VideoCapture cap;   //Camera
    cap.open(0);

    if(!cap.isOpened()){
        printf("\nCamera is not connected.\n\n");
        return -1;
    }

    while(true){
        cap >> img;
        cvtColor( img, grayimg, CV_BGR2GRAY);
        GaussianBlur( grayimg, blurimg, Size(5,5), 1.5, 1.5);
        Canny( blurimg, cannyimg, edgethresh, edgethresh*3, 3);
        imshow("finally",cannyimg);

        if(waitKey(5) == 27){             //30ms
            printf("\nClose Capture!!\n");
            break;
        }
	//設定i2c從arduino接收訊號控制啟動搜尋位置
	//0:關閉
	//1:啟動
        //if(wiringPiI2CRead(fd) == 0) button = false;
	//else if(wiringPiI2CRead(fd) == 1) button = true;

        int value = 25;
        if( file_idx < value ){                     //儲存圖片25張0-24
            sprintf(fileidx, "%03d", file_idx++);
    	    strcpy(filename, "test");
    	    strcat(filename, fileidx);
    	    strcat(filename, ".png");

    	    imwrite(filename, cannyimg);
    	    //printf("Image file %s saved.\n",filename);
    	    if(file_idx == value){
		    printf("Image file have %d saved.\n\n",file_idx);
		    button = true;
	    }
    	 }
	if(button == true){
            for(int i = 0; i< value; i++){          //讀取圖片25張0-24
                sprintf(imagename,"test%03d.png",i);
                textimg[i] = imread(imagename);
                //printf("Load %s !!\n",imagename);
                cvtColor( textimg[i], textimg[i], CV_BGR2GRAY);
                add(textimg[i],addimgall,addimgall);

                if(i == (value-1)){             //顯示最後相加圖片

                    imwrite("addimg.png",addimgall);
                    imshow("addimg.png",addimgall);
                    //addimgcl = addimgall.clone();

                    imageroi= addimgall(Rect( 0, 470, 640,1)); //ROI區域
                    imwrite("imageroi.png",imageroi);
                    imshow("imageroi.png",imageroi);
                    printf("Add image all step!!!\n\n");
                    break;
                }
            }
            int freq = 0;
            for(int x = 0; x <= imageroi.cols;x++){
                uchar data=imageroi.at<uchar>(imageroi.rows,x);
                //printf("data = %d!!\n",data);

                if(data == 255){
                    printf("data.cols = %d !!\n",x);
                    edgedata[freq] = x;
                    freq +=1;
                }
            }
            printf("freq = %d !!!!\n",freq);
            int y =0;
            for(int z = 1;z < freq;z++){
                 //printf("edgedata = %d !!!!\n",edgedata[i]);
                 if(edgedata[z]-edgedata[z-1] > 1){
                        //printf("%d edge\n",z);
                        double N = 640 - (edgedata[z]+edgedata[z-1])/2; //花朵座標設定
                        flowerlocal[y] = (int) round(200*0.005*N/2.3);  //步數 此數將要傳給arduino
                        printf("flowerlocal[%d] = ",y);printf(" %d !!\n",flowerlocal[y]);
                 	y+=1;
		 }
	    }
	    button = false;
        }//button
    }
 return 0;
}







