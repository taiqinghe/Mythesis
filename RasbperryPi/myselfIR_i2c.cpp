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

    Mat img, grayimg, blurimg, cannyimg, imageroi,thresh,erodeimg;
    Mat textimg[25];
    Mat addimgall(480,640,CV_8U,Scalar(0));
	Mat Struct = getStructuringElement(MORPH_RECT,Size(5,5));

    int edgethresh = 30, value =25, file_idx = 0,i2cdata = 0,d = 0 ,count = 0;
    int edgedata[50] = {0};
    int center[21] = {0}, centerdata[10] ={0}, coordinate[10] = {0};         //中心點;目標物座標
    char filename[12], fileidx[3];                      //圖片名稱和編號
    char imagename[12];
    bool button = false, i2ctype = false;
    wiringPiSetup();
    int fd = wiringPiI2CSetup(master_address);
    VideoCapture cap;   //Camera
    cap.open(0);

    if(!cap.isOpened()){
        printf("\nCamera is not connected.\n\n");
        return -1;
    }
/*********************************************************************************/
    while(true){                                       //camera start
        cap >> img;	//openCV
        cvtColor( img, grayimg, CV_BGR2GRAY);
	erode( grayimg, erodeimg, Struct);
        GaussianBlur( erodeimg, blurimg, Size(3,3), 1.5, 1.5);
	threshold(blurimg,thresh, 160, 255, THRESH_BINARY);
        Canny( thresh, cannyimg, edgethresh, edgethresh*3, 3);

	//imshow("gray",grayimg);
	//imshow("erode",erodeimg);
	//imshow("IMG",img);
	//imshow("Blurimg",blurimg);
	//imshow("thresh",thresh);
	imshow("Canny",cannyimg);
        if(waitKey(5) == 27){             //5ms
            printf("\nClose Capture!!\n\n");
            break;
        }
/*********************************************************************************/
	if( i2cdata == 10){		//i2c send data
        	if(i2ctype  == false){
			for(int i = 0;i < 10;i++){
                printf("coordinate[%d] =",i);
                printf(" %d\n",coordinate[i]);
			}
			//wiringPiI2CWrite(fd,1);
			printf("i2c start working!!\n\n\n");
			i2ctype = true;
		}
		switch( wiringPiI2CRead(fd) ){
			case 0:
	    			//printf("Waiting Arduino to start\n\n");
	  			if(waitKey(5) == 27){             //30ms
            				printf("\nClose Capture!!\n\n");
           				return 0;
        			}
				break;
			case 1:
				if(d != i2cdata){
					wiringPiI2CWrite(fd,coordinate[d]);
		    			printf("Send Arduino coordinate[%d] = ",d);
					printf("%d\n",coordinate[d]);
					delay(100);
					d+=1;
				}
				else {
					printf(" RPI send 10 data to arduino\n\n");
					//wiringPiI2CWrite(fd,0);
				}
				break;
			case 2:
				printf("Working Done!!\n");
				printf("Clear Old DATA!!\n\n");
				d = 0;
				i2cdata = 0;
				file_idx = 0;
				threshold(addimgall,addimgall,0,0,THRESH_TOZERO_INV);
				//imshow("addimgall zero",addimgall);
				imageroi.release();
				for(int i = 0;i < 25;i++) textimg[i].release();
				for(int i = 0;i < 50;i++)	edgedata[i] = 0;
				for(int i = 0;i < 21;i++)	center[i] = 0;
				for(int i = 0;i < 10;i++)	centerdata[i] = 0;
				for(int i = 0;i < 10;i++)	coordinate[i] = 0;
				i2ctype = false;
				break;
		}

	}
/**********************************************************************************/
        if( file_idx < value ){                     	//擷取儲存圖片25張0-24
            sprintf(fileidx, "%03d", file_idx++);
    	    strcpy(filename, "test");
    	    strcat(filename, fileidx);
    	    strcat(filename, ".png");

    	    imwrite(filename, cannyimg); //*****************************see*****//
    	    //printf("Image file %s saved.\n",filename);
    	    if(file_idx == value){
    	        button = true;
                printf("Image file have %d saved.\n\n",file_idx);
            }
    	}
/**********************************************************************************/
        if(button == true){                             //button
            for(int i = 0; i< value; i++){              //讀取圖片25張0-24
                sprintf(imagename,"test%03d.png",i);
                textimg[i] = imread(imagename);
                //printf("Load %s !!\n",imagename);
                cvtColor( textimg[i], textimg[i], CV_BGR2GRAY);
                add(textimg[i],addimgall,addimgall);

                if(i == (value-1)){             	//顯示最後相加圖片

                   imwrite("addimg.png",addimgall);
                   imshow("addimg.png",addimgall);
                   imageroi= addimgall(Rect( 0, 470, 640,1)); //ROI區域*********************************see
                   imwrite("imageroi.png",imageroi);
                   //imshow("imageroi.png",imageroi);
                   printf("Add All images together !!!\n\n");
                   break;
                }
            }
/**********************************************************************************/
            int freq = 0;
            //for(int x = 0; x <= imageroi.cols;x++){                //搜尋roi中白色之座標
	    for(int x = 53; x <= 590;x++){                //搜尋roi中白色之座標
		 uchar data=imageroi.at<uchar>(imageroi.rows,x);
                //printf("data = %d!!\n",data);

                if(data == 255){
                    //printf("data.cols = %d !!\n",x);
                    edgedata[freq] = x; //儲存白色之座標(所有物體的邊緣)應該有22個
                    freq +=1;
                }
            }
            //printf("freq = %d !!!!\n",freq);
            int y = 0;
            for(int z = 1;z < freq;z++){
              //printf("edgedata = %d !!!!\n",edgedata[i]);
              if(edgedata[z]-edgedata[z-1] > 3){
                //printf("%d edge\n",z);
                double N = 640 - (edgedata[z]+edgedata[z-1])/2;   //兩邊緣之間的中心座標
                center[y] = (int) round(N*10/23);         //中心座標轉成實際移動需要的步數
                //center[y] = (int) round(200*0.005*N/2.3);         //中心座標轉成實際移動需要的步數

		//center[y] = (int) round(5*N*135/23);         //中心座標轉成實際移動需要的步數
		//center[y] = (int) round(100*0.005*N*135/2.3);         //中心座標轉成實際移動需要的步數
                //printf("center[%d] = ",y);
                //printf(" %d !!\n",center[y]);
                y+=1;

              }
            }
            int k = 0;
            for(int j = 18; j >=0;j--){ //花朵座標
                if( j%2 == 0){
		    centerdata[k] = center[j];
                    //printf("centerdata[%d] =",k);
                    //printf(" %d\n",centerdata[k]);
		    k+=1;
                }
            }
/**********************************************************************************/
	    i2cdata = k;
	    int x = 0;
	    for(int i = 0;i < 10; i++){
		if(centerdata[i] > 0){
			coordinate[x] = centerdata[i];
			x+=1;
		}
            }
	    printf("i2cdata =  %d\n",i2cdata);
            button = false;
        }//button
    }//camera start
 return 0;
}







