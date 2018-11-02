#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <highgui.h>
/* opencv */
#include "math.h"
#include "cv.h"
#include "highgui.h"

/* raspicam setting */
static pid_t pid = 0;


/* raspicam */
void getPicture(char *filename, char *options) {
    if ((pid = fork()) == 0) {
        char **cmd;

        // count tokens in options string
        int count = 0;
        char *copy;
        copy = strdup(options);
        if (strtok(copy, " \t") != NULL) {
            count = 1;
            while (strtok(NULL, " \t") != NULL)
                count++;
        }

        cmd = malloc((count + 8) * sizeof(char **));
        free(copy);

        // if any tokens in options,
        // copy them to cmd starting at positon[1]
        if (count) {
            int i;
            copy = strdup(options);
            cmd[1] = strtok(copy, " \t");
            for (i = 2; i <= count; i++)
                cmd[i] = strtok(NULL, " \t");
        }

        // add default options
        cmd[0] = "raspivid"; // executable name
        cmd[count + 1] = "-n"; // no preview
        cmd[count + 2] = "-t"; // default time (overridden by -s)
                               // but needed for clean exit
        cmd[count + 3] = "0"; // 10 millisecond (minimum) time for -t
        cmd[count + 4] = "-w";
        cmd[count + 5] = "640";
        cmd[count + 6] = "-h";
        cmd[count + 7] = "480";
        //cmd[count + 8] = "-o"; // output file specifer
        //cmd[count + 9] = filename;
        cmd[count + 8] = (char *)0; // terminator
        execv("/usr/bin/raspivid", cmd);
    }

}
float distance(IplImage* img)
{

    //CvMoments moments;
    //cvMoments(img, &moments,0);
    //double area = cvGetCentralMoment(&moments, 0, 0);
    //double moment10 = cvGetSpatialMoment(&moments, 1, 0);
    //double moment01 = cvGetSpatialMoment(&moments, 0, 1);


    /* find the circle */

    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* results = cvHoughCircles(img,storage, CV_HOUGH_GRADIENT, 2, img->height, 10, 5, 10,200);
    //printf("circle number: %d\n",results->total);

    float angle = 180;

    //for (int i = 0; i< results->total;i++)
    if(results->total != 0)
    //if(area > 1000)
    {
	float* p= (float*) cvGetSeqElem(results, 0);
	//int posX = moment10/area;
	//int posY = moment01/area;
        //printf("p[0]:%d, p[1]:%d\n",cvRound(p[0]),cvRound(p[1]));
	//printf("width:%d, height:%d",img->width,img->height);
	//printf("position:%d,%d",cvRound(p[0])-img->width/2,img->height-cvRound(p[1]));
	float parm = ((float)cvRound(p[0])-img->width/2)/(img->width/2); //((float)img->height-cvRound(p[1]));
	//printf("parm %f\n",parm);
	//right+left
	//float pi = 3.14159;
	//angle = ((float)posX-img->width/2)/((float)img->width/2);
	//angle = ((float)posX-img->width/2)/((float)img->height - posY);
	//angle = atan(parm)*180/pi*25/90;
	angle = parm * 60;
	//angle = ((float)cvRound(p[0])-img->width/2)/((float)img->width/2) * 30.0;
    }

	//printf("angle:%f",angle);
    return angle;
}
int main(int argc, char **argv) {

    IplImage *frame = NULL;
    CvCapture *capture;
    IplImage* hsv;
    IplImage* colorR;
    IplImage* colorB;
    IplImage* colorG;
    float angleR;
    float angleG;
    float angleB;
    FILE *fp;
    char state[10];
    int check = 0;
    int key;
    int i = -1;
    for (i;i<100;i++){
    capture = cvCaptureFromCAM(i);
    if(!capture){
        printf("ERROR: caputre is NULL\n");

    }else
    {
        printf("succeed\n");
        break;}
    }

    //cvNamedWindow("myWindow",CV_WINDOW_AUTOSIZE);
    //cvNamedWindow("myImage",CV_WINDOW_AUTOSIZE);
    while(1){
    frame = cvQueryFrame(capture);
    cvGrabFrame(capture);
    frame = cvRetrieveFrame(capture,0);

    if(! frame){
        fprintf(stderr, "ERROR: frame is null... \n");
        getchar();
    }


    // turn to HSV images

    if(!check){
        hsv = cvCreateImage(cvSize(frame->width,frame->height),8,3);
        cvCvtColor(frame,hsv,CV_BGR2HSV);
        colorG = cvCreateImage(cvSize(frame->width,frame->height),8,1);
        colorB = cvCreateImage(cvSize(frame->width,frame->height),8,1);
        colorR = cvCreateImage(cvSize(frame->width,frame->height),8,1);
        check = 1;
    }
    else
        cvCvtColor(frame,hsv,CV_BGR2HSV);
    // color mask
    cvInRangeS(hsv,cvScalar(46,90,80,0), cvScalar(77,255,255,0),colorG);
    cvInRangeS(hsv,cvScalar(90,90,80,0), cvScalar(124,255,255,0),colorB);
    //cvInRangeS(hsv,cvScalar(26,70,80,0), cvScalar(34,255,255,0),colorR);

    //post processing
    //cvErode(colorR,colorR,NULL,6);
    //cvDilate(colorR,colorR,NULL,2);
    cvErode(colorG,colorG,NULL,6);
    cvDilate(colorG,colorG,NULL,2);
    cvErode(colorB,colorB,NULL,6);
    cvDilate(colorB,colorB,NULL,2);




    //angleR = distance(colorR);
    angleB = distance(colorB);
    angleG = distance(colorG);


    // check if there are any box
    fp = fopen("box.txt","w+");
    if(angleB != 180)
    {
    	//printf("Box is detected\n");
        strcpy(state,"box");
        //write to file
        fprintf(fp,"%s %f",state,angleB);
    }else
        fprintf(fp,"");

    fclose(fp);

    // check if there are any ball
    fp = fopen("ball.txt","w+");
    if(angleG != 180){
        //printf("Blue ball is detected\n");
        strcpy(state,"green");
        fprintf(fp,"%s %f",state,angleG);
        //}
    //else if(abs(angleG)<abs(angleB)){
        //printf("Green ball is detected\n");
      //  strcpy(state,"green");
        //fprintf(fp,"%s %f",state,angleG);
    }else
        fprintf(fp,"");

    fclose(fp);

    //printf("angle:%f\n",angleR);
    //cvShowImage("myWindow",frame);
    //cvShowImage("myImage",colorB);
    key = cvWaitKey(100);
    }
    cvReleaseImage(&hsv);
    cvReleaseCapture(&capture);
    cvReleaseImage(&colorG);
    cvReleaseImage(&colorB);
    cvReleaseImage(&colorR);
    cvReleaseImage(&frame);
    //cvDestroyWindow("myWindow");
    //cvDestroyWindow("myImage");
    return 0;

}
