#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <libMesaSR.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <vector>

int main(void)
{
        printf("ORF pid %d\n", (int) getpid());
        SRCAM cam;
        // int ret = SR_OpenDlg(&cam, 2, 0); // 2: call open dialog, 0: no parent window
        int ret = SR_OpenETH(&cam, "192.168.1.33");
        if(ret<=0) return -1; // ret holds the number of cameras found
        cv::Size imsize(SR_GetCols(cam), SR_GetRows(cam)); // SR image size
        int sizebytes = 2 * imsize.area() * sizeof(unsigned short); // number of bytes sent from the SR 
        // namedWindow("mainWin",WINDOW_AUTOSIZE );
        cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
        int sizestep = sizeof(float)*3; // size step from one xyz component to the next
        int c=-1; // user input variable
        // enable software trigger mode so that the LEDs will only turn
        // on when SR_Acquire() is called
        SR_SetMode(cam,AM_SW_TRIGGER);
        while(c==-1) // infinite loop, breaks if key pressed
        {
                ret = SR_Acquire(cam);
                cv::Mat xyz(imsize, CV_32FC3, cv::Scalar::all(0));
                if(ret!=sizebytes) break;
                // the coordinates are stored as three channels in the format
                // (x1, y1, z1, x2, y2, z2, ... ) squentially row by row
                SR_CoordTrfFlt( cam,
                                &((float*)xyz.ptr())[0], // pointer to first x
                                &((float*)xyz.ptr())[1], // pointer to first y
                                &((float*)xyz.ptr())[2], // pointer to first z
                                sizestep, sizestep, sizestep); // increments to next element

                cv::Mat z, z_display; // z channel and output image
                extractImageCOI(&CvMat(xyz), z, 2); // extract the z channel (change the 2 for another channel)
                z.convertTo(z_display, CV_8UC1, 256.0 / 5.0, 0); // convert to 8 bit (0..255) values, here for 5 meter camera
                cv::imshow("mainWin", z_display); // display image
                c = cvWaitKey(1000); // wait 1 sec before continuing loop. if user presses a button, the loop will exit
        }
        SR_Close(cam);
        return 0;
}

// this can be used to record a stream and save it to a file
// int SR_StreamToFile(SRCAM srCam, const char *filename, int mode)
