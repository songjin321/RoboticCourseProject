///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV. 					  	      **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

/**
 * OpenCV video streaming over TCP/IP
 * Server: Captures video from a webcam and send it to a client
 * by Isaac Maia
 */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#include <ros/ros.h>
using namespace cv;

void display(void *);

// Open the ZED camera
VideoCapture cap(0);

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "vr_zed");
    //--------------------------------------------------------
    //networking stuff: socket, bind, listen
    //--------------------------------------------------------
    int     localSocket,
            remoteSocket,
            port = 4097;

    struct  sockaddr_in localAddr,
            remoteAddr;
    pthread_t thread_id;


    int addrLen = sizeof(struct sockaddr_in);


    if ( (argc > 1) && (strcmp(argv[1],"-h") == 0) ) {
        std::cerr << "usage: ./cv_video_srv [port] [capture device]\n" <<
                  "port           : socket port (4097 default)\n" <<
                  "capture device : (0 default)\n" << std::endl;

        exit(1);
    }

    if (argc == 2) port = atoi(argv[1]);

    localSocket = socket(AF_INET , SOCK_STREAM , 0);
    if (localSocket == -1){
        perror("socket() call failed!!");
    }

    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons( port );

    if( bind(localSocket,(struct sockaddr *)&localAddr , sizeof(localAddr)) < 0) {
        perror("Can't bind() socket");
        exit(1);
    }
    // set send buff 120000
    int sendbuf = 120000;
    int len = sizeof(sendbuf);
    setsockopt(localSocket, SOL_SOCKET, SO_RCVBUF, &sendbuf, sizeof(sendbuf));
    getsockopt(localSocket, SOL_SOCKET, SO_RCVBUF, &sendbuf, (socklen_t*)&len);
    std::cerr << "the send buff size after set is " << sendbuf <<  std::endl;
    //Listening
    listen(localSocket , 3);

    std::cout <<  "Waiting for connections...\n"
              <<  "Server Port:" << port << std::endl;


    if(!cap.isOpened())
        return -1;
    // Set the video resolution to HD720 (2560*720)
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    //int optVal = 1000000;
    //setsockopt(localSocket,SOL_SOCKET,SO_SNDBUF,(char *)&optVal,sizeof(int));


    //accept connection from an incoming client
    while(1){
        //if (remoteSocket < 0) {
        //    perror("accept failed!");
        //    exit(1);
        //}

        remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);
        //std::cout << remoteSocket<< "32"<< std::endl;
        if (remoteSocket < 0) {
            perror("accept failed!");
            exit(1);
        }
        std::cout << "Connection accepted" << std::endl;
        //pthread_create(&thread_id,NULL,display,&remoteSocket);
        display(&remoteSocket);
        //pthread_join(thread_id,NULL);

    }
    //pthread_join(thread_id,NULL);
    close(remoteSocket);

    return 0;
}

void display(void *ptr){
    int socket = *(int *)ptr;
    //OpenCV Code
    //----------------------------------------------------------
    int imgSize = 0;
    int bytes = 0;
    int key;

    Mat frame, left, right;
    frame = Mat::zeros(720 , 2560, CV_8UC3);
    //make it continuous
    if (!frame.isContinuous()) {
        frame = frame.clone();
    }

    while(1) {
        // Get a new frame from camera
        clock_t begin = clock();
        cap >> frame;
        left = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        right = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        //do video processing here
        vector<uchar> buf_left;
        vector<uchar> buf_right;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(70);
        if(!imencode(".jpg", left, buf_left, compression_params))
        {
            std::cout << "compress image failed!" << std::endl;
        }
        if(!imencode(".jpg", right, buf_right, compression_params))
        {
            std::cout << "compress image failed!" << std::endl;
        }
        //resize(frame, out_image, cv::Size(), 0.5, 0.5);
        //imgSize = out_image.total() * out_image.elemSize();
        imgSize = buf_left.size();
        bytes = send(socket, &imgSize, sizeof(imgSize), 0);
        std::cerr << "left_size = " << imgSize << std::endl;
        bytes = send(socket, buf_left.data(), imgSize, 0);
        std::cerr << "left_bytes = " << bytes << std::endl;
        //cv::imshow("left", left);
        imgSize = buf_right.size();
        bytes = send(socket, &imgSize, sizeof(imgSize), 0);
        std::cerr << "right_size = " << imgSize << std::endl;
        bytes = send(socket, buf_right.data(), imgSize, 0);
        std::cerr << "right_bytes = " << bytes << std::endl;
        if (bytes < 0){
            break;
        }

        clock_t end = clock();
        double elapsed_time = (double)(end - begin)/CLOCKS_PER_SEC * 1000;
        std::cout << "one loop elapsed time : " << elapsed_time << std::endl;
        if (30 - elapsed_time > 0 )
            usleep(30 - elapsed_time);
    }

}
