//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <thread>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ZED Includes
#include <zed/Camera.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include "ORB_SLAM/System.h"

using namespace sl::zed;
using namespace std;

const int FPS = 30;
const ZEDResolution_mode ZED_RES = VGA;

Camera* zed;
cv::Mat Left;
cv::Mat Depth;
cv::Mat Right;
int width, height;

bool stop_signal;

void grab_run()
{
    while (!stop_signal)
    {
        bool res = zed->grab(SENSING_MODE::STANDARD, 1, 1);

        if (!res)
        {
            slMat2cvMat(zed->retrieveImage(SIDE::LEFT)).copyTo(Left);
            slMat2cvMat(zed->normalizeMeasure(MEASURE::DEPTH)).copyTo(Depth);
            slMat2cvMat(zed->retrieveImage(SIDE::RIGHT)).copyTo(Right);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    delete zed;
}

int main()
{

    zed = new Camera(ZED_RES, FPS);

    InitParams parameters;
    parameters.mode = QUALITY;
    parameters.unit = MILLIMETER;
    parameters.verbose = 1;

    ERRCODE err = zed->init(parameters);

    width = zed->getImageSize().width;
    height = zed->getImageSize().height;
    Left = cv::Mat(height, width, CV_8UC4, 1);
    Depth = cv::Mat(height, width, CV_8UC4, 1);
    Right = cv::Mat(height, width, CV_8UC4, 1);

    char key = ' ';

    std::thread grab_thread(grab_run);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("../Vocabulary/ORBvoc.bin","../zed_ygx.yaml",ORB_SLAM2::System::STEREO,true);

    cv::Mat frame_left;
    cv::Mat frame_depth;
    cv::Mat frame_right;

    while (key != 'q')
    {
        cvtColor(Left, frame_left, CV_BGRA2BGR);
        cvtColor(Depth, frame_depth, CV_BGRA2BGR);
        cvtColor(Right, frame_right, CV_BGRA2BGR);

        //cv::imshow("Left", Left);
        //cv::imshow("Depth", frame_depth);
        //cv::imshow("Right", Right);

        // Pass the image to the SLAM system
        SLAM.TrackStereo(frame_left, frame_right, 1);

        key = cv::waitKey(5);
    }

    // Stop all threads
    SLAM.Shutdown();

    stop_signal = true;

    grab_thread.join();

    return 0;
}
