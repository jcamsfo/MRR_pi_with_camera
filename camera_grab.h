
#ifndef CAMERA_GRAB_H
#define CAMERA_GRAB_H

#include <opencv2/opencv.hpp>



// Loads a grayscale image
// cv::Mat loadImage(const std::string &imageFile);

cv::VideoCapture InitWebCam(bool &valid_cam, int Horizontal_Res, int Vertical_Res);

int get_camera_frame(cv::VideoCapture &capture, cv::Mat &gray_frame_local, cv::Mat &diff_frame,
                       const float Cycle_Time_In, const int Motion_H_Pos, const int Motion_V_Pos,
                       const int Noise_Thresh, const int Motion_Thresh);                            


#endif // CAMERA_GRAB_H