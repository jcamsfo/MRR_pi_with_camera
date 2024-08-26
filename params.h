
#ifndef PARAMS_H
#define PARAMS_H




struct Client_Parameters_Main
{
    int Cam_H_Size;
    int Cam_V_Size;
    int Screen_H_Size;
    int Screen_V_Size;

    int Motion_Window_H_Size_Multiplier;
    int Motion_Window_V_Size_Multiplier;

    float Cycle_Time;

    int Noise_Threshold;
    int Motion_Threshold;

    std::string param3;
    // std::string param4;

    // Constructor to initialize default values
    Client_Parameters_Main() : Cam_H_Size(800), Cam_V_Size(600),
                               Screen_H_Size(1024), Screen_V_Size(768),
                               Motion_Window_H_Size_Multiplier(75), Motion_Window_V_Size_Multiplier(75), Cycle_Time(1.2), Noise_Threshold(5), Motion_Threshold(5000) {}

    int Motion_Window_H_Size = (Screen_H_Size * Motion_Window_H_Size_Multiplier) / 100;
    int Motion_Window_V_Size = (Screen_V_Size * Motion_Window_V_Size_Multiplier) / 100;

    int Motion_Window_H_Position = (Screen_H_Size - Motion_Window_H_Size) / 2;
    int Motion_Window_V_Position = (Screen_V_Size - Motion_Window_V_Size) / 2;
};



void readParametersFromFile(const std::string &filename, Client_Parameters_Main &params);


void Sequencer(const bool Image_Motion, const cv::Mat &gray_frame_local);



#endif // PARAMS_H