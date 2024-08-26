#include <iostream>
#include <thread>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <fstream>
#include <chrono>
#include <list>
#include <cstring>
#include <map>
#include <stdlib.h>
#include <string>
#include <ctime>

#include <opencv2/opencv.hpp>

#include "comms.h"

#include "camera_grab.h"
#include "file_io.h"
#include "params.h"

void usage()
{
    cout << "usage: MRR_Pi_client_2 [-r repeat_count]  [-f fps] [-p port_number] [-i ip_address] [-p port_number] [-i ip_address] ..." << endl;
    cout << endl;
    cout << "Sample MRR_Pi client code which sends images to one or more MRR_Pi servers." << endl;
    cout << "Each server is described by both a port_number and an ip_address," << endl;
    cout << "so the count of port_numbers must mach the count of ip_addresses," << endl;
    cout << "which are paired by their order in the command line." << endl;
    cout << "If both MRR_Pi_Client_2 and MRR_Pi_server_2 are on the same machine, use 127.0.0.1 as the ip address." << endl;
    cout << "Repeat_count defaults to 0 (loop forever), it is the total number of image files to send to each server, repeatedly picking from the 5 images in the 'raw' folder." << endl;
    cout << "Default fps is 30" << endl;
    cout << endl;

    cout << "sample command line (server is running on default port on localhost): ./MRR_Pi_client_2" << endl;
    cout << "sample command line (specify port and ip_address): ./MRR_Pi_client_2 -i 127.0.0.1 -p 5569" << endl;
    cout << "sample command line (two servers specified): ./MRR_Pi_client_2 -i 127.0.0.1 -p 5569 -i 127.0.0.1 -p 5570" << endl;
    cout << endl;
}

class CommPlus : public Comm
{
public:
    SD blocking_sd;
};

// used to create the subclass CommPlus instead of the default Comm class
Comm *comm_factory()
{
    return new CommPlus();
}

int Display_Test_Images(cv::Mat &Image_1, cv::Mat &Image_2)
{
    static bool displayImage = true;
    static bool displayMotion = true;
    cv::namedWindow("Test Webcam Feed", cv::WINDOW_AUTOSIZE);

    // opencv display stuff
    int key = cv::waitKey(1);
    if (key == 27) // ASCII code for the escape key
        return -1;

    if (key == 'i')
    {
        displayImage = !displayImage; // Toggle the flag
    }

    if (key == 'm')
    {
        displayMotion = !displayMotion; // Toggle the flag
    }

    // Debug Display  // display the image (not needed in final)
    if (displayImage)
    {
        cv::imshow("Test Webcam Feed", Image_1);
    }
    else if (cv::getWindowProperty("Test Webcam Feed", cv::WND_PROP_AUTOSIZE) != -1)
    {
        cv::destroyWindow("Test Webcam Feed"); // Close the window if the image is not displayed
    }

    if (displayMotion)
    {
        cv::imshow("Test frame_Abs_Diff Feed", Image_2);
    }
    else if (cv::getWindowProperty("Test frame_Abs_Diff Feed", cv::WND_PROP_AUTOSIZE) != -1)
    {
        cv::destroyWindow("Test frame_Abs_Diff Feed"); // Close the window if the image is not displayed
    }

    return 1;
}

int main(int argc, char *argv[])
{

    /***************************  MY CODE  **************************************/

    bool camera_good;
    auto loopStartTime = std::chrono::steady_clock::now();
    auto loopEndTime = std::chrono::steady_clock::now();
    auto ProcessStartTime = std::chrono::steady_clock::now();
    auto ProcessEndTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> ProcessTime = ProcessEndTime - ProcessStartTime;
    bool displayImage = true;
    bool displayMotion = true;
    bool Image_Motion = false;
    bool New_Frame = true;
    int Image_Status = -1;
    long loopCount = 0;
    Client_Parameters_Main Client_Params;

    readParametersFromFile("memory_params2.txt", Client_Params);

    std::string imageFile = "../../images/image.jpg"; // Path to your image file
    cv::Mat img = loadImage(imageFile);

    uchar gray_frame_raw[Client_Params.Screen_H_Size * Client_Params.Screen_V_Size];
    cv::Mat gray_frame(Client_Params.Screen_V_Size, Client_Params.Screen_H_Size, CV_8UC1, gray_frame_raw);   // Create an empty cv::Mat with the desired dimensions
    cv::Mat frame_Abs_Diff(Client_Params.Motion_Window_V_Size, Client_Params.Motion_Window_H_Size, CV_8UC1); // Create an empty cv::Mat with the desired dimensions
    std::vector<cv::Mat> Mats_5;

    cv::VideoCapture cap = InitWebCam(camera_good, Client_Params.Cam_H_Size, Client_Params.Cam_V_Size);

    cv::namedWindow("Test Webcam Feed", cv::WINDOW_AUTOSIZE);

    std::cout << " HERR " << getNextFileNameRaw("./raw/") << std::endl;
    std::cout << " HERR " << getNextFileNameTif("./tif/") << std::endl;

    /***************************  MY CODE  DONE  ********************************/

    float fps = .5; // was30  1.1 seconds per image
    long loop_count = 0;

    std::string connections[5] = {"x", "-i", "127.0.0.1", "-p", "5569"};

    char *argv_file[5];

    std::string strr;
    strr = "123";

    for (int i = 0; i < 5; i++)
    {
        argv_file[i] = new char[connections[i].length() + 1];
        strcpy(argv_file[i], connections[i].c_str());
    }

    for (int i = 0; i < 5; i++)
    {
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX argc " << argv_file[i] << " " << endl;
    }

    usage();

    auto blocking_send = Comm::NON_BLOCKING;

    list<Comm *> comms = Comm::start_clients(nullptr, 5, argv_file, comm_factory);
    if (comms.empty())
    {
        return -1;
    }

    // for debugging
    string files[] = {
        "../raw/24-06-03-04-30-10.raw",
        "../raw/24-06-03-04-31-09.raw",
        "../raw/24-06-03-04-35-02.raw",
        "../raw/24-06-03-04-32-06.raw",
        "../raw/24-06-03-04-37-06.raw"};

    auto files_len = sizeof(files) / sizeof(files[0]);
    // end debugging

    for (auto comm : comms)
    {
        comm->send_start_timer();
    }

    SD blocking_sd;
    SD loop_sd;
    long late_count = 0;
    auto begin = SteadyClock::now();
    long unack_count = 0;

    // for (long loop_count = 0; loop_count < ; loop_count++)
    while (true)
    {

        ProcessStartTime = std::chrono::steady_clock::now();

        Image_Status = get_camera_frame(cap,
                                        gray_frame,
                                        frame_Abs_Diff,
                                        Client_Params.Cycle_Time,
                                        Client_Params.Motion_Window_H_Position,
                                        Client_Params.Motion_Window_V_Position,
                                        Client_Params.Noise_Threshold,
                                        Client_Params.Motion_Threshold);

        Image_Motion = (Image_Status == 1);
        New_Frame = (Image_Status >= 0);

        // sets the timing of the images presented and stores the image if it moved
        Sequencer(Image_Motion, gray_frame);

        list<string> images_to_send_2;
        list<string> names_to_send_2;
        ostringstream string_stream;


        // pull out of loop

        for(int i=0; i<5; i++)
        {
            names_to_send_2.push_back("Screen X" + to_string(i) ) ;
        }
        
        // store all the images ready to send
        if (Image_Status >= 0)
        {
            string_stream.write(reinterpret_cast<const char*>(gray_frame.data), gray_frame.total() * gray_frame.elemSize());
            images_to_send_2.push_back(string_stream.str());

            if (images_to_send_2.size() > 5)
            {
                images_to_send_2.resize(5);
            }

            // for test viewing
            // Mats_5.push_back(gray_frame);
            // if (Mats_5.size() > 5)
            // {
            //     Mats_5.resize(5);
            // }
        }


        // now send
        if (New_Frame)
        {
            for (auto &comm : comms)
            {
                string image_data = images_to_send_2.front();
                // images_to_send.pop_front();
                string send_name = names_to_send_2.front();
                // names_to_send.pop_front();

                comm->send_image(send_name, image_data);
            }
        }

        ProcessEndTime = std::chrono::steady_clock::now();

        ProcessTime = ProcessEndTime - ProcessStartTime;

        // if(true)
        if( ProcessTime.count()  > .005)
             std::cout << "ProcessTime: " << ProcessTime.count() << std::endl;



        // sets the timing of a frame  1/30th
        loopEndTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = loopEndTime - loopStartTime;
        while (elapsed_seconds.count() < .0333333)
            elapsed_seconds = std::chrono::steady_clock::now() - loopStartTime;
        loopStartTime = std::chrono::steady_clock::now();

        if (Display_Test_Images(gray_frame, frame_Abs_Diff) == -1)
            break;

        loop_count++;

    }

    // give the server time to process the last sends before the connection is dropped
    this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
