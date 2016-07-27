
//standard includes
#include <cstdio>
#include <cstring>
#include <signal.h>
#include <cstdlib>
#include <chrono>
#include <thread>


//Opencv Include (for display
#include <opencv2/opencv.hpp>

//ZED Include
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>


#ifdef _WIN32
#include <windows.h>
#endif

using namespace sl::zed;
using namespace std;

Camera* zed_ptr;

#ifdef _WIN32

BOOL CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
            // Handle the CTRL-C signal.
        case CTRL_C_EVENT:
            printf("\nSaving file...\n");
            zed_ptr->stopRecording();
            delete zed_ptr;
            exit(0);
        default:
            return FALSE;
    }
}
#else

void nix_exit_handler(int s) {
    printf("\nSaving file...\n");
    zed_ptr->stopRecording();
    delete zed_ptr;
    exit(1);
}
#endif

int main(int argc, char **argv) {
  sleep(3);
  std::string filename = "/home/ubuntu/Videos/zed_record_2.svo";

    bool display = 0;
    int resolution = 2; //Default resolution is set to HD720
   
    // Camera init
    ERRCODE err;
    // Create the camera at HD 720p resolution
    // The realtime recording will depend on the write speed of your disk.
    Camera* zed = new Camera(static_cast<ZEDResolution_mode> (resolution));
    // ! not the same Init function - a more lighter one , specific for recording !//
    err = zed->initRecording(filename);

    zed_ptr = zed; // To call Camera::stop_recording() from the exit handler function

    std::cout << "ERR code : " << errcode2str(err) << std::endl;

    // Quit if an error occurred
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }

    // CTRL-C (= kill signal) handler
#ifdef _WIN32
    SetConsoleCtrlHandler((PHANDLER_ROUTINE) CtrlHandler, TRUE);
#else // unix
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
#endif

    // Wait for the auto exposure and white balance
    std::this_thread::sleep_for(std::chrono::seconds(1));
	
    // Recording loop
    cout << "Recording..." << endl;
    cout << "Press 'Ctrl+C' to stop and exit " << endl;
    while (1) {
        //simple recording function
        bool test = zed->record(); // record the current frame with the predefined size

        //if (display && !test) zed->displayRecorded(); // convert the image to RGB and display it
    }

    return 0;
}

  
  
  
