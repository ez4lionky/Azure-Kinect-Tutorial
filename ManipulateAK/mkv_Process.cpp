/*
** This file is tightly coupled with the code in the Azure Kinect Sensor SDK samples,
** https://github.com/microsoft/Azure-Kinect-Samples
** such as k4a_playback_open, k4a_playback_get_calibration,
** k4a_playback_get_next_capture functions, and etc.
** Recommended using ManipulateAK.cpp
*/
using namespace std;

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>
#include <k4abt.h>

static cv::Mat color_to_opencv(const k4a::image &im)
{
    cv::Mat cv_image_with_alpha(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void *)im.get_buffer());
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

static cv::Mat depth_to_opencv(const k4a::image &im)
{
    return cv::Mat(im.get_height_pixels(),
                   im.get_width_pixels(),
                   CV_16U,
                   (void *)im.get_buffer(),
                   static_cast<size_t>(im.get_stride_bytes()));
}

static k4a::image create_depth_image_like(const k4a::image &im)
{
    return k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                              im.get_width_pixels(),
                              im.get_height_pixels(),
                              im.get_width_pixels() * static_cast<int>(sizeof(uint16_t)));
}

bool check_data_exists(k4a_capture_t capture, const string data_type)
{
    k4a_image_t data;
    if (!strcmp(data_type.c_str(), "depth"))
        data = k4a_capture_get_depth_image(capture);
    else if (!strcmp(data_type.c_str(), "color"))
        data = k4a_capture_get_color_image(capture);

    if (data != nullptr)
    {
        k4a_image_release(data);
        return true;
    }
    else
    {
        return false;
    }
}

bool process_mkv_offline(const char *input_path, const char *output_path)
{
    uint16_t depth_threshold = 1000;
    k4a_playback_t playback_handle = nullptr;
    k4a_result_t result = k4a_playback_open(input_path, &playback_handle);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Cannot open recording at " << input_path << endl;
        return false;
    }

    k4a_calibration_t calibration;
    result = k4a_playback_get_calibration(playback_handle, &calibration);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Failed to get calibration" << endl;
        return false;
    }
    k4a::transformation depth_to_color(calibration);

    cout << "Reading RGB frame and depth data" << input_path << endl;

    int frame_count = 0;
    bool success = true;

    while (true)
    {
        k4a_capture_t capture_handle = nullptr;
        k4a_image_t colorImage_handle = nullptr, depthImage_handle = nullptr;
        k4a_stream_result_t stream_result = k4a_playback_get_next_capture(playback_handle, &capture_handle);
        k4a::image colorImage, depthImage;
        cv::Mat depthFrame, colorFrame;

        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            break;
        }

        cout << "frame " << frame_count << '\r';
        if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
        {
            if (check_data_exists(capture_handle, "depth") && check_data_exists(capture_handle, "color"))
            {
                colorImage_handle = k4a_capture_get_color_image(capture_handle);
                colorImage = k4a::image(colorImage_handle);
                depthImage_handle = k4a_capture_get_depth_image(capture_handle);
                depthImage = k4a::image(depthImage_handle);

                k4a::image depth_in_color = create_depth_image_like(colorImage);
                // Create a blank image size like colorImage, which used as depth_in_color image
                depth_to_color.depth_image_to_color_camera(depthImage, &depth_in_color);
                // Transforms the depth map into the geometry of the color camera.
                // depth_in_color is the transformed_depth_image

                cv::Mat depthFrame = depth_to_opencv(depth_in_color);
                cv::Mat colorFrame = color_to_opencv(colorImage);

                cv::Mat within_threshold_range = (depthFrame != 0) &
                                             (depthFrame < depth_threshold);
                colorFrame.copyTo(colorFrame, within_threshold_range);
                imshow("Depth image of frame: " + to_string(frame_count), depthFrame);
                imshow("Color image of frame: " + to_string(frame_count), colorFrame);

                cvWaitKey(0);
                k4a_capture_release(capture_handle);
            }
        }
        else
        {
            success = false;
            cerr << "Stream error for clip at frame " << frame_count << endl;
            break;
        }
        frame_count++;
    }
};

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage: k4abt_offline_processor.exe <input_mkv_file> <output_path>" << endl;
        return -1;
    }

    return process_mkv_offline(argv[1], argv[2]) ? 0 : -1;
}