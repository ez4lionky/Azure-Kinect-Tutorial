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

using DepthPixel = uint16_t;

struct Pixel
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alhpa;
};

using DepthPixelVisualizationFunction = Pixel(const DepthPixel &value, const DepthPixel &min, const DepthPixel &max);

inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode)
{
    switch (depthMode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return {(uint16_t)500, (uint16_t)5800};
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return {(uint16_t)500, (uint16_t)4000};
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return {(uint16_t)2500, (uint16_t)3000};
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return {(uint16_t)250, (uint16_t)2500};

    case K4A_DEPTH_MODE_PASSIVE_IR:
    default:
        throw logic_error("Invalid depth mode!");
    }
}

inline void ColorConvertHSVtoRGB(float h, float s, float v, float &out_r, float &out_g, float &out_b)
{
    if (s == 0.0f)
    {
        out_r = out_g = out_b = v;
        return;
    }

    h = fmodf(h, 1.0f) / (60.0f / 360.0f);
    int i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (i)
    {
    case 0:
        out_r = v;
        out_g = t;
        out_b = p;
        break;
    case 1:
        out_r = q;
        out_g = v;
        out_b = p;
        break;
    case 2:
        out_r = p;
        out_g = v;
        out_b = t;
        break;
    case 3:
        out_r = p;
        out_g = q;
        out_b = v;
        break;
    case 4:
        out_r = t;
        out_g = p;
        out_b = v;
        break;
    case 5:
    default:
        out_r = v;
        out_g = p;
        out_b = q;
        break;
    }
}

Pixel ColorizeBlueToRed(const DepthPixel &depthPixel, const DepthPixel &min, const DepthPixel &max)
{
    constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
    Pixel result = {
        uint8_t(0), uint8_t(0), uint8_t(0), PixelMax};
    if (depthPixel == 0)
    {
        return result;
    }
    uint16_t clampedValue = depthPixel;
    clampedValue = std::min(clampedValue, max);
    clampedValue = std::max(clampedValue, min);

    float hue = (clampedValue - min) / static_cast<float>(max - min);

    constexpr float range = 2.f / 3.1;
    hue *= range;

    hue = range - hue;

    float fRed = 0.f;
    float fGreen = 0.f;
    float fBlue = 0.f;
    ColorConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);

    result.Red = static_cast<uint8_t>(fRed * PixelMax);
    result.Green = static_cast<uint8_t>(fGreen * PixelMax);
    result.Blue = static_cast<uint8_t>(fBlue * PixelMax);

    return result;
}

void ColorizeDepthImage(const k4a::image &depthImage,
                        DepthPixelVisualizationFunction visualizationFn,
                        std::pair<uint16_t, uint16_t> expectedValueRange,
                        std::vector<Pixel> *buffer)
{
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }
    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();
    buffer->resize(static_cast<size_t>(width * height));
    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t currentPixel = static_cast<size_t>(h * width + w);
            (*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
                                                      expectedValueRange.first, expectedValueRange.second);
        }
    }
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
        // std::vector<Pixel> depthTextureBuffer;

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
                // depthImage_handle = k4a_capture_get_depth_image(capture_handle);
                // depthImage = k4a::image(depthImage_handle);

                // ColorizeDepthImage(depthImage,
                //                    ColorizeBlueToRed,
                //                    GetDepthModeRange(K4A_DEPTH_MODE_NFOV_UNBINNED),
                //                    &depthTextureBuffer);

                uint8_t *colorTextureBuffer = colorImage.get_buffer();

                // cv::Mat depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, &depthTextureBuffer[0]);
                cv::Mat colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);

                // imshow("Depth image of frame: " + to_string(frame_count), depthFrame);
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

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage: k4abt_offline_processor.exe <input_mkv_file> <output_path>" << endl;
        return -1;
    }

    return process_mkv_offline(argv[1], argv[2]) ? 0 : -1;
}