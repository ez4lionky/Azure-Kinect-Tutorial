## The introduction of Azure Kinect SDK

#### The pipeline of Azure Kinect and related code
Pipeline of rgb-d data collection: 

1. refer to [blog](https://cloud.tencent.com/developer/article/1535784), collect data in real-time.
Power on = k4a::device::open() -> dev.start_cameras()
Capture RGB-D data = k4a_capture_t capture_handle (A capture represents a set of images that were captured by a device at approximately the same time. A capture may have a color, IR, and depth image, [link](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/release/1.3.x/structk4a__capture__t.html#details)) ->
Write color image and depth image = capture.get_color_image() / capture.get_depth_image() -> cv::imwrite() ->
Power off = dev.close()
(The code is in file ManipulateAK.cpp, but it is not complete)
2. refer to [official example](https://github.com/microsoft/Azure-Kinect-Samples/tree/master/body-tracking-samples/offline_processor), using playback() function to read mkv file.
(The code is in file mkv_Process.cpp, but the result is strange)

#### The easier way

1. Recording rgbd data by using Azure Kinect SDK -> .mkv file
2. Using [API](http://www.open3d.org/docs/latest/tutorial/Basic/azure_kinect.html?highlight=azure%20kinect) (azure_kinect_mkv_reader.py) of python package Open3D to process mkv file.
3. Using reconstruction [module](http://www.open3d.org/docs/latest/tutorial/ReconstructionSystem/capture_your_own_dataset.html#) in Open3D.
Open3D scripts will generate fragments and scene folder under the path_dataset (which is specified in the config.json file)
For reconstruction, [tsdf-fusion](https://github.com/andyzeng/tsdf-fusion-python) also can be used, but need to have related RT. The module in Open3D is recommended.

