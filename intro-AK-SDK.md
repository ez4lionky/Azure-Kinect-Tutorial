## The introduction of Azure Kinect SDK

#### The pipeline of Azure Kinect and related code
Pipeline of rgb-d data collection: 
Power on = startDevice() -> 
Capture RGB-D data = k4a_capture_t capture_handle (A capture represents a set of images that were captured by a device at approximately the same time. A capture may have a color, IR, and depth image, [link](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/release/1.3.x/structk4a__capture__t.html#details)) ->
After 


