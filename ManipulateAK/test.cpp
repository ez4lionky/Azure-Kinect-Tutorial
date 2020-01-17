
#pragma once
#include <k4a/k4a.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  struct pcl::PointXYZ;
  struct pcl::PointXYZI;
  struct pcl::PointXYZRGB;
  struct pcl::PointXYZRGBA;
  template <typename T> class pcl::PointCloud;

  class KinectAzureDKGrabber : public pcl::Grabber
  {
  public:
    KinectAzureDKGrabber(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_);
    virtual ~KinectAzureDKGrabber() throw ();
    virtual void start();
    virtual void stop();
    virtual bool isRunning() const;
    virtual std::string getName() const;
    virtual float getFramesPerSecond() const;

    typedef void (signal_KinectAzureDK_PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
    typedef void (signal_KinectAzureDK_PointXYZI)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
    typedef void (signal_KinectAzureDK_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
    typedef void (signal_KinectAzureDK_PointXYZRGBA)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);
  
  protected:
    void setupDevice(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_);
    boost::signals2::signal<signal_KinectAzureDK_PointXYZ>* signal_PointXYZ;
    boost::signals2::signal<signal_KinectAzureDK_PointXYZI>* signal_PointXYZI;
    boost::signals2::signal<signal_KinectAzureDK_PointXYZRGB>* signal_PointXYZRGB;
    boost::signals2::signal<signal_KinectAzureDK_PointXYZRGBA>* signal_PointXYZRGBA;

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ();
    pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA();

    boost::thread thread;
    mutable boost::mutex mutex;

    void threadFunction();

    bool quit;
    bool running;

    k4a_device_configuration_t config;
    k4a::device dev;
    int device_id;

    k4a::calibration calibration;
    k4a::transformation transformation;

    int colorWidth;
    int colorHeight;
    k4a::image colorImage;

    int depthWidth;
    int depthHeight;
    k4a::image depthImage;

    int infraredWidth;
    int infraredHeight;
    k4a::image infraredImage;
  };

  pcl::KinectAzureDKGrabber::KinectAzureDKGrabber(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_) : 
    config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
    dev(nullptr),
    colorImage(nullptr),
    depthImage(nullptr),
    infraredImage(nullptr),
    running(false),
    quit(false),
    signal_PointXYZ(nullptr),
    signal_PointXYZI(nullptr),
    signal_PointXYZRGB(nullptr),
    signal_PointXYZRGBA(nullptr)
  {
    setupDevice(device_id_, depth_mode_, color_format_, color_resolution_);
    
    signal_PointXYZ     = createSignal<signal_KinectAzureDK_PointXYZ>();
    signal_PointXYZI    = createSignal<signal_KinectAzureDK_PointXYZI>();
    signal_PointXYZRGB  = createSignal<signal_KinectAzureDK_PointXYZRGB>();
    signal_PointXYZRGBA = createSignal<signal_KinectAzureDK_PointXYZRGBA>();
  }

  pcl::KinectAzureDKGrabber::~KinectAzureDKGrabber() throw()
  {
    stop();

    disconnect_all_slots<signal_KinectAzureDK_PointXYZ>();
    disconnect_all_slots<signal_KinectAzureDK_PointXYZI>();
    disconnect_all_slots<signal_KinectAzureDK_PointXYZRGB>();
    disconnect_all_slots<signal_KinectAzureDK_PointXYZRGBA>();

    thread.join();
    
    if (dev)
    {
      transformation.destroy();
      dev.close();
    }
  }

  void pcl::KinectAzureDKGrabber::start()
  {
    dev = k4a::device::open(device_id);
    dev.start_cameras(&config);
    calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
    transformation = k4a::transformation(calibration);

    running = true;
    
    thread = boost::thread(&KinectAzureDKGrabber::threadFunction, this);
  }

  void pcl::KinectAzureDKGrabber::stop()
  {
    boost::unique_lock<boost::mutex> lock(mutex);

    quit = true;
    running = false;

    lock.unlock();
  }

  bool pcl::KinectAzureDKGrabber::isRunning() const
  {
    boost::unique_lock<boost::mutex> lock(mutex);

    return running;

    lock.unlock();
  }

  std::string pcl::KinectAzureDKGrabber::getName() const
  {
    return std::string("KinectAzureDKGrabber");
  }

  float pcl::KinectAzureDKGrabber::getFramesPerSecond() const
  {
    return config.camera_fps;
  }

  void pcl::KinectAzureDKGrabber::setupDevice(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_)
  {
    device_id = device_id_;

    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = k4a_depth_mode_t(depth_mode_);//K4A_DEPTH_MODE_NFOV_UNBINNED
    config.color_format = k4a_image_format_t(color_format_);// K4A_IMAGE_FORMAT_COLOR_BGRA32
    config.color_resolution = k4a_color_resolution_t(color_resolution_);// K4A_COLOR_RESOLUTION_720P
    config.synchronized_images_only = true;  // ensures that depth and color images are both available in the capture
  }
  void pcl::KinectAzureDKGrabber::threadFunction()
  {
    while (!quit)
    {
      boost::unique_lock<boost::mutex> lock(mutex);
      k4a::capture capture;
      if (!dev.get_capture(&capture, std::chrono::milliseconds(0)))
      {
        continue;
      }

      depthImage = capture.get_depth_image();
      if (depthImage == nullptr)
      {
        throw std::exception("Failed to get depth image from capture\n");
      }  

      colorImage = capture.get_color_image();
      if (colorImage == nullptr)
      {
        throw std::exception("Failed to get color image from capture\n");
      }

      infraredImage = capture.get_ir_image();
      if (infraredImage == nullptr)
      {
        throw std::exception("Failed to get IR image from capture\n");
      }

      lock.unlock();

      if (signal_PointXYZ->num_slots() > 0) 
      {
        signal_PointXYZ->operator()(convertDepthToPointXYZ());
      }

      if (signal_PointXYZI->num_slots() > 0) 
      {
        signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI());
      }

      if (signal_PointXYZRGB->num_slots() > 0)
      {
        signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB());
      }

      if (signal_PointXYZRGBA->num_slots() > 0) 
      {
        signal_PointXYZRGBA->operator()(convertRGBADepthToPointXYZRGBA());
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::KinectAzureDKGrabber::convertDepthToPointXYZ(/*UINT16* depthBuffer*/)
  {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    int depth_image_width_pixels = depthImage.get_width_pixels();
    int depth_image_height_pixels = depthImage.get_height_pixels();

    cloud->width = depth_image_width_pixels;
    cloud->height = depth_image_height_pixels;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    k4a::image transformed_color_image = NULL;
    transformed_color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
      depth_image_width_pixels,
      depth_image_height_pixels,
      depth_image_width_pixels * 4 * (int)sizeof(uint8_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
      depth_image_width_pixels,
      depth_image_height_pixels,
      depth_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.color_image_to_depth_camera(depthImage, colorImage, &transformed_color_image);
    transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

    int width = point_cloud_image.get_width_pixels();
    int height = point_cloud_image.get_height_pixels();

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
      PointXYZ point;

      point.x = point_cloud_image_data[3 * i + 0];
      point.y = point_cloud_image_data[3 * i + 1];
      point.z = point_cloud_image_data[3 * i + 2];

      cloud->points[i] = point;
    }
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl::KinectAzureDKGrabber::convertInfraredDepthToPointXYZI(/*UINT16* infraredBuffer, UINT16* depthBuffer*/)
  {
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
    int depth_image_width_pixels = depthImage.get_width_pixels();
    int depth_image_height_pixels = depthImage.get_height_pixels();

    cloud->width = depth_image_width_pixels;
    cloud->height = depth_image_height_pixels;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    k4a::image transformed_color_image = NULL;
    transformed_color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
      depth_image_width_pixels,
      depth_image_height_pixels,
      depth_image_width_pixels * 4 * (int)sizeof(uint8_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
      depth_image_width_pixels,
      depth_image_height_pixels,
      depth_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.color_image_to_depth_camera(depthImage, colorImage, &transformed_color_image);
    transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

    int width = point_cloud_image.get_width_pixels();
    int height = point_cloud_image.get_height_pixels();

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    float *ir_image_data = reinterpret_cast<float *>(infraredImage.get_buffer());

    for (int i = 0; i < width * height; ++i)
    {
      PointXYZI point;

      point.x = point_cloud_image_data[3 * i + 0];
      point.y = point_cloud_image_data[3 * i + 1];
      point.z = point_cloud_image_data[3 * i + 2];
      point.intensity = ir_image_data[i];

      cloud->points[i] = point;
    }
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::KinectAzureDKGrabber::convertRGBDepthToPointXYZRGB(/*RGBQUAD* colorBuffer, UINT16* depthBuffer*/)
  {
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    int depth_image_width_pixels = depthImage.get_width_pixels();
    int depth_image_height_pixels = depthImage.get_height_pixels();
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
      color_image_width_pixels,
      color_image_height_pixels,
      color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
      color_image_width_pixels,
      color_image_height_pixels,
      color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = point_cloud_image.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
      PointXYZRGB point;

      point.x = point_cloud_image_data[3 * i + 0];
      point.y = point_cloud_image_data[3 * i + 1];
      point.z = point_cloud_image_data[3 * i + 2];
      
      point.b = color_image_data[4 * i + 0];
      point.g = color_image_data[4 * i + 1];
      point.r = color_image_data[4 * i + 2];
      uint8_t alpha = color_image_data[4 * i + 3];

      cloud->points[i] = point;
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::KinectAzureDKGrabber::convertRGBADepthToPointXYZRGBA(/*RGBQUAD* colorBuffer, UINT16* depthBuffer*/)
  {
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int depth_image_width_pixels = depthImage.get_width_pixels();
    int depth_image_height_pixels = depthImage.get_height_pixels();
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
      color_image_width_pixels,
      color_image_height_pixels,
      color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
      color_image_width_pixels,
      color_image_height_pixels,
      color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = point_cloud_image.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
      PointXYZRGBA point;

      point.x = point_cloud_image_data[3 * i + 0];
      point.y = point_cloud_image_data[3 * i + 1];
      point.z = point_cloud_image_data[3 * i + 2];

      point.b = color_image_data[4 * i + 0];
      point.g = color_image_data[4 * i + 1];
      point.r = color_image_data[4 * i + 2];
      point.a = color_image_data[4 * i + 3];

      cloud->points[i] = point;
    }
    return cloud;
  }
}