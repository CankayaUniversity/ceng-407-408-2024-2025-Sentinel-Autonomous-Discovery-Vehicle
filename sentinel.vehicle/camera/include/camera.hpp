#pragma once

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "observer.hpp"

#define LIBCAMERA

#ifdef LIBCAMERA
#include <libcamera/libcamera.h>
#endif

struct CameraConfiguration
{
  int width;
  int height;
  int fps;
#ifdef LIBCAMERA
  libcamera::PixelFormat pixel_format;
#endif
  std::string encoding;
};

#ifdef LIBCAMERA

class Camera : public CameraRequestSubject
{
public:
  Camera(const CameraConfiguration &camera_configuration);
  ~Camera();

public:
  void start();

private:
  void init_camera();
  void init_frame_allocator();
  void capture();
  void requestComplete(libcamera::Request *request);

private:
  const CameraConfiguration &camera_configuration;
  std::vector<int> compression_params;

private:
  std::shared_ptr<libcamera::CameraManager> camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::shared_ptr<libcamera::Request>> requests;
  std::unique_ptr<libcamera::CameraConfiguration> config;
  std::map<libcamera::FrameBuffer *, void *> buffer_mem;
};

#else

class Camera : public CameraRequestSubject
{
public:
  Camera(const CameraConfiguration &camera_configuration);
  ~Camera();

public:
  void start();

private:
  const CameraConfiguration &camera_configuration;
  cv::VideoCapture capture;
};
#endif