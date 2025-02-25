#pragma once

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "observer.hpp"

#ifdef LIBCAMERA
#include <libcamera/libcamera.h>

struct CameraConfiguration
{
  CameraConfiguration(int width, int height, int fps, const std::string &encoding, libcamera::PixelFormat pixel_format);
  int width;
  int height;
  int fps;
  std::string encoding;
  libcamera::PixelFormat pixel_format;
};

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

struct CameraConfiguration
{
  CameraConfiguration(int width, int height, int fps, const std::string &encoding);
  int width;
  int height;
  int fps;
  std::string encoding;
};

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