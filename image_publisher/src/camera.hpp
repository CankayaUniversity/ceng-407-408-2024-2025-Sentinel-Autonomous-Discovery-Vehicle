#pragma once

#include <memory>
#include <libcamera/libcamera.h>
#include <map>

#include "observer.hpp"

class Camera : public RequestSubject
{
public:
  Camera();
  ~Camera();

public:
  void start();

private:
  void init_camera();
  void init_frame_allocator();
  void capture();
  void requestComplete(libcamera::Request *request);

private:
  std::shared_ptr<libcamera::CameraManager> camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::shared_ptr<libcamera::Request>> requests;
  std::unique_ptr<libcamera::CameraConfiguration> config;
  std::map<libcamera::FrameBuffer *, void *> buffer_mem;
};