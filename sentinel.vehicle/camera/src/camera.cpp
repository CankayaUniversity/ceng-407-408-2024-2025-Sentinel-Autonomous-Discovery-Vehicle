#include "camera.hpp"

#ifdef LIBCAMERA

Camera::Camera(const CameraConfiguration &camera_configuration) : camera_configuration{camera_configuration}
{
  this->init_camera();
  this->init_frame_allocator();
  this->capture();
}

Camera::~Camera()
{
  libcamera::StreamConfiguration &stream_config = this->config->at(0);
  libcamera::Stream *stream = stream_config.stream();
  this->camera->stop();
  this->allocator->free(stream);
  this->camera->release();
  this->camera_manager->stop();
}

void Camera::start()
{
  this->camera->requestCompleted.connect(this, &Camera::requestComplete);
  this->camera->start();
  for (auto &request : this->requests)
  {
    this->camera->queueRequest(request.get());
  }
}

void Camera::init_camera()
{
  this->camera_manager = std::make_shared<libcamera::CameraManager>();
  if (this->camera_manager->start() != 0)
  {
    throw std::runtime_error("Failed to start camera manager");
  }
  if (this->camera_manager->cameras().empty())
  {
    throw std::runtime_error("No cameras found");
  }

  std::string camera_id = this->camera_manager->cameras().at(0)->id();
  this->camera = this->camera_manager->get(camera_id);
  if (!this->camera)
  {
    throw std::runtime_error("Failed to get camera");
  }
  this->camera->acquire();

  this->config = camera->generateConfiguration({libcamera::StreamRole::VideoRecording});
  if (!this->config)
  {
    throw std::runtime_error("Failed to generate configuration");
  }

  libcamera::StreamConfiguration &stream_config = this->config->at(0);

  stream_config.pixelFormat = libcamera::formats::RGB888;
  stream_config.size.width = this->camera_configuration.width;
  stream_config.size.height = this->camera_configuration.height;
  if (this->config->validate() == libcamera::CameraConfiguration::Invalid)
  {
    throw std::runtime_error("Invalid configuration");
  }
  if (this->camera->configure(this->config.get()) != 0)
  {
    throw std::runtime_error("Failed to configure camera");
  }
}

void Camera::init_frame_allocator()
{
  this->allocator = std::make_unique<libcamera::FrameBufferAllocator>(this->camera);

  for (libcamera::StreamConfiguration &cfg : *(this->config))
  {
    int ret = this->allocator->allocate(cfg.stream());
    if (ret < 0)
    {
      throw std::runtime_error("Cannot allocate buffer");
    }
  }
}

void Camera::capture()
{
  libcamera::StreamConfiguration &stream_config = this->config->at(0);
  libcamera::Stream *stream = stream_config.stream();

  const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = this->allocator->buffers(stream);
  for (size_t i = 0; i < buffers.size(); i++)
  {
    std::unique_ptr<libcamera::Request> request = this->camera->createRequest();
    if (!request)
    {
      throw std::runtime_error("Cannot create request");
    }
    const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers.at(i);

    int ret = request->addBuffer(stream, buffer.get());
    if (ret < 0)
    {
      throw std::runtime_error("Cannot set buffer for request");
    }

    size_t buffer_size = 0;

    for (size_t i = 0; i < buffer->planes().size(); i++)
    {
      const libcamera::FrameBuffer::Plane &plane = buffer->planes().at(i);
      buffer_size += plane.length;

      if (i == buffer->planes().size() - 1 || plane.fd.get() != buffer->planes()[i + 1].fd.get())
      {
        this->buffer_mem[buffer.get()] = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
        buffer_size = 0;
      }
    }

    this->requests.push_back(std::move(request));
  }
}

void Camera::requestComplete(libcamera::Request *request)
{
  if (request->status() == libcamera::Request::RequestCancelled)
  {
    return;
  }
  const libcamera::Request::BufferMap &buffers = request->buffers();

  for (auto &bufferPair : buffers)
  {
    libcamera::FrameBuffer *buffer = bufferPair.second;
    const libcamera::FrameMetadata &metadata = buffer->metadata();
    size_t byteused = metadata.planes()[0].bytesused;
    void *mem = buffer_mem.find(buffer)->second;
    void *dest = malloc(byteused);
    memcpy(dest, mem, byteused);
    this->notify(dest, byteused);
  }

  request->reuse(libcamera::Request::ReuseBuffers);
  this->camera->queueRequest(request);
}

#else

Camera::Camera(const CameraConfiguration &camera_configuration) : camera_configuration{camera_configuration}
{
}

Camera::~Camera()
{
  if (this->capture.isOpened())
  {
    this->capture.release();
  }
}

void Camera::start()
{
  this->capture.open("/sentinel/static/video.mp4");

  if (!this->capture.isOpened())
  {
    throw std::runtime_error("Error: Could not open the video file!");
  }

  cv::Mat frame;
  std::vector<uchar> buffer;
  cv::Size target_size(640, 480);
  while (true)
  {
    this->capture >> frame;
    if (frame.empty())
    {
      break;
    }

    cv::resize(frame, frame, target_size);
    bool result = cv::imencode(this->camera_configuration.encoding, frame, buffer);
    if (!result)
    {
      throw std::runtime_error("Error: Could not encode frame.");
    }

    this->notify(buffer.data(), buffer.size());
  }
  std::cout << "End of video reached." << std::endl;
}

#endif