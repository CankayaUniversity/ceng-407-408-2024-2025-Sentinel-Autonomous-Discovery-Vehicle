#pragma once

#include <memory>
#include <cstddef>
#include <vector>

class CameraRequestObserver
{
public:
  using SharedPtr = std::shared_ptr<CameraRequestObserver>;

public:
  CameraRequestObserver() = default;
  virtual ~CameraRequestObserver() = default;
  virtual void completed(void *frame, size_t frame_size) = 0;
};

class CameraRequestSubject
{
public:
  CameraRequestSubject() = default;
  virtual ~CameraRequestSubject() = default;

public:
  void attach(CameraRequestObserver::SharedPtr observer);
  void notify(void *frame, size_t frame_size);

private:
  std::vector<CameraRequestObserver::SharedPtr> observers;
};