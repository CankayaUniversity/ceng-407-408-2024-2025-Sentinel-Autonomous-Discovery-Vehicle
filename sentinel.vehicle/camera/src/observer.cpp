#include "observer.hpp"

void CameraRequestSubject::attach(CameraRequestObserver::SharedPtr observer)
{
  this->observers.push_back(observer);
}

void CameraRequestSubject::notify(void *frame, size_t frame_size, int id)
{
  for (auto &&observer : this->observers)
  {
    observer->completed(frame, frame_size, id);
  }
}
