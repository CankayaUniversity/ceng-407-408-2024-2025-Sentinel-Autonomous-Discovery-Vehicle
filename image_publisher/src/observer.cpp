#include "observer.hpp"

void RequestSubject::attach(std::shared_ptr<RequestObserver> observer)
{
  this->observers.push_back(observer);
}

void RequestSubject::notify(void *mem, size_t byteused)
{
  for (auto &&observer : this->observers)
  {
    observer->onCompleted(mem, byteused);
  }
}
