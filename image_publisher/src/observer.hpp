#pragma once

#include <memory>
#include <vector>

class RequestObserver
{
public:
  RequestObserver() = default;
  virtual ~RequestObserver() = default;
  virtual void onCompleted(void *mem, size_t byteused) = 0;
};

class RequestSubject
{
public:
  RequestSubject() = default;
  virtual ~RequestSubject() = default;

  void attach(std::shared_ptr<RequestObserver> observer);
  void notify(void *mem, size_t byteused);

private:
  std::vector<std::shared_ptr<RequestObserver>> observers;
};