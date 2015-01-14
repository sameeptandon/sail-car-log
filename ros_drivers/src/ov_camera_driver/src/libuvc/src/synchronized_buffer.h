#ifndef _pcd_buffer
#define _pcd_buffer

#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <iostream>
using namespace std;

template <typename T>
class SynchronizedBuffer
{
public:
  SynchronizedBuffer (boost::mutex *io_mtx) {
    is_done = false;
    io_mutex = io_mtx; 
  }

  bool 
  pushBack (const T*); // thread-save wrapper for push_back() method of ciruclar_buffer

  const T* getFront (); // thread-save wrapper for front() method of ciruclar_buffer  
  
  void
  waitUntilEmpty();

  void
  waitForSpace();

  inline void setDone() { 
    is_done = true; 
  }

  inline bool 
  isFull ()
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    return (buffer_.full ());
  }

  inline bool
  isEmpty ()
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    return (buffer_.empty ());
  }

  inline int 
  getSize ()
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    return (int (buffer_.size ()));
  }

  inline int 
  getCapacity ()
  {
    return (int (buffer_.capacity ()));
  }

  inline void 
  setCapacity (int buff_size)
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    buffer_.set_capacity (buff_size);
  }

private:
  SynchronizedBuffer (const SynchronizedBuffer&); // Disabled copy constructor
  SynchronizedBuffer& operator =(const SynchronizedBuffer&); // Disabled assignment operator

  boost::mutex bmutex_;
  boost::condition_variable buff_empty_, buff_has_data_;
  boost::circular_buffer<const T*> buffer_;
  bool is_done;
  boost::mutex* io_mutex;
};

//////////////////////////////////////////////////////////////////////////////////////////
template <typename T> bool 
SynchronizedBuffer<T>::pushBack (const T* obj)
{
  bool retVal = false;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    if (!buffer_.full ())
      retVal = true;
    buffer_.push_back (obj);
  }
  buff_empty_.notify_one ();
  return (retVal);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
const T* SynchronizedBuffer<T>::getFront ()
{
  const T* obj;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    while (buffer_.empty ())
      {
	if (is_done)
	  break;
	{
	  boost::mutex::scoped_lock io_lock (*io_mutex);
	  cerr << "No data in buffer_ yet or buffer is empty." << endl;
	}
	buff_empty_.wait (buff_lock);
      }
    obj = buffer_.front ();
    buffer_.pop_front ();
    buff_has_data_.notify_one();
  }
  return (obj);
}



template <typename T> void
SynchronizedBuffer<T>::waitUntilEmpty() {
  boost::mutex::scoped_lock buff_lock (bmutex_);
  while (!buffer_.empty ()) {
    if (is_done)
      break;
    buff_has_data_.wait(buff_lock);
  }
}

template <typename T> void
SynchronizedBuffer<T>::waitForSpace() {
  boost::mutex::scoped_lock buff_lock (bmutex_);
  while (buffer_.full()) {
    if (is_done)
      break;
    buff_has_data_.wait(buff_lock);
  }
}

#endif
 
