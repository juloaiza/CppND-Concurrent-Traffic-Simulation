#include <iostream>
#include <random>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    /* Create a lock and pass it to the condition variable */
    std::unique_lock<std::mutex> u_lock(_mutex);
    _condition.wait(u_lock, [this] { return !_queue.empty(); });

    /* Get the latest element and remove it from the queue */
    T msg = std::move(_queue.back());
    _queue.pop_back();
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    std::lock_guard<std::mutex> u_lock(_mutex);
    
  // add vector to queue
    std::cout << "   Message " << msg << " has been sent to the queue" << std::endl;
  
    /* Move into queue and send a notification. */
    _queue.push_back(std::move(msg));
    _condition.notify_one();
}


/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
  std::lock_guard<std::mutex> lock(_mutex);
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    
    //std::lock_guard<std::mutex> lock(_mutex);
    while (true)
    {
      std::cout << " Wait for Green" << std::endl;
      /* Sleep at every iteration to reduce CPU usage */
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      /* Wait until the traffic light is green, received from message queue */
      //auto curr_phase = _msgQueue->receive();
      auto curr_phase = _msgQueue.receive();
      if (curr_phase == TrafficLightPhase::green)
      {
        return;
      }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
  std::lock_guard<std::mutex> lock(_mutex);
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 

  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
  
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 

  
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_int_distribution<> distr(4000, 6000);
  double cycleDuration = distr(eng); // duration of a single simulation cycle in ms
  std::chrono::time_point<std::chrono::system_clock> lastUpdate;

      /* Print id of the current thread */
    std::unique_lock<std::mutex> lck(_mtx);
    std::cout << "Traffic_Light #" << _id << "::CycleThroughPhases: thread id = " << std::this_thread::get_id() << std::endl;
    lck.unlock();
  
  
  // init stop watch
  lastUpdate = std::chrono::system_clock::now();
  while (true)
  {
      // sleep at every iteration to reduce CPU usage
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // compute time difference to stop watch
      long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
      if (timeSinceLastUpdate >= cycleDuration)
      {
          // toggle between red and green
          _currentPhase = _currentPhase == TrafficLightPhase::green ? TrafficLightPhase::red : TrafficLightPhase::green;

          // send update message using a condition variable
        
         //_msgQueue->send(std::move(_currentPhase));
        _msgQueue.send(std::move(_currentPhase));

          // reset stop watch for next cycle
          lastUpdate = std::chrono::system_clock::now();
      }
  } // eof simulation loop
}

