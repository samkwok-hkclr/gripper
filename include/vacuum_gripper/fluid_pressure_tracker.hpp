#ifndef FLUID_PRESSURE_TRACKER_HPP__
#define FLUID_PRESSURE_TRACKER_HPP__

#pragma once

#include <deque>
#include <numeric>
#include <cmath>

class FluidPressureTracker 
{
public:
  FluidPressureTracker(size_t max_history_size = 100) 
    : max_size(max_history_size) 
  {

  }

  void insert_pressure(float value)
  {
    history.push_back(value);

    sum += value;
    sum_squares += value * value;

    // if max_size is set and exceeded, remove oldest value
    if (max_size > 0 && history.size() > max_size) 
    {
      float old_value = history.front();
      sum -= old_value;
      sum_squares -= old_value * old_value;
      history.pop_front();
    }
  }

  float back() const
  {
    if (history.empty())
      return 0.0;
    
    return history.back();
  }

  float variance() const 
  {
    if (history.empty()) 
      return 0.0;

    float mean = sum / history.size();
    
    return (sum_squares / history.size()) - (mean * mean);
  }

  size_t size() const 
  { 
    return history.size(); 
  }

private:
  std::deque<float> history; // store fluid pressure values

  float sum = 0.0;
  float sum_squares = 0.0;
  size_t max_size;

};

#endif