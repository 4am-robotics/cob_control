/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2014
 *
 * \brief
 *   This Class contains an moving average filter with exponential and normal weighting
 *
 ****************************************************************/
#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

#include <stdint.h>
#include <ros/ros.h>
#include <deque>

template
<typename T>
class MovingAverage
{
    private:
        uint16_t size_;
        std::deque<T> s_;
        std::deque<double> weighting_;

    public:
        MovingAverage(uint16_t size = 3, bool do_auto_weighting = true);

        inline void addElement(T element);
        inline void calcMovingAverage(T& sum) const;
        inline void calcWeightedMovingAverage(T& sum) const;
        inline void calculateWeighting();

        inline void setWeighting(const std::deque<double>& weighting);

        inline double factorial(uint16_t n) const
        {
            if ( n <= 1 )
            {
                return  1.0;
            }
            else
            {
                return  (double)(n * factorial(n - 1));
            }
        }
};

template <typename T>
MovingAverage<T>::MovingAverage(uint16_t size, bool do_auto_weighting) : size_(size)
{
    if(do_auto_weighting)
    {
        calculateWeighting();
    }
}


template <typename T>
void MovingAverage<T>::setWeighting(const std::deque<double>& weighting)
{
    weighting_.clear();
    for(std::deque<double>::const_iterator i = weighting.begin(); i != weighting.end(); ++i)
    {
        weighting_.push_back(*i); // highest weighting should be first!
    }
}


template <typename T>
void MovingAverage<T>::addElement(T element)
{
    if(s_.size() < size_)
    {
        s_.push_front(element);
    }
    else
    {
        // Drops the first element
        s_.pop_back();
        s_.push_front(element);
    }
}


template <typename T>
void MovingAverage<T>::calcMovingAverage(T& sum) const
{
    for(typename std::deque<T>::const_iterator i = s_.begin(); i != s_.end(); ++i)
    {
        sum += *i;
    }


    sum = sum / s_.size();
}


template <typename T>
void MovingAverage<T>::calcWeightedMovingAverage(T& sum) const
{
    for(uint16_t i = 0; i < s_.size(); ++i)
    {
        sum += s_[i] * weighting_[i];
    }

}


template <typename T>
void MovingAverage<T>::calculateWeighting()
{
    double sum = 0;
    double err = 0;
    uint16_t j = 0;

    for(uint16_t i = 0; i < size_; ++i)
    {
        weighting_.push_back((pow(log(2.0), static_cast<double>(j + 1)) / factorial(j + 1)));
        sum += weighting_[i];
        j += 1;
    }

    err = 1.0 - sum;
    std::deque<double>::iterator i = weighting_.begin();
    *i += err;
}


typedef MovingAverage<double> MovingAvg_double_t;


#endif /* MOVING_AVERAGE_H_ */
