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

#include <ros/ros.h>
#include <deque>

class MovingAverage
{
    private:
        std::deque<double> s_;
        std::deque<double> weighting_, temp_;

        unsigned int size_;

    public:
        MovingAverage(int size=3)
        {
            size_ = size;
            calculateWeighting();
        }
        
        void addElement(double element)
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

        double calcMovingAverage()
        {
            double sum = 0;
            for(std::deque<double>::const_iterator i = s_.begin(); i != s_.end(); ++i)
            {
                sum += *i;
            }

            return sum / s_.size();
        }

        double calcWeightedMovingAverage()
        {
            double sum = 0;
            for(int i = 0; i < s_.size(); i++)
            {
                sum += s_[i] * weighting_[i];
            }

            return sum;
        }

        void calculateWeighting()
        {
            double sum = 0;
            double err = 0;
            double j = 0.0;

            for(int i = 0; i < size_; i++)
            {
                weighting_.push_back((pow(log(2),j+1) / factorial(j+1)));
                sum += weighting_[i];
                j += 1.0;
            }
            
            err = 1 - sum;
            std::deque<double>::iterator i = weighting_.begin();
            *i += err;
        }

        double factorial(unsigned int n)
        {
            if ( n <= 1 )
            {
                return  1;
            }
            else
            {
                return  (double)(n * factorial(n-1));
            }
        }
};

#endif /* MOVING_AVERAGE_H_ */
