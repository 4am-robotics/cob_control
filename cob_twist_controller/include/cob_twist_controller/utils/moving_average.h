/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef COB_TWIST_CONTROLLER_UTILS_MOVING_AVERAGE_H
#define COB_TWIST_CONTROLLER_UTILS_MOVING_AVERAGE_H

#include <stdint.h>
#include <ros/ros.h>
#include <deque>

template
<typename T>
class MovingAverageBase
{
    public:
        explicit MovingAverageBase()
        {}

        virtual void reset() = 0;
        virtual void addElement(T element) = 0;
        virtual bool calcMovingAverage(T& average) const = 0;
};

template
<typename T>
class MovingAverageSimple : public MovingAverageBase<T>
{
    public:
        explicit MovingAverageSimple(uint16_t size)
        : MovingAverageBase<T>(),
          size_(size)
        {
            weighting_.assign(size_, 1.0);
        }

        virtual void reset()
        {
            s_.clear();
        }

        virtual void addElement(T element)
        {
            if (s_.size() < size_)
            {
                s_.push_front(element);
            }
            else
            {
                s_.pop_back();
                s_.push_front(element);
            }
        }

        virtual bool calcMovingAverage(T& average) const
        {
            if (!s_.empty())
            {
                T sum;
                T diff;
                for (uint16_t i = 0; i < s_.size(); ++i)
                {
                    sum += s_[i] * weighting_[i];
                    diff += weighting_[i];
                }
                average = sum / diff;
                return true;
            }
            else
            {
                // no element available
                return false;
            }
        }

    protected:
        uint16_t size_;
        std::deque<T> s_;
        std::deque<double> weighting_;
};

template
<typename T>
class MovingAverageWeighted : public MovingAverageSimple<T>
{
    public:
        explicit MovingAverageWeighted(uint16_t size)
        : MovingAverageSimple<T>(size)
        {
            this->weighting_.clear();
            for (uint16_t i = 0; i < this->size_; i++)
            {
                this->weighting_.push_front(triangle(i));
            }
        }

    private:
        double triangle(uint16_t n)
        {
            if (n == 0)
            {
                return 0.0;
            }
            else
            {
                return static_cast<double>(n)*(static_cast<double>(n)+1.0)/2.0;
            }
        }
};

template
<typename T>
class MovingAverageExponential : public MovingAverageBase<T>
{
    public:
        explicit MovingAverageExponential(double factor)
        : MovingAverageBase<T>(),
          factor_(factor)
        {
            empty_ = true;
        }

        virtual void reset()
        {
            average_ = T();
            empty_ = true;
        }

        void addElement(T element)
        {
            if (empty_)
            {
                average_ = element;
                empty_ = false;
            }
            else
            {
                average_ = factor_ * element + (1.0 - factor_) * average_;
            }
        }

        bool calcMovingAverage(T& average) const
        {
            if (!empty_)
            {
                average = average_;
                return true;
            }
            else
            {
                // no element available
                return false;
            }
        }

    private:
        bool empty_;
        double factor_;
        T average_;
};


typedef MovingAverageBase<double> MovingAvgBase_double_t;
typedef MovingAverageSimple<double> MovingAvgSimple_double_t;
typedef MovingAverageWeighted<double> MovingAvgWeighted_double_t;
typedef MovingAverageExponential<double> MovingAvgExponential_double_t;

#endif  // COB_TWIST_CONTROLLER_UTILS_MOVING_AVERAGE_H
