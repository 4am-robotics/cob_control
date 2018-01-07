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


#ifndef COB_TWIST_CONTROLLER_TASK_STACK_TASK_STACK_CONTROLLER_H
#define COB_TWIST_CONTROLLER_TASK_STACK_TASK_STACK_CONTROLLER_H

#include <vector>
#include <string>
#include <stdint.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/damping_methods/damping_base.h"

template
<typename PRIO>
struct Task
{
    PRIO prio_;
    Eigen::MatrixXd task_jacobian_;
    Eigen::VectorXd task_;
    std::string id_;
    bool is_active_;
    TwistControllerParams tcp_;

    Task(PRIO prio, std::string id) : prio_(prio), id_(id), is_active_(true)
    {}

    Task(PRIO prio, std::string id, Eigen::MatrixXd task_jacobian, Eigen::VectorXd task)
    : prio_(prio), id_(id), task_jacobian_(task_jacobian), task_(task), is_active_(true)
    {}

    Task(const Task& task)
    : prio_(task.prio_),
      id_(task.id_),
      task_jacobian_(task.task_jacobian_),
      task_(task.task_),
      is_active_(task.is_active_),
      tcp_(task.tcp_)
    {}

    ~Task()
    {}

    inline void setPriority(PRIO prio)
    {
        this->prio_ = prio;
    }

    inline bool operator<(const Task& other) const
    {
        return ( this->prio_ < other.prio_ );
    }

    inline bool operator>(const Task& other) const
    {
        return ( this->prio_ > other.prio_ );
    }

    inline bool operator==(const Task& other) const
    {
        return ( this->prio_ == other.prio_ );
    }
};

template
<typename PRIO>
class TaskStackController
{
    public:
        typedef typename std::vector<Task<PRIO> >::iterator TypedIter_t;
        typedef typename std::vector<Task<PRIO> >::const_iterator TypedConstIter_t;

        ~TaskStackController()
        {
            this->tasks_.clear();
        }

        TaskStackController()
        {
            this->active_task_iter_ = this->tasks_.begin();
            this->modification_time_ = ros::Time(0);
        }

        void clearAllTasks();

        void addTask(Task<PRIO> t);

        void deactivateTask(typename std::vector<Task<PRIO> >::iterator it);
        void deactivateTask(std::string task_id);
        void activateTask(std::string task_id);
        void deactivateAllTasks();

        void activateAllTasks();
        void activateHighestPrioTask();

        typename std::vector<Task<PRIO> >::iterator getTasksBegin();
        typename std::vector<Task<PRIO> >::iterator getTasksEnd();
        typename std::vector<Task<PRIO> >::iterator nextActiveTask();
        typename std::vector<Task<PRIO> >::iterator beginTaskIter();

        int countActiveTasks() const;
        ros::Time getLastModificationTime() const;

    private:
        void updateModificationTime(bool change);

        std::vector<Task<PRIO> > tasks_;
        TypedIter_t active_task_iter_;
        ros::Time modification_time_;
};

template <typename PRIO>
int TaskStackController<PRIO>::countActiveTasks() const
{
    int i = 0;
    for (TypedConstIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (it->is_active_)
        {
            ++i;
        }
    }

    return i;
}

/**
 * Insert new task sorted.
 */
template <typename PRIO>
void TaskStackController<PRIO>::addTask(Task<PRIO> t)
{
    TypedIter_t begin_it = this->tasks_.begin();
    TypedIter_t mem_it = this->tasks_.end();
    for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (it->id_ == t.id_)  // task already existent -> ignore add
        {
            mem_it = it;
            it->task_jacobian_ = t.task_jacobian_;
            it->task_ = t.task_;
            it->tcp_ = t.tcp_;
            break;
        }
    }

    if (this->tasks_.end() == mem_it)
    {
        for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
        {
            if (t.prio_ < it->prio_)
            {
                mem_it = it;
                break;
            }
        }

        if (this->tasks_.end() == mem_it)
        {
            this->tasks_.push_back(t);
        }
        else
        {
            this->tasks_.insert(mem_it, t);
        }

        this->updateModificationTime(true);
    }
}

template <typename PRIO>
void TaskStackController<PRIO>::activateAllTasks()
{
    bool change = false;
    for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (!it->is_active_)
        {
            change = true;
        }

        it->is_active_ = true;
    }

    this->updateModificationTime(change);
}

template <typename PRIO>
void TaskStackController<PRIO>::activateHighestPrioTask()
{
    TypedIter_t it = this->tasks_.begin();
    if (this->tasks_.end() != it)
    {
        this->updateModificationTime(!it->is_active_);

        ROS_WARN_STREAM("Activation of highest prio task in stack: " << it->id_);
        it->is_active_ = true;
    }
}

template <typename PRIO>
typename std::vector<Task<PRIO> >::iterator TaskStackController<PRIO>::getTasksBegin()
{
    return this->tasks_.begin();
}

template <typename PRIO>
typename std::vector<Task<PRIO> >::iterator TaskStackController<PRIO>::getTasksEnd()
{
    return this->tasks_.end();
}

template <typename PRIO>
void TaskStackController<PRIO>::activateTask(std::string task_id)
{
    for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (it->id_ == task_id)
        {
            this->updateModificationTime(!it->is_active_);
            it->is_active_ = true;
            break;
        }
    }
}

template <typename PRIO>
void TaskStackController<PRIO>::deactivateTask(typename std::vector<Task<PRIO> >::iterator it)
{
    if (std::find(this->tasks_.begin(), this->tasks_.end(), it) != this->tasks_.end())
    {
        this->updateModificationTime(it->is_active_);
        it->is_active_ = false;
    }
}

template <typename PRIO>
void TaskStackController<PRIO>::deactivateTask(std::string task_id)
{
    for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (it->id_ == task_id)
        {
            this->updateModificationTime(it->is_active_);
            it->is_active_ = false;
            break;
        }
    }
}

template <typename PRIO>
void TaskStackController<PRIO>::deactivateAllTasks()
{
    bool change = false;
    for (TypedIter_t it = this->tasks_.begin(); it != this->tasks_.end(); it++)
    {
        if (it->is_active_)
        {
            change = true;
        }

        it->is_active_ = false;
    }

    this->updateModificationTime(change);
}

template <typename PRIO>
typename std::vector<Task<PRIO> >::iterator TaskStackController<PRIO>::nextActiveTask()
{
    TypedIter_t ret = this->tasks_.end();

    while (this->tasks_.end() != this->active_task_iter_)
    {
        if (this->active_task_iter_->is_active_)
        {
            ret = this->active_task_iter_++;
            break;
        }

        this->active_task_iter_++;
    }

    return ret;
}

template <typename PRIO>
typename std::vector<Task<PRIO> >::iterator TaskStackController<PRIO>::beginTaskIter()
{
    this->active_task_iter_ = this->tasks_.begin();
    return this->active_task_iter_;
}

template <typename PRIO>
void TaskStackController<PRIO>::clearAllTasks()
{
    this->tasks_.clear();
    this->active_task_iter_ = this->tasks_.end();
    this->updateModificationTime(true);
}

template <typename PRIO>
ros::Time TaskStackController<PRIO>::getLastModificationTime() const
{
    return this->modification_time_;
}

template <typename PRIO>
void TaskStackController<PRIO>::updateModificationTime(bool change)
{
    if (change)
    {
        this->modification_time_ = ros::Time::now();
    }
}

// -------------------- typedefs ---------------------------
typedef TaskStackController<uint32_t> TaskStackController_t;
typedef Task<uint32_t> Task_t;
typedef std::vector<Task_t >::iterator TaskSetIter_t;

#endif  // COB_TWIST_CONTROLLER_TASK_STACK_TASK_STACK_CONTROLLER_H
