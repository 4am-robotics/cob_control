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
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the implementation of the task stack controller
 *   and providing a task struct.
 *
 ****************************************************************/

#ifndef TASK_STACK_CONTROLLER_H_
#define TASK_STACK_CONTROLLER_H_

#include <Eigen/Dense>
#include <set>
#include <stdint.h>

#include <ros/ros.h>

template
<typename PRIO>
struct Task
{

    // TODO: Use a activation flag here?
    PRIO prio_;
    Eigen::MatrixXd task_jacobian_;
    Eigen::VectorXd task_;
    std::string id_;

    Task(PRIO prio, std::string id) : prio_(prio), id_(id)
    {}

    Task(PRIO prio, std::string id, Eigen::MatrixXd task_jacobian, Eigen::VectorXd task)
    : prio_(prio), id_(id), task_jacobian_(task_jacobian), task_(task)
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
    private:
        // TODO: Use a activation flag here instead of managing separate lists?
        std::set<Task<PRIO> > active_tasks_;
        std::set<Task<PRIO> > inactive_tasks_;

    public:
        typedef typename std::set<Task<PRIO> >::iterator TypedIter_t;

        ~TaskStackController()
        {
            active_tasks_.clear();
            inactive_tasks_.clear();
        }

        void addTask(Task<PRIO> t);

        void deactivateTask(typename std::set<Task<PRIO> >::iterator it);
        void deactivateTask(std::string task_id);

        void activateAllTasks();

        typename std::set<Task<PRIO> >::iterator getActiveTasksBegin();

        typename std::set<Task<PRIO> >::iterator getActiveTasksEnd();
};

template <typename PRIO>
void TaskStackController<PRIO>::addTask(Task<PRIO> t)
{
    TypedIter_t mem_it = this->inactive_tasks_.end();
    for(TypedIter_t it = this->inactive_tasks_.begin(); it != this->inactive_tasks_.end(); it++)
    {
        if(it->id_ == t.id_) // task already existent -> ignore add
        {
            mem_it = it;
            ROS_WARN("Task already existent in inactive_tasks_");
            break;
        }
    }

    if(this->inactive_tasks_.end() != mem_it)
    {
        this->inactive_tasks_.erase(mem_it);
        this->inactive_tasks_.insert(t);
        return; // already found no check for active.
    }

    mem_it = this->active_tasks_.end();
    for(TypedIter_t it = this->active_tasks_.begin(); it != this->active_tasks_.end(); it++)
    {
        if(it->id_ == t.id_) // task already existent -> ignore add
        {
            ROS_WARN("Task already existent in active_tasks_ ... Updating matrices");
            mem_it = it;
            break;
        }
    }

    if(this->active_tasks_.end() != mem_it)
    {
        this->active_tasks_.erase(mem_it);
    }

    this->active_tasks_.insert(t);
}

template <typename PRIO>
void TaskStackController<PRIO>::activateAllTasks()
{
    for(TypedIter_t it = this->inactive_tasks_.begin(); it != this->inactive_tasks_.end(); it++)
    {
        Task<PRIO> t(it->prio_, it->id_, it->task_jacobian_, it->task_);
        this->addTask(t);
    }

    this->inactive_tasks_.clear();
}

template <typename PRIO>
typename std::set<Task<PRIO> >::iterator TaskStackController<PRIO>::getActiveTasksBegin()
{
    return this->active_tasks_.begin();
}


template <typename PRIO>
typename std::set<Task<PRIO> >::iterator TaskStackController<PRIO>::getActiveTasksEnd()
{
    return this->active_tasks_.end();
}


template <typename PRIO>
void TaskStackController<PRIO>::deactivateTask(typename std::set<Task<PRIO> >::iterator it)
{
    if(std::find(this->active_tasks_.begin(), this->active_tasks_.end(), it) != this->active_tasks_.end())
    {
        this->inactive_tasks_.insert(*it);
        this->active_tasks_.erase(it);
    }
}

template <typename PRIO>
void TaskStackController<PRIO>::deactivateTask(std::string task_id)
{
    if(this->active_tasks_.size() <= 1)
    {
        return; // already all deactivated. One task must be left.
    }

    TypedIter_t mem_it = this->active_tasks_.end();
    for (TypedIter_t it = this->active_tasks_.begin(); it != this->active_tasks_.end(); it++)
    {
        if(it->id_ == task_id)
        {
            mem_it = it;
            break;
        }
    }

    if (this->active_tasks_.end() != mem_it)
    {
        this->inactive_tasks_.insert(*mem_it);
        this->active_tasks_.erase(mem_it);
    }
}

typedef TaskStackController<uint32_t> TaskStackController_t;
typedef Task<uint32_t> Task_t;
typedef std::set<Task_t >::iterator TaskSetIter_t;

#endif /* TASK_STACK_CONTROLLER_H_ */
