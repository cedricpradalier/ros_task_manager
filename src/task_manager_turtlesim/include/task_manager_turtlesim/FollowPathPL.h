#ifndef TASK_FOLLOWPATHPL_H
#define TASK_FOLLOWPATHPL_H

#include "task_manager_turtlesim/TaskFollowPathConfig.h"
#include "task_manager_lib/ParameterList.h"

namespace task_manager_turtlesim {
    
    struct FollowPathWP {
        float x,y;
        FollowPathWP() : x(0), y(0) {}
        FollowPathWP(float x, float y) : x(x), y(y) {}
    };

    class FollowPathPL : public task_manager_lib::ParameterList<TaskFollowPathConfig,FollowPathWP> {
        public:
            // defines Clear, Push, Execute
            TASK_PARAMETER_ENUM(TaskFollowPath);
        public:
            virtual void push_config(const TaskFollowPathConfig & cfg) {
                push_storage(FollowPathWP(cfg.goal_x,cfg.goal_y));
            }

            bool collect_front(float & x, float & y) const {
                if (empty()) return false;
                const FollowPathWP & wp = front();
                x = wp.x; y = wp.y;
                return true;
            }
    };

};

#endif // TASK_FOLLOWPATHPL_H
