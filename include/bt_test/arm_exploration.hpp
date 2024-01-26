//
// Created by gianluca on 1/25/24.
//

#ifndef BUILD_ARM_EXPLORATION_HPP
#define BUILD_ARM_EXPLORATION_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T6 {
    class Report
    {
    private:
        int _counter;
    public:
        explicit Report(int counter) {
            _counter = counter;
        }

        BT::NodeStatus nextConfiguration() {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout<<"Configurations remaining "<<_counter<<std::endl;
            _counter--;
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus checkForTarget() const {
            if(_counter < 0) {
                std::cout<<"Target found -> SUCCESS"<<std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            std::cout<<"Target not found -> CONTINUE"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
    };
}

#endif //BUILD_ARM_EXPLORATION_HPP
