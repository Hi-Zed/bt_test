#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_observer.h"

#include "bt_test/move_to.hpp"
#include "bt_test/move_to_threshold.hpp"
#include "bt_test/move_to_async.hpp"
#include "bt_test/exploration.hpp"
#include "bt_test/arm_exploration.hpp"
#include "bt_test/pick_and_place.hpp"
#include "bt_test/press_button.hpp"
#include "bt_test/assembly.hpp"

using namespace std::chrono_literals;
using namespace BT;

// class ActionInterface
//{
// public:
//     ActionInterface() {
//         _locations = {{0,0}, {1,1}};
//     }
//
//     NodeStatus MoveTo(float x, float y) {
//         if(_locations.front().first == x && _locations.front().second == y) {
//             _locations.erase(_locations.begin());
//             return NodeStatus::SUCCESS;
//         } else {
//             return NodeStatus::FAILURE;
//         }
//     }
// private:
//     std::vector<std::pair<float, float>> _locations;
// };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("dummy_bt");
    std::vector<std::pair<float, float>> locations = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}};
    std::vector<std::string> labels = {"Button R", "Button P", "Button C"};
    std::vector<float> values = {10, 16, 8, 21};
    bool done = false;
    int success_count = 0;
    int failure_count = 0;

    // for (int i = 5; i <= 5; i++)
    for (int i = 1; i <= 9; i++)
    {
        BehaviorTreeFactory factory;
        Tree tree;

        factory.registerSimpleAction("Done", [&](TreeNode &)
                                     {
                                        std::cout<<"Success"<<std::endl;
                                        success_count++;
                                        done = true;
                                        return NodeStatus::SUCCESS;
                                    });

        factory.registerSimpleAction("Fail", [&](TreeNode &)
                                     {
                                        std::cout<<"Failure"<<std::endl;
                                        failure_count++;
                                        done = true;
                                        return NodeStatus::SUCCESS;
                                    });

        switch (i)
        {
        case 1:
        {
            factory.registerNodeType<T1::MoveTo>("MoveTo", std::make_shared<T1::Report>(locations));

            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree1.xml");
            break;
        }
        case 2:
        {
            auto report2 = std::make_shared<T2::Report>(
                15.0,
                locations,
                values);

            factory.registerNodeType<T2::MoveToThreshold>("MoveTo", report2);
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree2.xml");
            break;
        }
        case 3:
        {
            factory.registerNodeType<MoveToAsync>("MoveTo");
            factory.registerSimpleAction("CheckReachable", [&](TreeNode &)
                                         {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout<<"Checking ->";
                auto value = rand() % 10 + 1;
                if ( value < 8) {
                    std::cout<<"SUCCESS"<<std::endl;
                    return NodeStatus::SUCCESS;
                }
                std::cout<<"FAILURE"<<std::endl;
                return NodeStatus::FAILURE; });
            factory.registerSimpleAction("Continue", [&](TreeNode &)
                                         {
                std::cout<<"Continue"<<std::endl;
                return NodeStatus::SUCCESS; });
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree3.xml");
            break;
        }
        case 4:
        {
            factory.registerNodeType<T1::MoveTo>("MoveTo", std::make_shared<T1::Report>(locations));
            factory.registerSimpleAction("ActivateManipulator", [&](TreeNode &)
                                         {
                std::cout<<"Arm activated"<<std::endl;
                return NodeStatus::SUCCESS; });
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree4.xml");
            break;
        }
        case 5:
        {
            auto report5 = std::make_shared<T5::Report>(locations);

            factory.registerSimpleAction("GenerateNextDestination", [&](TreeNode &)
                                         { return report5->nextDestination(); });
            factory.registerSimpleAction("CheckForExplorationComplete", [&](TreeNode &)
                                         { return report5->explorationComplete(); });
            factory.registerNodeType<T5::MoveTo>("MoveToDestination", report5);
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree5.xml");
            break;
        }
        case 6:
        {
            auto report6 = std::make_shared<T6::Report>(5);

            factory.registerSimpleAction("MoveToNewConfiguration", [&](TreeNode &)
                                         { return report6->nextConfiguration(); });
            factory.registerSimpleAction("CheckForTarget", [&](TreeNode &)
                                         { return report6->checkForTarget(); });
            factory.registerNodeType<T6::ApproachTarget>("ApproachTarget", report6);
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree6.xml");
            break;
        }
        case 7:
        {
            bool firstTime = true;

            factory.registerSimpleAction("MoveToRestPosition", [&](TreeNode &)
                                         {
                std::cout<<"Moving to rest position"<<std::endl;
                return NodeStatus::SUCCESS; });
            factory.registerSimpleAction("Drop", [&](TreeNode &)
                                         {
                std::cout<<"Dropping item"<<std::endl;
                return NodeStatus::SUCCESS; });
            factory.registerSimpleAction("MoveToDropPosition", [&](TreeNode &)
                                         {
                std::cout<<"Moving to drop position"<<std::endl;
                return NodeStatus::SUCCESS; });
            factory.registerSimpleAction("Pick", [&](TreeNode &)
                                         {
                std::cout<<"Picking item"<<std::endl;
                return NodeStatus::SUCCESS; });
            factory.registerSimpleAction("PerformObservation", [&](TreeNode &)
                                         {
                std::cout<<"Observing item"<<std::endl;
                return NodeStatus::SUCCESS; });
            factory.registerSimpleAction("EstimateGrasp", [&](TreeNode &)
                                         {
                std::cout<<"Estimate grasp"<<std::endl;
                if(firstTime) {
                    firstTime = false;
                    return NodeStatus::SUCCESS;
                } else {
                    return NodeStatus::FAILURE;
                } });

            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree7.xml");
            break;
        }
        case 8:
        {
            std::cout << "Task 8" << std::endl;
            factory.registerNodeType<T8::PressButton>("PressButton", std::make_shared<T8::Report>(labels));
            factory.registerSimpleAction("EvaluateProcessing", [&](TreeNode &)
                                         {
                std::cout<<"Evaluating processing ->";
                auto value = rand() % 10 + 1;
                if ( value < 4) {
                    std::cout<<"SUCCESS"<<std::endl;
                    return NodeStatus::SUCCESS;
                }
                std::cout<<"FAILURE"<<std::endl;
                return NodeStatus::FAILURE; });

            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree8.xml");
            break;
        }
        case 9:
        {
            std::cout << "Task 9" << std::endl;
            auto report9 = std::make_shared<T9::Report>();
            factory.registerNodeType<T9::MoveTo>("MoveTo", report9);
            factory.registerNodeType<T9::Pick>("Pick", report9);
            factory.registerNodeType<T9::Drop>("Drop", report9);
            factory.registerSimpleAction("ActivateProcess", [&](TreeNode &)
                                         { return report9->ActivateProcess(); });
            factory.registerSimpleAction("EmptyTray", [&](TreeNode &)
                                         {
                std::cout<<"EmptyTray"<<std::endl;
                return NodeStatus::SUCCESS; });

            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree9.xml");
            break;
        }
        default:
            return -1;
        }

        // Helper function to print the tree.
        BT::printTreeRecursively(tree.rootNode());

        // a certain set of transitions happened as expected
        //    BT::TreeObserver observer(tree);

        while (rclcpp::ok())
        {
            tree.tickWhileRunning();
            if (done)
            {
                break;
            }
        }
    }

    std::cout << "Success: " << success_count << std::endl;
    std::cout << "Failure: " << failure_count << std::endl;

    return 0;
}
