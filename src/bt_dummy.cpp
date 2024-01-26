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

using namespace std::chrono_literals;
using namespace BT;

//class ActionInterface
//{
//public:
//    ActionInterface() {
//        _locations = {{0,0}, {1,1}};
//    }
//
//    NodeStatus MoveTo(float x, float y) {
//        if(_locations.front().first == x && _locations.front().second == y) {
//            _locations.erase(_locations.begin());
//            return NodeStatus::SUCCESS;
//        } else {
//            return NodeStatus::FAILURE;
//        }
//    }
//private:
//    std::vector<std::pair<float, float>> _locations;
//};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("dummy_bt");
    std::vector<std::pair<float, float>> locations = {{0.0,0.0}, {1.0,1.0}, {2.0,2.0}, {3.0,3.0}};
    std::vector<float> values = {10, 16, 8, 21};
    bool done = false;

    BehaviorTreeFactory factory;
    Tree tree;

    factory.registerSimpleAction("Done", [&](TreeNode&){
        done = true;
        return NodeStatus::SUCCESS;
    } );

    int task = 3;
    switch(task) {
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
            factory.registerSimpleAction("CheckReachable", [&](TreeNode&){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout<<"Checking ->";
                auto value = rand() % 10 + 1;
                if ( value < 8) {
                    std::cout<<"SUCCESS"<<std::endl;
                    return NodeStatus::SUCCESS;
                }
                std::cout<<"FAILURE"<<std::endl;
                return NodeStatus::FAILURE;
            } );
            factory.registerSimpleAction("Continue", [&](TreeNode&){
                std::cout<<"Continue"<<std::endl;
                return NodeStatus::SUCCESS;
            });
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree3.xml");
            break;
        }
        case 4:
        {
            factory.registerNodeType<T1::MoveTo>("MoveTo", std::make_shared<T1::Report>(locations));
            factory.registerSimpleAction("ActivateManipulator", [&](TreeNode&){
                std::cout<<"Arm activated"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree4.xml");
            break;
        }
        case 5:
        {
            auto report5 = std::make_shared<T5::Report>(locations);

            factory.registerSimpleAction("GenerateNextDestination", [&](TreeNode&){
                return report5->nextDestination();
            } );
            factory.registerSimpleAction("CheckForExplorationComplete", [&](TreeNode&){
                return report5->explorationComplete();
            } );
            factory.registerSimpleAction("MoveToDestination", [&](TreeNode&){
                return report5->moveTo();
            } );
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree5.xml");
            break;
        }
        case 6:
        {
            auto report6 = std::make_shared<T6::Report>(5);

            factory.registerSimpleAction("MoveToNewConfiguration", [&](TreeNode&){
                return report6->nextConfiguration();
            } );
            factory.registerSimpleAction("CheckForTarget", [&](TreeNode&){
                return report6->checkForTarget();
            } );
            factory.registerSimpleAction("ApproachTarget", [&](TreeNode&){
                std::cout<<"Target reached"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree6.xml");
            break;
        }
        case 7:
        {
            srand(time(nullptr));

            factory.registerSimpleAction("MoveToRestPosition", [&](TreeNode&){
                std::cout<<"Moving to rest position"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            factory.registerSimpleAction("Drop", [&](TreeNode&){
                std::cout<<"Dropping item"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            factory.registerSimpleAction("MoveToDropPosition", [&](TreeNode&){
                std::cout<<"Moving to drop position"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            factory.registerSimpleAction("Pick", [&](TreeNode&){
                std::cout<<"Picking item"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            factory.registerSimpleAction("PerformObservation", [&](TreeNode&){
                std::cout<<"Observing item"<<std::endl;
                return NodeStatus::SUCCESS;
            } );
            factory.registerSimpleAction("EstimateGrasp", [&](TreeNode&){
                std::cout<<"Estimate grasp"<<std::endl;
//                auto value = rand() % 10 + 1;
//                if ( value % 2 == 0) {
//                    return NodeStatus::SUCCESS;
//                }
                return NodeStatus::FAILURE;
            } );
            tree = factory.createTreeFromFile("/home/gianluca/development/humble/bt_ws/bt_test/xml/tree7.xml");
            break;
        }
        default:
            return -1;
    }


    // Helper function to print the tree.
    BT::printTreeRecursively(tree.rootNode());

    // a certain set of transitions happened as expected
//    BT::TreeObserver observer(tree);

    while(rclcpp::ok())
    {
        tree.tickWhileRunning();
        if(done) {
            break;
        }
    }

//    for(auto stats: observer.statistics()) {
//        std::cout << "[" << stats.first
//                  << "] \tT/S/F:  " << stats.second.transitions_count
//                  << "/" << stats.second.success_count
//                  << "/" << stats.second.failure_count
//                  << std::endl;
//    }

    return 0;
}
