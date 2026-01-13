#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

// class StrikerDecide : public SyncActionNode
// {
// public:
//     StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

//     static PortsList providedPorts()
//     {
//         return {
//             InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
//             InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
//             InputPort<string>("position", "offense", "offense | defense, 공을 어느 방향으로 찰지 결정"),
//             OutputPort<string>("decision_out")};
//     }

//     NodeStatus tick() override;

// private:
//     Brain *brain;
//     double lastDeltaDir = 0.0; 
//     rclcpp::Time timeLastTick; 
// };

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("clearing_min", 1.0, "골에서 이 거리보다 멀면 위험구역 밖"),
            InputPort<double>("closer_margin", 0.2, "내가 더 가깝다고 판정할 여유분"),
            InputPort<double>("clearing_max", 2.0, "이 이상 멀어지면 clearing 종료하고 hold 복귀"), 
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoalieClearingDecide : public SyncActionNode
{
public:
    GoalieClearingDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("chase_threshold", 0.5, "이 거리보다 멀면 공 추격(Chase) 동작을 실행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// class DefenderDecide : public SyncActionNode
// {
// public: 
//     DefenderDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

//     static PortsList providedPorts()
//     {
//         return {
//             InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
//             InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
//             OutputPort<string>("decision_out")};
//     }

//     NodeStatus tick() override;

// private:
//     Brain *brain;
//     double lastDeltaDir = 0.0; 
//     rclcpp::Time timeLastTick; 
// };