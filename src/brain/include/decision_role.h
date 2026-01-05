#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            InputPort<string>("position", "offense", "offense | defense, 공을 어느 방향으로 찰지 결정"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("isolated_dist", 1.0, "골에서 이 거리보다 멀면 위험구역 밖"),
            InputPort<double>("isolated_y", 0.7, "|y|가 이보다 크면 외곽"),
            InputPort<double>("closer_margin", 0.2, "내가 더 가깝다고 판정할 여유분"),
            InputPort<double>("max_clear_run_dist", 1.2, "너무 멀면 골키퍼가 비우므로 clearing 금지"),
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
            InputPort<double>("clearing_threshold", 2.5, "이 이상 멀어지면 clearing 종료하고 hold 복귀"), 
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class DefenderDecide : public SyncActionNode
{
public: 
    DefenderDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};