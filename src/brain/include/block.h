#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterBlockNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class CalcGoliePos : public SyncActionNode
{
public:
    CalcGoliePos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("block_threshold", 1.0, "골키퍼가 공을 막을지 찰지 결정"),
            InputPort<double>("golie_radius", 1.0, "골대 중앙을 기준으로 하는 반경"),
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
        };
    }

    NodeStatus tick() override;

private:

    Brain *brain;
};

class GolieMove : public SyncActionNode
{
public:
    GolieMove(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("vx_limit", 0.7, "최대 속도"),
            InputPort<double>("vy_limit", 0.35, "최대 속도"),
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("return_position_limit", 1.5, "이 이상 멀어지면 return 역할을 수행"),

        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class GolieInitPos : public SyncActionNode
{
public:
    GolieInitPos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.35, "여기까진 직진으로 성큼성큼 오다가, 여기부터 정면 바라보도록 회전"),
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("vx_limit", 0.6, "최대 속도"),
            InputPort<double>("vy_limit", 0.3, "최대 속도"),
            InputPort<double>("init_golie_pos_x", -4.0, "골대중앙위치... 보다 살짝 앞에"),
            InputPort<double>("init_golie_pos_y", 0.0, "골대중앙위치"),
            InputPort<double>("init_golie_pos_theta", 0.0, "골대중앙위치"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};