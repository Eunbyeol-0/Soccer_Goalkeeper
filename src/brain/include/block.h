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
            //InputPort<double>("return_position_limit", 1.2, "이 이상 멀어지면 return 역할을 수행"),

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

class PredictBallTraj : public SyncActionNode
{
public:
    PredictBallTraj(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("R_meas", 9e-4, "measurement noise (R)"), //(0.03)^2
            InputPort<double>("sigma_a", 2.0, "proccess noise (Q)"),
            InputPort<double>("P0_pos", 0.25, "위치 공분산의 초기값"), // (0.5m)^2
            InputPort<double>("P0_vel", 1.0, "속도 공분산의 초기값"), // (1m/s)^2
            InputPort<double>("horizon", 0.5, "horizen초 뒤의 공을 예측"),
        };
    }

    NodeStatus tick() override;

private:
    // ----- 시간 dt 계산용 -----
    bool has_prev_time_{false};
    rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};

    // ----- ego-motion 계산용(odom 이전값) -----
    bool has_prev_odom_{false};
    double prev_ox_{0.0};
    double prev_oy_{0.0};
    double prev_otheta_{0.0};

    // ----- Kalman Filter 상태 -----
    bool kf_initialized_{false};
    double x_{0.0}, y_{0.0}, vx_{0.0}, vy_{0.0};
    double P_[4][4]{};  // 0으로 초기화

    Brain *brain;
};