#include "brain.h"
#include "decision_role.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_DECISION_ROLE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DECISION_ROLE_BUILDER(StrikerDecide)
    REGISTER_DECISION_ROLE_BUILDER(GoalieDecide)
    REGISTER_DECISION_ROLE_BUILDER(GoalieClearingDecide)
    REGISTER_DECISION_ROLE_BUILDER(DefenderDecide)
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    // } else if (!brain->data->tmImLead) {
    //     newDecision = "assist";
    //     color = 0x00FFFFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    // 세트피스 상황에서 adjust 없이 바로 킥
    else if (
        (
            (
                (brain->tree->getEntry<string>("gc_game_sub_state_type") == "CORNER_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "GOAL_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "DIRECT_FREE_KICK"
                || brain->tree->getEntry<string>("gc_game_sub_state_type") == "THROW_IN")
                && brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
            )
            // 세트피스가 아닌 경기에서도 골대를 보고있고 장애물이 없다면 정렬 없이 킥 
            // || (
            //     angleGoodForKick          // 골대에 각이 있고
            //     && !avoidKick             // 장애물이 없고
            //     && reachedKickDir         // 킥 방향 정렬도 되어있다면
            // )
        )
        && brain->data->ballDetected
        && ball.range < 0.5 // 거리가 매우 가까울 때 -> 0.4에서 0.5
        && fabs(brain->data->ball.yawToRobot) < 0.3 // 각도가 거의 정면일 때 -> 0.2에서 0.3
    ) {
        newDecision = "one_touch";
        color = 0xFF0000FF; // Red color
    }
    else if (
        (
            (reachedKickDir && !brain->data->isFreekickKickingOff) 
            // || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        // if (brain->data->kickType == "cross") newDecision = "cross";
        // else newDecision = "kick";      
        newDecision = "kick"; // Striker는 Cross 없이 무조건 슛
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{
    string lastDecision;
    getInput("decision_in", lastDecision); // 사용 X

    // 공 위치 신뢰 (아직 사용 X)
    bool iKnowBallPos      = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    bool ballKnown = (iKnowBallPos || tmBallPosReliable);

    // 기본값: hold
    string newDecision = "hold";
    auto color = 0xFFFFFFFF;

    // if (!ballKnown) {
    //     newDecision = "find_ball";
    //     color = 0x0000FFFF;
    //     setOutput("decision_out", newDecision);
    //     return NodeStatus::SUCCESS;
    // }

    // 위치들 (필드 좌표계)
    auto bPos = brain->data->ball.posToField; 
    auto gPos = brain->data->robotPoseToField; 
    Pose2D ctPos;
    ctPos.x = -4.5; ctPos.y = 0.0;

    // 거리 계산
    double distGKToBall   = norm(bPos.x - gPos.x, bPos.y - gPos.y);
    double distBallToGoal = norm(bPos.x - ctPos.x, bPos.y - ctPos.y);

    // 상대 수집
    auto rPos = brain->data->getRobots();
    double distOppToBallMin = 1e9; // 상대 로봇들 중에서 공에 가장 가까운 상대까지의 최소 거리
    bool hasOpponent = false;
    for (const auto& r : rPos) {
        if (r.label != "Opponent") continue;
        hasOpponent = true;
        double d = norm(bPos.x - r.posToField.x, bPos.y - r.posToField.y);
        if (d < distOppToBallMin) distOppToBallMin = d;
    }

    // -----------------------------
    // 파라미터(튜닝 포인트)
    // -----------------------------
    const double isolated_dist = 1.0;   // 골에서 이 거리보다 멀면 위험구역 밖 
    const double isolated_y    = 0.7;   // |y|가 이보다 크면 외곽 
    const double closer_margin      = 0.2;   // 내가 더 가깝다고 판정할 여유분
    const double max_clear_run_dist = 0.7;   // 너무 멀면 골키퍼가 비우므로 clearing 금지

    // -----------------------------
    // 판정 기준
    // -----------------------------
    bool ballIsIsolated =
        (distBallToGoal > isolated_dist) && // 공이 위험구역 밖에 있고
        (fabs(bPos.y) > isolated_y) && // 공이 외곽에 있고
        (distGKToBall < max_clear_run_dist); // 공이 골키퍼로부터 꽤 가까이 있다

		// 공이 적보다 골키퍼와 가까이에 위치한다
    bool iAmCloser = (!hasOpponent) ? true : (distGKToBall + closer_margin < distOppToBallMin); 

    // -----------------------------
    // 판정 결과
    // -----------------------------
    if (ballIsIsolated && iAmCloser) {
        newDecision = "clearing";
        color = 0x00FF00FF;
    } else {
        newDecision = "hold";
        color = 0xFFFFFFFF;
    }

    setOutput("decision_out", newDecision);

    // brain->log->logToScreen(
    //     "tree/GoalieDecide",
    //     format("Decision:%s ballKnown:%d distGK2Ball:%.2f distBall2Goal:%.2f oppMin:%.2f isolated:%d closer:%d",
    //            newDecision.c_str(), (int)ballKnown,
    //            distGKToBall, distBallToGoal,
    //            (hasOpponent ? distOppToBallMin : -1.0),
    //            (int)ballIsIsolated, (int)iAmCloser),
    //     color
    // );
    
    // 간략한 로그
    brain->log->logToScreen(
        "tree/GoalieDecide",
        format("Decision:%s",
        newDecision.c_str()),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieClearingDecide::tick()
{
    double chaseRangeThreshold;
    double clearingThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    getInput("clearing_threshold", clearingThreshold);

    std::string lastDecision;
    getInput("decision_in", lastDecision);

    // 1) clearing 범위 이탈 체크: 골문 중앙(-4.5, 0)에서 너무 멀어지면 hold로 복귀
    const double goalCx = -4.5;
    const double goalCy =  0.0;
    auto gPos = brain->data->robotPoseToField;

    double distFromGoalCenter = norm(gPos.x - goalCx, gPos.y - goalCy);
    if (distFromGoalCenter > clearingThreshold) {
        setOutput("decision_out", std::string("hold"));
        return NodeStatus::SUCCESS;
    }

    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw   = ball.yawToRobot;

    std::string newDecision;

    // 멀면 chase 유지(히스테리시스 약간)
    if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
        newDecision = "clearing_chase";
    } else {
        // 가까우면 원터치 kick (adjust 없음)
        newDecision = "clearing_kick";
    }
    
    brain->log->logToScreen(
        "tree/GoalieDecide",
        format("Decision:%s",
        newDecision.c_str()),
        color
    );

    setOutput("decision_out", newDecision);
    return NodeStatus::SUCCESS;
}

NodeStatus DefenderDecide::tick() {
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("decision_in", lastDecision);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    // 수비수는 안전하게 걷어내는 것이 목표 (패스)
    // angleGoodForKick은 골대 방향을 보는지 확인하는 함수지만, 
    // 반코트 게임에서는 전방으로 차는 동작(패스/걷어내기)을 위해 그대로 사용합니다.
    const double goalpostMargin = 0.5; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < -2.0 
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");


    // 1. 공을 모르면 -> 찾기
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } 
    // 2. 추적 거리 밖이면 -> chase
    // else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    // {
    //     // 수비수는 쫓아가지 않고 대기 -> 당장은 pass 이후 등에서 활용됨
    //     if (brain->data->ball.posToField.x < -1.0) {
    //         newDecision = "wait";
    //         color = 0x00FFFFFF; // Cyan/White mix
    //     } else {
    //         newDecision = "chase";
    //         color = 0x0000FFFF;
    //     }
    // } 
    // 3. 킥 조건 만족하면 -> kick(패스)
    else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        newDecision = "pass"; // 수비수는 킥을 "패스"라고 명명
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    // 4. 그 외 -> 위치 조정 ("adjust")
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Defend",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}