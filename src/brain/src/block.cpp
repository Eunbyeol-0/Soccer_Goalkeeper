#include "brain.h"
#include "brain_tree.h"
#include "block.h"

#define REGISTER_BLOCK_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterBlockNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_BLOCK_BUILDER(CalcGoliePos)
    REGISTER_BLOCK_BUILDER(GolieMove)
    REGISTER_BLOCK_BUILDER(GolieInitPos)
    REGISTER_BLOCK_BUILDER(PredictBallTraj)

}

NodeStatus CalcGoliePos::tick(){

    double r;
    double ctPosx, ctPosy;
    getInput("golie_radius", r);
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);
    
    auto bPos = brain->data->ball.posToField; // 공 위치
		
		double cx = ctPosx, cy = ctPosy;
		double bx = bPos.x, by = bPos.y;
		
		double dx = bx - cx; // 방향벡터
		double dy = by - cy;
		double d = norm(dx, dy);
		
		double ux = dx / d; // 단위방향벡터
		double uy = dy / d;
		
		// "반원" 선택 로직... 반코트 기준 하드코딩이므로 나중에 수정필요
		if (ux < 0.15) {
    ux = cap(ux, 10.0, 0.15);
    uy = uy;
		}
		
		// 교점 계산
		double tx = cx + r * ux;
		double ty = cy + r * uy;
		
		// Pose2D로의 변환 및 저장
		Pose2D GoliePos;
		GoliePos.x = tx; GoliePos.y = ty;  
		brain->data->GoliePos = GoliePos;

    // 시각화
    brain->log->setTimeNow();
    brain->log->log(
        "field/block_dir",
        rerun::Arrows2D::from_vectors({{tx-cx, -(ty-cy)}})
            .with_origins({{cx, cy}})
            .with_colors({0x00FFFFFF}) 
            .with_radii(0.015) 
            .with_draw_order(32)
    );
		
		return NodeStatus::SUCCESS;
}

NodeStatus GolieMove::tick(){
    double stop_Threshold;
    double vxLimit, vyLimit;
    double ctPosx, ctPosy;
    double return_position_limit;
    getInput("stop_threshold", stop_Threshold); 
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);
    //getInput("return_position_limit", return_position_limit);

    
		auto rPos = brain->data->getRobots(); // 상대 위치
		auto gPos = brain->data->robotPoseToField; // 본인 위치
		auto target = brain->data->GoliePos; // 목표 위치

    if (!brain->tree->getEntry<bool>("ball_location_known")){
        brain->client->setVelocity(0,0,0);
        return NodeStatus::SUCCESS;
    }

    // 단순 P 제어
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;
		double targetx = target.x, targety = target.y;
		double targettheta = atan2((targety-ctPosy),(targetx-ctPosx));
		
    double vx = targetx - gx;
    double vy = targety - gy;
    double theta = atan2(vy, vx);
    double dist = norm(vx, vy);
    double Kp = 4.0;
    double vtheta;
    vtheta = toPInPI((theta - gtheta) + (targettheta - theta));
    // if(dist > return_position_limit) vtheta = toPInPI(theta - gtheta);
    // else vtheta = toPInPI((theta - gtheta) + (targettheta - theta));
    vtheta *= Kp;

    // map 좌표계의 제어명령 vx,vy를 ego좌표계 제어명령으로 변환
    double controlx = vx*cos(gtheta) + vy*sin(gtheta);
    double controly = -vx*sin(gtheta) + vy*cos(gtheta);
    
    // 가까워질수록 속도가 줄어들도록
    double linearFactor = 1.5 / (1.0 + exp(-6.0 * (dist - 0.5)));
    controlx *= linearFactor;
    controly *= linearFactor;
    // double v = norm(vx, vy);

    // 속도 제한
    controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
    controly = cap(controly, vyLimit, -vyLimit);

    if (dist < stop_Threshold){
        controlx = 0;
        controly = 0;
        theta = 0;
    }
    
    brain->client->setVelocity(controlx, controly, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus GolieInitPos::tick(){
    double turn_Threshold;
    double stop_Threshold;
    double vxLimit;
    double vyLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    // 골대중앙위치
    double targetx, targety, targettheta;
    getInput("init_golie_pos_x", targetx); 
    getInput("init_golie_pos_y", targety); 
    getInput("init_golie_pos_theta", targettheta); 

    // 본인 위치
    auto gPos = brain->data->robotPoseToField;
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double targetdir = atan2(errory, errorx); // 내 위치에서 골대중앙을 이은 벡터의 각도
    double errortheta = targetdir - gtheta; // 이걸 P제어한다면 골대중앙을 쳐다볼것.

    double dist = norm(errorx, errory); // 골대중앙까지의 거리
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

    // 수정
    if(dist > turn_Threshold){ // 직진
      controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
      controly = cap(controly, vyLimit, -vyLimit);
      controltheta = errortheta * Kp;
    }
    else if(dist < turn_Threshold && dist > stop_Threshold){ // 선회
		  controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
      controly = cap(controly, vyLimit, -vyLimit);
	    controltheta = (targettheta - gtheta) * Kp; // 이러면 gtheta(로봇방향)이 targettheta를 바라봄
    }
    
    else if(dist < turn_Threshold && dist < stop_Threshold){ // 정지
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

		brain->client->setVelocity(controlx, controly, controltheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus PredictBallTraj::tick()
{
    // ===============================
    // 0) 초기값
    // ===============================
    const double R_meas;  // measurement noise (R)
    const double sigma_a;  // proccess noise (Q)
    const double P0_pos;  
    const double P0_vel;  
 
    getInput("R_meas", R_meas);
    getInput("sigma_a", sigma_a);
    getInput("P0_pos", P0_pos);
    getInput("P0_vel", P0_vel);

    // ===============================
    // 1) 측정값 (로봇 좌표계)
    // ===============================
    const double mx = brain->data->ball.posToRobot.x;
    const double my = brain->data->ball.posToRobot.y;

    // ===============================
    // 2) 로봇 odom (절대값)
    // ===============================
    const double ox = brain->data->robotPoseToOdom.x;
    const double oy = brain->data->robotPoseToOdom.y;
    const double otheta = brain->data->robotPoseToOdom.theta;

    // ===============================
    // 3) dt 계산
    // ===============================
    const auto now = brain->get_clock()->now();

    double dt = 0.03;
    if (has_prev_time_) {
        dt = (now - prev_time_).seconds();
        dt = std::clamp(dt, 1e-3, 0.2);
    }
    prev_time_ = now;
    has_prev_time_ = true;

    // ===============================
    // 4) ego-motion 계산 (odom -> 이전 로봇좌표계 기준)
    // ===============================
    double dx_r = 0.0, dy_r = 0.0, dtheta = 0.0;
    if (has_prev_odom_) {
        const double dx_o = ox - prev_ox_;
        const double dy_o = oy - prev_oy_;
        dtheta = toPInPI(otheta - prev_otheta_);

        // odom 이동벡터를 "이전 로봇 좌표계(+x forward, +y left)"로 회전
        const double c0 = std::cos(-prev_otheta_);
        const double s0 = std::sin(-prev_otheta_);

        dx_r = c0 * dx_o - s0 * dy_o;
        dy_r = s0 * dx_o + c0 * dy_o;
    }
    prev_ox_ = ox; prev_oy_ = oy; prev_otheta_ = otheta;
    has_prev_odom_ = true;

    // ===============================
    // 5) KF 초기화
    // ===============================
    if (!kf_initialized_) {
        x_ = mx; y_ = my;
        vx_ = 0.0; vy_ = 0.0;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                P_[i][j] = 0.0;

        P_[0][0] = P_[1][1] = P0_pos;
        P_[2][2] = P_[3][3] = P0_vel;

        kf_initialized_ = true;
        return NodeStatus::SUCCESS;
    }

    // ===============================
    // 6) 예측 단계 (CV + ego-motion)
    // ===============================
    const double c = std::cos(-dtheta);
    const double s = std::sin(-dtheta);

    // 6-1) 상태 예측
    const double px = x_ + vx_ * dt - dx_r;
    const double py = y_ + vy_ * dt - dy_r;

    const double x_pred  = c * px - s * py;
    const double y_pred  = s * px + c * py;
    const double vx_pred = c * vx_ - s * vy_;
    const double vy_pred = s * vx_ + c * vy_;

    // 6-2) 공분산 예측: P = F P F^T + Q
    const double F[4][4] = {
        { c, -s,  c*dt, -s*dt },
        { s,  c,  s*dt,  c*dt },
        { 0,  0,  c,    -s    },
        { 0,  0,  s,     c    }
    };

    const double sa2 = sigma_a * sigma_a;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    double Q[4][4] = {0};
    Q[0][0] = sa2 * (dt4 * 0.25);
    Q[1][1] = sa2 * (dt4 * 0.25);
    Q[0][2] = sa2 * (dt3 * 0.5);
    Q[2][0] = sa2 * (dt3 * 0.5);
    Q[1][3] = sa2 * (dt3 * 0.5);
    Q[3][1] = sa2 * (dt3 * 0.5);
    Q[2][2] = sa2 * (dt2);
    Q[3][3] = sa2 * (dt2);

    double FP[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += F[i][k] * P_[k][j];
            FP[i][j] = sum;
        }
    }

    double P_pred[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += FP[i][k] * F[j][k]; // F^T(k,j)=F(j,k)
            P_pred[i][j] = sum + Q[i][j];
        }
    }

    x_ = x_pred;
    y_ = y_pred;
    vx_ = vx_pred;
    vy_ = vy_pred;

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P_[i][j] = P_pred[i][j];

    // ===============================
    // 7) 업데이트 단계
    // ===============================
    const double r0 = mx - x_;
    const double r1 = my - y_;

    const double S00 = P_[0][0] + R_meas;
    const double S01 = P_[0][1];
    const double S10 = P_[1][0];
    const double S11 = P_[1][1] + R_meas;

    double detS = S00 * S11 - S01 * S10;
    if (std::fabs(detS) < 1e-12) detS = (detS >= 0 ? 1e-12 : -1e-12);

    const double invS00 =  S11 / detS;
    const double invS01 = -S01 / detS;
    const double invS10 = -S10 / detS;
    const double invS11 =  S00 / detS;

    const double K00 = P_[0][0] * invS00 + P_[0][1] * invS10;
    const double K01 = P_[0][0] * invS01 + P_[0][1] * invS11;

    const double K10 = P_[1][0] * invS00 + P_[1][1] * invS10;
    const double K11 = P_[1][0] * invS01 + P_[1][1] * invS11;

    const double K20 = P_[2][0] * invS00 + P_[2][1] * invS10;
    const double K21 = P_[2][0] * invS01 + P_[2][1] * invS11;

    const double K30 = P_[3][0] * invS00 + P_[3][1] * invS10;
    const double K31 = P_[3][0] * invS01 + P_[3][1] * invS11;

    x_  += K00 * r0 + K01 * r1;
    y_  += K10 * r0 + K11 * r1;
    vx_ += K20 * r0 + K21 * r1;
    vy_ += K30 * r0 + K31 * r1;

    // P = (I - K H) P
    double Pold[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Pold[i][j] = P_[i][j];

    double IKH[4][4] = {0};
    for (int i = 0; i < 4; ++i) IKH[i][i] = 1.0;

    IKH[0][0] -= K00; IKH[0][1] -= K01;
    IKH[1][0] -= K10; IKH[1][1] -= K11;
    IKH[2][0] -= K20; IKH[2][1] -= K21;
    IKH[3][0] -= K30; IKH[3][1] -= K31;

    double Pnew[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += IKH[i][k] * Pold[k][j];
            Pnew[i][j] = sum;
        }
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P_[i][j] = Pnew[i][j];

    // ===============================
    // 8) 미래 위치 예측 (horizon)
    // ===============================
    double horizon;
    getInput("horizon", horizon);

    const double pred_x = x_ + vx_ * horizon;
    const double pred_y = y_ + vy_ * horizon;

    // ===============================
    // 9) 시각화 (rerun)
    // ===============================
    brain->log->setTimeNow();

    auto gPos = brain->data->robotPoseToField;
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;

    const rerun::components::Vector2D measured_ball{mx, -my};
    const rerun::components::Vector2D filtered_ball{x_, -y_};
    const rerun::components::Vector2D predicted_ball{pred_x, -pred_y};

    brain->log->log(
        "field/measured_ball", // 측정된 공의 위치
        rerun::Arrows2D::from_vectors({measured_ball})
            .with_origins({{gx, gy}})
            .with_colors({0x00FF00FF})
            .with_radii(0.01f)
            .with_draw_order(30)
    );

    brain->log->log(
        "field/filtered_ball", // 필터 추정된 공의 위치 
        rerun::Arrows2D::from_vectors({filtered_ball})
            .with_origins({{gx, gy}})
            .with_colors({0x00FFFFFF})
            .with_radii(0.01f)
            .with_draw_order(31)
    );

    brain->log->log(
        "field/predicted_ball", // 예측된 공의 위치
        rerun::Arrows2D::from_vectors({predicted_ball})
            .with_origins({{gx, gy}})
            .with_colors({0xFFAA00FF})
            .with_radii(0.015f)
            .with_draw_order(32)
    );

    return NodeStatus::SUCCESS;
}
