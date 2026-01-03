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

}

NodeStatus CalcGoliePos::tick(){


	  double blockThreshold; // 뭔가 새로운 이름이...
    getInput("block_threshold", blockThreshold);

    double r;
    getInput("golie_radius", r);
    
    auto bPos = brain->data->ball.posToField; // 공 위치
		
		Pose2D ctPos; // 하드코딩된 골대중앙 위치
		ctPos.x = -4.5; ctPos.y = 0.0;
		
		double cx = ctPos.x, cy = ctPos.y;
		double bx = bPos.x, by = bPos.y;
		
		double dx = bx - cx; // 방향벡터
		double dy = by - cy;
		double d = norm(dx, dy);
		
		double ux = dx / d; // 단위방향벡터
		double uy = dy / d;
		
		// "반원" 선택 로직 
		if (ux < 0.0) {
    ux = -ux;
    uy = -uy;
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
    double vLimit;
    getInput("stop_threshold", stop_Threshold); 
    getInput("v_limit", vLimit);
    
    Pose2D ctPos;
    ctPos.x = -4.5; ctPos.y = 0.0;
    
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
		double targettheta = atan2((targety-ctPos.y),(targetx-ctPos.x));
		
    double vx = targetx - gx;
    double vy = targety - gy;
    double theta = atan2(vy, vx);
    double dist = norm(vx, vy);
    double Kp = 4.0;
    double vtheta = toPInPI((theta - gtheta) + (targettheta - theta));
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
    controlx = cap(controlx, vLimit, -vLimit*0.5);    
    controly = cap(controly, vLimit*0.5, -vLimit*0.5);

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
    double vLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("v_limit", vLimit);

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

    if(dist > turn_Threshold){// 직진
      controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vLimit, -vLimit*0.5);    
      controly = cap(controly, vLimit*0.5, -vLimit*0.5);
      controltheta = errortheta * Kp;
    }
    else if(dist < turn_Threshold && dist > stop_Threshold){ // 선회
		  controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vLimit, -vLimit*0.5);    
      controly = cap(controly, vLimit*0.5, -vLimit*0.5);
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