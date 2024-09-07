#include "ModelTranslator/PushSoft.h"

PushSoft::PushSoft(std::shared_ptr<FileHandler> yamlReader, int _task_mode) : ModelTranslator(yamlReader),
                                                                              PushBaseClass(
                                                                                      std::shared_ptr<FileHandler>(),
                                                                                      "franka_gripper", "goal") {
    task_mode = _task_mode;

    std::string yamlFilePath;
    if(task_mode == PUSH_SOFT){
        yamlFilePath = "/TaskConfigs/soft_body_manipulation/push_soft.yaml";
    }
    else if(task_mode == PUSH_SOFT_RIGID){
        yamlFilePath = "/TaskConfigs/soft_body_manipulation/push_soft_into_rigid.yaml";
    }
    else{
        std::cerr << "Invalid task mode specified for push soft \n";
    }

    InitModelTranslator(yamlFilePath);
}

void PushSoft::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);

    // If task = push soft
    if(task_mode == PUSH_SOFT){
        // --------------- Residual 0 - average position of soft body to goal -----------------
        // Variable to count poses of all vertices to compute average position
        pose_6 soft_body_accumulator;
        soft_body_accumulator.position.setZero();

        // Accumulate all vertices positions
        pose_6 body_pose;
        for(int i = 0; i < full_state_vector.soft_bodies[0].num_vertices; i++){
            MuJoCo_helper->GetSoftBodyVertexPosGlobal(full_state_vector.soft_bodies[0].name, i, body_pose, d);

            soft_body_accumulator.position(0) += body_pose.position(0);
            soft_body_accumulator.position(1) += body_pose.position(1);
            soft_body_accumulator.position(2) += body_pose.position(2);
        }

        // Compute average position
        soft_body_accumulator.position(0) /= full_state_vector.soft_bodies[0].num_vertices;
        soft_body_accumulator.position(1) /= full_state_vector.soft_bodies[0].num_vertices;
        pose_6 soft_body_pose = soft_body_accumulator;
        soft_body_pose.position(2) = 0.0;

        // 2D euclidean distance between the goal and the soft body
        double diff_x = soft_body_accumulator.position(0) - residual_list[0].target[0];
        double diff_y = soft_body_accumulator.position(1) - residual_list[0].target[1];
        double dist = sqrt(pow(diff_x, 2)
                           + pow(diff_y, 2));

        // Residual 0 is euler distance of soft body to goal
        residuals(resid_index++, 0) = dist;

        // ---------------- Residual 1 - average velocity of soft body -----------------
        soft_body_accumulator.position.setZero();
        for(int i = 0; i < full_state_vector.soft_bodies[0].num_vertices; i++){
            MuJoCo_helper->GetSoftBodyVertexVel(full_state_vector.soft_bodies[0].name, i, body_pose, d);

            soft_body_accumulator.position(0) += body_pose.position(0);
            soft_body_accumulator.position(1) += body_pose.position(1);
            soft_body_accumulator.position(2) += body_pose.position(2);
        }

        // Compute average velocity
        soft_body_accumulator.position(0) /= full_state_vector.soft_bodies[0].num_vertices;
        soft_body_accumulator.position(1) /= full_state_vector.soft_bodies[0].num_vertices;

        // Residual 1 is average velocity of the soft body
        residuals(resid_index++, 0) = sqrt(pow(soft_body_accumulator.position(0), 2)
                                           + pow(soft_body_accumulator.position(1), 2)
                                           + pow(soft_body_accumulator.position(2), 2));

        // ---------------- Residual 2 - distance between EE and soft body average position -----------------

        pose_6 EE_pose;
        MuJoCo_helper->GetBodyPoseAngleViaXpos("franka_gripper", EE_pose, d);

        dist = sqrt(pow(EE_pose.position(0) - soft_body_pose.position(0), 2)
                    + pow(EE_pose.position(1) - soft_body_pose.position(1), 2)
                    + pow(EE_pose.position(2) - soft_body_pose.position(2), 2));

        residuals(resid_index++, 0) = dist - residual_list[2].target[0];

    }
    // If task = push soft into rigid
    else{

    }

//    int num_obstacles = 0;
//
//    pose_6 goal_pose;
//    pose_6 goal_vel;
//    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);
//    MuJoCo_helper->GetBodyVelocity("goal", goal_vel, d);
//
//    // --------------- Residual 0: Body goal position -----------------
//    double diff_x = goal_pose.position(0) - residual_list[0].target[0];
//    double diff_y = goal_pose.position(1) - residual_list[0].target[1];
//    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                       + pow(diff_y, 2));
//
//    // --------------- Residual 1: Body goal velocity -----------------
//    residuals(resid_index++, 0) = sqrt(pow(goal_vel.position(0), 2)
//                                       + pow(goal_vel.position(1), 2));
//
//    // Residuals 2 -> 2 + num_obstacles: Obstacle positions
//    for(int i = 0; i < num_obstacles; i++){
//        pose_6 obstacle_pose;
//        MuJoCo_helper->GetBodyPoseAngle("obstacle_" + std::to_string(i + 1), obstacle_pose, d);
//
//        diff_x = obstacle_pose.position(0) - full_state_vector.rigid_bodies[i + 1].start_linear_pos[0];
//        diff_y = obstacle_pose.position(1) - full_state_vector.rigid_bodies[i + 1].start_linear_pos[1];
//
//        residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                           + pow(diff_y, 2));
//    }
//
//    // --------------- Residual 3 + num_obstacles: Joint velocity -----------------
//    std::vector<double> joint_velocities;
//    MuJoCo_helper->GetRobotJointsVelocities("panda", joint_velocities, d);
//    residuals(resid_index++, 0) = joint_velocities[5];
//
//    // --------------- Residual 4 + num_obstacles: EE position towards goal object -----------------
//    pose_7 EE_pose;
//    MuJoCo_helper->GetBodyPoseQuatViaXpos("franka_gripper", EE_pose, d);
//    diff_x = EE_pose.position(0) - goal_pose.position(0);
//    diff_y = EE_pose.position(1) - goal_pose.position(1);
//    double diff_z = EE_pose.position(2) - goal_pose.position(2);
//    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                       + pow(diff_y, 2)
//                                       + pow(diff_z, 2));

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void PushSoft::ReturnRandomStartState(){

    // Randomly generate a start and goal x and y position for cylinder
    // Random generate a goal x and y position for cylinder
    float startX = randFloat(0.5, 0.501);
    float startY = randFloat(0, 0.01);

    float goalX = randFloat(0.65, 0.75);
    float goalY = randFloat(-0.1, 0.1);

    // Place rigid object at
    randomGoalX = goalX;
    randomGoalY = goalY;

    // Initialise soft body poses to start configuration
    for(auto & soft_body : full_state_vector.soft_bodies){
        pose_6 body_pose;

        body_pose.position[0] = 0.0;
        body_pose.position[1] = 1.0;
        body_pose.position[2] = 0.0;

        for(int i = 0; i < 3; i++){
            body_pose.orientation[i] = soft_body.start_angular_pos[i];
        }

        // TODO - better way to do this where we use the spacing information and transforms
        for(int i = 0; i < soft_body.num_vertices; i++){
            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, MuJoCo_helper->main_data);
        }
    }

    // step simulator
    for(int t = 0; t < 10; t++){
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
    }

    // Set up the goal positions and velocities
    // Robot start configuration
    double robot_start_config[7] = {0, 0.1, 0, -3, 0, 1.34, 0};

    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].start_pos[i] = robot_start_config[i];
    }

    if(task_mode == PUSH_SOFT_RIGID){
        full_state_vector.rigid_bodies[0].start_linear_pos[0] = startX;
        full_state_vector.rigid_bodies[0].start_linear_pos[1] = startY;
        full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.032;

        full_state_vector.rigid_bodies[0].start_angular_pos[0] = 0.0;
        full_state_vector.rigid_bodies[0].start_angular_pos[1] = 0.0;
        full_state_vector.rigid_bodies[0].start_angular_pos[2] = 0.0;
    }

    // Soft body
    full_state_vector.soft_bodies[0].start_linear_pos[0] = -0.8;
    full_state_vector.soft_bodies[0].start_linear_pos[1] = 1.0;
    full_state_vector.soft_bodies[0].start_linear_pos[2] = 0.1;
    std::cout << "soft body x: " << full_state_vector.soft_bodies[0].start_linear_pos[0] << " y: " << full_state_vector.soft_bodies[0].start_linear_pos[1] << "\n";
    full_state_vector.soft_bodies[0].start_angular_pos[0] = 0.0;
    full_state_vector.soft_bodies[0].start_angular_pos[1] = 0.0;
    full_state_vector.soft_bodies[0].start_angular_pos[2] = 0.0;

}

void PushSoft::ReturnRandomGoalState(){

    // Robot configuration doesnt matter for this task
//    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
//        full_state_vector.robots[0].goal_pos[i] = 0.0;
//        full_state_vector.robots[0].goal_vel[i] = 0.0;
//    }

    if(task_mode == PUSH_SOFT){

        // Soft body distractor
//        full_state_vector.soft_bodies[0].goal_linear_pos[0] = randomGoalX;
//        full_state_vector.soft_bodies[0].goal_linear_pos[1] = randomGoalY;
//        full_state_vector.soft_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.soft_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[2] = 0.0;
    }
    else if(task_mode == PUSH_SOFT_RIGID){
        // Goal object body
//        full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randomGoalX;
//        full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randomGoalY;
//        full_state_vector.rigid_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.rigid_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.rigid_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.rigid_bodies[0].goal_angular_pos[2] = 0.0;
//
//        // Soft body distractor
//        full_state_vector.soft_bodies[0].goal_linear_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_linear_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_linear_pos[2] = 0.0;
//
//        full_state_vector.soft_bodies[0].goal_angular_pos[0] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[1] = 0.0;
//        full_state_vector.soft_bodies[0].goal_angular_pos[2] = 0.0;
    }



}
//
////std::vector<MatrixXd> PushSoft::CreateInitSetupControls(int horizonLength){
////    std::vector<MatrixXd> initSetupControls;
////
////    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
////    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
////
////    // Pushing create init controls borken into three main steps
////    // Step 1 - create main waypoints we want to end-effector to pass through
////    m_point goalPos;
////    std::vector<m_point> mainWayPoints;
////    std::vector<int> mainWayPointsTimings;
////    std::vector<m_point> allWayPoints;
////    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
////    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
////    goalPos(2) = 0.0;
////    EEWayPointsSetup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//////    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//////    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;
////
////    // Step 2 - create all subwaypoints over the entire trajectory
////    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);
////
////    // Step 3 - follow the points via the jacobian
////    initSetupControls = JacobianEEControl(goalPos, allWayPoints);
////
////    return initSetupControls;
////}
//
std::vector<MatrixXd> PushSoft::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Get EE current posi

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    double angle_EE_push = 0.0;

    pose_6 current_pose;
    MuJoCo_helper->GetSoftBodyVertexPos(full_state_vector.soft_bodies[0].name,
                                        0, current_pose, MuJoCo_helper->main_data);

    if(task_mode == PUSH_SOFT_RIGID){
//        goal_pos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
        goal_pos(0) = residual_list[0].target[0];
//        goal_pos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
        goal_pos(1) = residual_list[0].target[1];
        goal_pos(2) = 0.3;

        angle_EE_push = 0.0;
    }
    else{
//        goal_pos(0) = current_state_vector.soft_bodies[0].goal_linear_pos[0];
//        goal_pos(1) = current_state_vector.soft_bodies[0].goal_linear_pos[1];

        goal_pos(0) = residual_list[0].target[0];
        goal_pos(1) = residual_list[0].target[1];
        goal_pos(2) = 0.3;

        pose_6 goal_obj_start;
        MuJoCo_helper->GetSoftBodyVertexPos(current_state_vector.soft_bodies[0].name, 0, goal_obj_start, MuJoCo_helper->master_reset_data);
        double diff_x = goal_pos(0) - goal_obj_start.position[0];
        double diff_y =  goal_pos(1) - goal_obj_start.position[1];

        angle_EE_push = atan2(diff_y, diff_x);
    }

    EEWayPointsPush(current_pose.position, goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
    cout << mainWayPoints.size() << " waypoints created" << endl;
    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
    cout << "mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(allWayPoints, angle_EE_push);

    return initControls;
}

bool PushSoft::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    if(task_mode == PUSH_SOFT){
        // Minimise the sum of all vertices from the goal position
        dist = 0.0;
        pose_6 vertex_pose;
        for(int i = 0; i < full_state_vector.soft_bodies[0].num_vertices; i++){
            MuJoCo_helper->GetSoftBodyVertexPosGlobal(full_state_vector.soft_bodies[0].name, i, vertex_pose, d);
            // TODO - fix this!
//            double diffX = full_state_vector.soft_bodies[0].goal_linear_pos[0] - vertex_pose.position[0];
//            double diffY = full_state_vector.soft_bodies[0].goal_linear_pos[1] - vertex_pose.position[1];
//            dist += sqrt(pow(diffX, 2) + pow(diffY, 2));
        }

        // dist = 15
//        std::cout << "dist: " << dist << "\n";
//        if(dist < 3.0){
//            taskComplete = true;
//        }

    }
    else if(task_mode == PUSH_SOFT_RIGID){
        pose_6 goal_pose;
        MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

        // TODO - fix this!
//        double x_diff = goal_pose.position(0) - full_state_vector.rigid_bodies[0].goal_linear_pos[0];
//        double y_diff = goal_pose.position(1) - full_state_vector.rigid_bodies[0].goal_linear_pos[1];
//        dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

//    std::cout << "dist: " << dist << "\n";
        if(dist < 0.03){
            taskComplete = true;
        }

        // if weve pushed too far in x direction, the task should stop
//        if(x_diff > 0.08){
//            taskComplete = true;
//            std::cout << "pushed too far \n";
//        }
    }

    return taskComplete;
}

void PushSoft::SetGoalVisuals(mjData *d){
    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("display_goal", goal_pose, d);

    // Set the goal object position
    goal_pose.position(0) = residual_list[0].target[0];
    goal_pose.position(1) = residual_list[0].target[1];
    goal_pose.position(2) = 0.032;
    MuJoCo_helper->SetBodyPoseAngle("display_goal", goal_pose, d);
}
