
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <time.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <MotionPlanner.h>

#define CYCLE_TIME_IN_SECONDS                   0.01
#define NUMBER_OF_DOFS                          7

using namespace Eigen;
using namespace rt_motion_planner;
namespace filters {
class IIR {
 public:
  // [n] Elements to filter
  // [m] Filter Order
  // [b_coef] Filter denominator (size: (m+1)x1)
  // [a_coed] Filter Numerator  (size: mx1)
  //  IIR(int n, int m, VectorXd b_coef, VectorXd a_coef);

  IIR(int n, int m, VectorXd b_coef, VectorXd a_coef)
      : N_elements_(n), order_(m), b_coef_(b_coef), a_coef_(a_coef) {
    Xmat_ = MatrixXd::Zero(N_elements_, order_ + 1);
    Ymat_ = MatrixXd::Zero(N_elements_, order_);
    flag_initialized = false;
  }

  ~IIR();

  VectorXd get_iir() {
    VectorXd iir = Xmat_ * b_coef_ - Ymat_ * a_coef_;
    // for (int i = 0; i < N_elements_; i++) {
    //   iir[i] = el_mat_.row(i).sum() / order_;
    // }
    return iir;
  }

  void shiftX(int n) {
    MatrixXd::Map(&Xmat_(0, n), N_elements_, ((order_ + 1) - n)) =
        MatrixXd::Map(&Xmat_(0, 0), N_elements_, ((order_ + 1) - n));
    MatrixXd::Map(&Xmat_(0, 0), N_elements_, n) =
        MatrixXd::Zero(N_elements_, n);
  }

  void shiftY(int n) {
    MatrixXd::Map(&Ymat_(0, n), N_elements_, (order_ - n)) =
        MatrixXd::Map(&Ymat_(0, 0), N_elements_, (order_ - n));
    MatrixXd::Map(&Ymat_(0, 0), N_elements_, n) =
        MatrixXd::Zero(N_elements_, n);
  }

  void addX(VectorXd new_vec) {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Xmat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM(
          "filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Mat Filter: " << el_mat_);
  }

  void addY(VectorXd new_vec) {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Ymat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM(
          "filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Mat Filter: " << el_mat_);
  }

  void init_full(VectorXd i_vector) {
    if (!flag_initialized) {
      for (int i = 0; i < order_ + 1; i++) {
        shiftX(1);
        addX(i_vector);
        if (i < order_) {
          shiftY(1);
          addY(get_iir());
        }
      }
      flag_initialized = true;
    }
  }

  VectorXd update(VectorXd new_vec) {
    if (!flag_initialized) {
      init_full(new_vec);
    }
    addX(new_vec);
    VectorXd out_filtered = get_iir();
    shiftX(1);
    shiftY(1);
    addY(out_filtered);

    return out_filtered;
  }

  //  VectorXd get_iir();
  //  void     shiftX(int n);
  //  void     shiftY(int n);
  //  void     init_full(VectorXd i_value);
  //  void     addX(VectorXd);
  //  void     addY(VectorXd);
  //  VectorXd update(VectorXd);

 private:
  int N_elements_;
  int order_;
  VectorXd a_coef_;
  VectorXd b_coef_;
  MatrixXd Xmat_;
  MatrixXd Ymat_;
  bool flag_initialized;
};  // class end

// ROS_INFO_STREAM(" Mat Initi: " << el_mat_);
}  // namespace filters
trajectory_msgs::JointTrajectory joint_right;
trajectory_msgs::JointTrajectory joint_left;

ros::Publisher right_command;
ros::Publisher left_command;
ros::Subscriber right_joint_sub;
ros::Subscriber left_joint_sub;
ros::Subscriber right_joint_upd;
ros::Subscriber left_joint_upd;
OnlineTrajGen *right_joint_traj_gen;
OnlineTrajGen *left_joint_traj_gen;

VectorXd des_right_joint_pos;
VectorXd des_left_joint_pos;
VectorXd des_right_joint_vel;
VectorXd des_left_joint_vel;

VectorXd right_joint_pos;
VectorXd right_joint_vel;
VectorXd right_joint_acc;

VectorXd left_joint_pos;
VectorXd left_joint_vel;
VectorXd left_joint_acc;

VectorXd right_joint_pos_cmd;
VectorXd right_joint_vel_cmd;

VectorXd left_joint_pos_cmd;
VectorXd left_joint_vel_cmd;

bool RightJPosCb_flag;
bool LeftJPosCb_flag;

bool RightJTarCb_flag;
bool LeftJTarCb_flag;

bool RightFirstPos_flag;
bool LeftFirstPos_flag;
bool RightFirstTar_flag;
bool LeftFirstTar_flag;
VectorXd a_coef(2);
VectorXd b_coef(3);
VectorXd pos_filter(7);
VectorXd vel_filter(7);
filters::IIR *force_filter;
filters::IIR *force_filter2;

void getRightJPosCb(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg){
    if(!RightFirstPos_flag){
        RightFirstPos_flag = true;
    }
    right_joint_pos = VectorXd::Map(&msg->actual.positions[0], msg->actual.positions.size());
    right_joint_vel = VectorXd::Map(&msg->actual.velocities[0], msg->actual.velocities.size());
    RightJPosCb_flag = true;
}

void getLeftJPosCb(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg){
    if(!LeftFirstPos_flag){
        LeftFirstPos_flag = true;
    }
    left_joint_pos = VectorXd::Map(&msg->actual.positions[0], msg->actual.positions.size());
    left_joint_vel = VectorXd::Map(&msg->actual.velocities[0], msg->actual.velocities.size());
    LeftJPosCb_flag = true;
}

void getRightJTarCb(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    if(!RightFirstTar_flag){
        RightFirstTar_flag = true;
    }
    des_right_joint_pos = VectorXd::Map(&msg->points[0].positions[0], msg->points[0].positions.size());
    RightJTarCb_flag = true;
}
void getLeftJTarCb(const trajectory_msgs::JointTrajectory::ConstPtr &msg){
    if(!LeftFirstTar_flag){
        LeftFirstTar_flag = true;
    }
    des_left_joint_pos = VectorXd::Map(&msg->points[0].positions[0], msg->points[0].positions.size());
    LeftJTarCb_flag = true;
}
void sendRightCmd(const VectorXd pos, const VectorXd vel){
    force_filter->update(pos);
    pos_filter = force_filter->get_iir();
    force_filter2->update(vel);
    vel_filter = force_filter2->get_iir();
    for (int j = 0; j < 7; ++j){
        joint_right.points[0].positions[j] = pos_filter[j];
        joint_right.points[0].velocities[j] = pos_filter[j];
    }
    right_command.publish(joint_right);
}

void sendLeftCmd(const VectorXd pos, const VectorXd vel){

    for (int j = 0; j < 7; ++j){
        joint_left.points[0].positions[j] = pos[j];
        joint_left.points[0].velocities[j] = vel[j];
    }
    left_command.publish(joint_left);
}

void init_(){

    RightJPosCb_flag = false;
    LeftJPosCb_flag = false;
    RightJTarCb_flag = false;
    LeftJTarCb_flag = false;
    RightFirstPos_flag = false;
    LeftFirstPos_flag = false;
    RightFirstTar_flag = false;
    LeftFirstTar_flag = false;
    ROS_INFO(" #####  get grasp command 1##########");
    des_right_joint_vel = VectorXd::Zero(7);
    des_left_joint_vel = VectorXd::Zero(7);
    right_joint_acc = VectorXd::Zero(7);
    left_joint_acc = VectorXd::Zero(7);
    ROS_INFO(" #####  get grasp command 2##########");
    joint_right.joint_names.push_back("arm_right_1_joint");
    joint_right.joint_names.push_back("arm_right_2_joint");
    joint_right.joint_names.push_back("arm_right_3_joint");
    joint_right.joint_names.push_back("arm_right_4_joint");
    joint_right.joint_names.push_back("arm_right_5_joint");
    joint_right.joint_names.push_back("arm_right_6_joint");
    joint_right.joint_names.push_back("arm_right_7_joint");
    joint_right.points.resize(1);
    joint_right.points[0].positions.resize(7);
    joint_right.points[0].velocities.resize(7);
    joint_right.points[0].time_from_start = ros::Duration(0.01);
    ROS_INFO(" #####  get grasp command 3##########");
    joint_left.joint_names.push_back("arm_left_1_joint");
    joint_left.joint_names.push_back("arm_left_2_joint");
    joint_left.joint_names.push_back("arm_left_3_joint");
    joint_left.joint_names.push_back("arm_left_4_joint");
    joint_left.joint_names.push_back("arm_left_5_joint");
    joint_left.joint_names.push_back("arm_left_6_joint");
    joint_left.joint_names.push_back("arm_left_7_joint");
    joint_left.points.resize(1);
    joint_left.points[0].positions.resize(7);
    joint_left.points[0].velocities.resize(7);
    joint_left.points[0].time_from_start = ros::Duration(0.01);
    
    a_coef << -1.82269493, 0.837181651;
    b_coef << 0.00362168151, 0.00724336303, 0.0036216815;
    force_filter = new filters::IIR(7, 2, b_coef, a_coef);
    force_filter2 = new filters::IIR(7, 2, b_coef, a_coef);

}

int main(int argc, char **argv)
{   
    ROS_INFO(" #####  get grasp command 555##########");
    ros::init(argc, argv, "reflex_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    ros::Rate loop_rate(200); //200hz
    spinner.start();
    

    init_();
    
    int ResultValue_right;
    int ResultValue_left;

    right_joint_traj_gen = new OnlineTrajGen(7, 0.05);
    left_joint_traj_gen = new OnlineTrajGen(7, 0.05);    

    RMLPositionOutputParameters *OPR;
    RMLPositionOutputParameters *OPL;

    
    right_joint_traj_gen->setLimit(0.4 * VectorXd::Ones(7), 500 * VectorXd::Ones(7), 500 * VectorXd::Ones(7)); //0.4 500 500
    right_joint_traj_gen->setStateReset();
    left_joint_traj_gen->setLimit(0.4 * VectorXd::Ones(7), 500 * VectorXd::Ones(7), 500 * VectorXd::Ones(7));
    left_joint_traj_gen->setStateReset();


    
    right_command = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_right_controller/command", 1);
    left_command = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_left_controller/command", 1);

    right_joint_sub = nh.subscribe("/arm_right_controller/state", 1, getRightJPosCb, ros::TransportHints().tcpNoDelay(true));
    left_joint_sub = nh.subscribe("/arm_left_controller/state", 1, getLeftJPosCb, ros::TransportHints().tcpNoDelay(true));

    right_joint_upd = nh.subscribe("/state_center/right_joint_upd", 1, getRightJTarCb, ros::TransportHints().tcpNoDelay(true));
    left_joint_upd = nh.subscribe("/state_center/left_joint_upd", 1, getLeftJTarCb, ros::TransportHints().tcpNoDelay(true));
    ROS_INFO(" #####  get grasp command 10##########");
    while (ros::ok())
    {
        
        if(RightFirstPos_flag && RightFirstTar_flag){
            
            if(RightJPosCb_flag){
                right_joint_traj_gen->setCurrIn(right_joint_pos, right_joint_vel, right_joint_acc);
                RightJPosCb_flag = false;
                
            }
            if(RightJTarCb_flag){
                right_joint_traj_gen->setTarget(des_right_joint_pos, des_right_joint_vel);
                RightJTarCb_flag = false;
                right_joint_traj_gen->setStateReset();
                
            }
            if (right_joint_traj_gen->getState() != ReflexxesAPI::RML_FINAL_STATE_REACHED){
                ResultValue_right = right_joint_traj_gen->initRml();
                
                if (ResultValue_right < 0){
                    ROS_ERROR_STREAM("Traj. Generation failed" << ResultValue_right);
                }
                else{
                    RMLPositionOutputParameters *OPL;
                    OPR = right_joint_traj_gen->get_output();
                    right_joint_traj_gen->updateCurrIn(OPR);
                    
                    right_joint_pos_cmd = VectorXd::Map(&OPR->NewPositionVector->VecData[0], 7);
                    right_joint_vel_cmd = VectorXd::Map(&OPR->NewVelocityVector->VecData[0], 7);
                    sendRightCmd(right_joint_pos_cmd, right_joint_vel_cmd);
                    ROS_INFO_STREAM("position_right"<<right_joint_pos_cmd[0]);
                    ROS_INFO_STREAM("velocity_right"<<right_joint_vel_cmd[0]);
                }
            }
        }
        if(LeftFirstPos_flag && LeftFirstTar_flag){
            if(LeftJPosCb_flag){
                left_joint_traj_gen->setCurrIn(left_joint_pos, left_joint_vel, left_joint_acc);
                LeftJPosCb_flag = false;
            }
            if(LeftJTarCb_flag){
                left_joint_traj_gen->setTarget(des_left_joint_pos, des_left_joint_vel);
                LeftJTarCb_flag = false;
                left_joint_traj_gen->setStateReset();
            }
            if (left_joint_traj_gen->getState() != ReflexxesAPI::RML_FINAL_STATE_REACHED){
                ResultValue_left = left_joint_traj_gen->initRml();

                if (ResultValue_left < 0){
                    ROS_ERROR_STREAM("Traj. Generation failed" << ResultValue_left);
                }
                else{
                    RMLPositionOutputParameters *OPL;
                    OPL = left_joint_traj_gen->get_output();
                    left_joint_traj_gen->updateCurrIn(OPL);
                    left_joint_pos_cmd = VectorXd::Map(&OPL->NewPositionVector->VecData[0], 7);
                    left_joint_vel_cmd = VectorXd::Map(&OPL->NewVelocityVector->VecData[0], 7);
                    sendLeftCmd(left_joint_pos_cmd, left_joint_vel_cmd);
                    ROS_INFO_STREAM("position_left"<<left_joint_pos_cmd[0]);
                    ROS_INFO_STREAM("velocity_left"<<left_joint_vel_cmd[0]);
                }
            }
        }
        loop_rate.sleep();
    }
    

    // int i;
    // ros::init(argc, argv, "main");
    // ros::start();

    // int                         ResultValue                 =   0       ;

    // ReflexxesAPI                *RML                        =   NULL    ;

    // RMLPositionInputParameters  *IP                         =   NULL    ;

    // RMLPositionOutputParameters *OP                         =   NULL    ;

    // RMLPositionFlags            Flags                                   ;

    // RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
    //                                         ,   CYCLE_TIME_IN_SECONDS   );

    // IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    // OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // IP->CurrentPositionVector->VecData      [0] =    100.0      ;
    // IP->CurrentPositionVector->VecData      [1] =      0.0      ;
    // IP->CurrentPositionVector->VecData      [2] =     50.0      ;

    // IP->CurrentVelocityVector->VecData      [0] =    100.0      ;
    // IP->CurrentVelocityVector->VecData      [1] =   -220.0      ;
    // IP->CurrentVelocityVector->VecData      [2] =    -50.0      ;

    // IP->CurrentAccelerationVector->VecData  [0] =   -150.0      ;
    // IP->CurrentAccelerationVector->VecData  [1] =    250.0      ;
    // IP->CurrentAccelerationVector->VecData  [2] =    -50.0      ;

    // IP->MaxVelocityVector->VecData          [0] =    300.0      ;
    // IP->MaxVelocityVector->VecData          [1] =    100.0      ;
    // IP->MaxVelocityVector->VecData          [2] =    300.0      ;

    // IP->MaxAccelerationVector->VecData      [0] =    300.0      ;
    // IP->MaxAccelerationVector->VecData      [1] =    200.0      ;
    // IP->MaxAccelerationVector->VecData      [2] =    100.0      ;

    // IP->MaxJerkVector->VecData              [0] =    400.0      ;
    // IP->MaxJerkVector->VecData              [1] =    300.0      ;
    // IP->MaxJerkVector->VecData              [2] =    200.0      ;

    // IP->TargetPositionVector->VecData       [0] =   -600.0      ;
    // IP->TargetPositionVector->VecData       [1] =   -200.0      ;
    // IP->TargetPositionVector->VecData       [2] =   -350.0      ;

    // IP->TargetVelocityVector->VecData       [0] =    50.0       ;
    // IP->TargetVelocityVector->VecData       [1] =   -50.0       ;
    // IP->TargetVelocityVector->VecData       [2] =  -200.0       ;

    // IP->SelectionVector->VecData            [0] =   true        ;
    // IP->SelectionVector->VecData            [1] =   true        ;
    // IP->SelectionVector->VecData            [2] =   true        ;

    // while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    // {   
    //     i++;

    //     ResultValue =   RML->RMLPosition(       *IP
    //                                         ,   OP
    //                                         ,   Flags       );

    //     if (ResultValue < 0)
    //     {
    //         printf("An error occurred (%d).\n", ResultValue );
    //         break;
    //     }

    //     *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
    //     *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
    //     *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
    //     printf("Round (%d)", i);
    //     printf("%f",IP->CurrentPositionVector->VecData[0]);
    // }
    // printf("finished");

    // delete  RML         ;
    // delete  IP          ;
    // delete  OP          ;

    // exit(EXIT_SUCCESS)  ;
    delete right_joint_traj_gen;
    delete left_joint_traj_gen;

}
