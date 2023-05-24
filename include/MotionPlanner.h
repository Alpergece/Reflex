#ifndef ROBOT_MOTION_PLANNER_H
#define ROBOT_MOTION_PLANNER_H

// Eigen
#include <Eigen/Dense>

// Eigen Conversions
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>


// Reflexxes
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>

// C++ Std 11
#include <iostream>

using namespace Eigen;

//
// template <typename T>
// struct Vector
// {
//
// } ;

namespace rt_motion_planner
{

class OnlineTrajGen
{

public:
  OnlineTrajGen(int n_dofs, double cycle_time_s)
      : n_dofs_(n_dofs), cycle_time_s_(cycle_time_s)
  {

    RML = new ReflexxesAPI(n_dofs_, cycle_time_s);

    IP = new RMLPositionInputParameters(n_dofs_);

    OP = new RMLPositionOutputParameters(n_dofs_);

  }; // Constructor

  ~OnlineTrajGen()
  {
    delete RML;
    delete IP;
    delete OP;
  }; // Destructor

  // Functions
  void control_loop();

  int initRml()
  {

    for (int i = 0; i < n_dofs_; i++) {
      IP->SelectionVector->VecData[i] = true;
    }

    int resultValue = RML->RMLPosition(*IP, OP, Flags);
    state_          = resultValue;
    return resultValue;
  };

  RMLPositionOutputParameters *get_output() const { return OP; };

  void updateCurrIn(const RMLPositionOutputParameters *OP)
  {
    *IP->CurrentPositionVector     = *OP->NewPositionVector;
    *IP->CurrentVelocityVector     = *OP->NewVelocityVector;
    *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
  };

  void setCurrIn(const VectorXd CurrPos, const VectorXd CurrVel,
                 const VectorXd CurrAcc)
  {
    for (int i = 0; i < n_dofs_; i++) {
      IP->CurrentPositionVector->VecData[i]     = CurrPos[i];
      IP->CurrentVelocityVector->VecData[i]     = CurrVel[i];
      IP->CurrentAccelerationVector->VecData[i] = CurrAcc[i];
    }
  };

  void printActual() const
  {
    std::cout << "Printing Current Joint Values" << std::endl;
    for (int i = 0; i < n_dofs_; i++) {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\t Cur. Pos: " << IP->CurrentPositionVector->VecData[i]
                << " \t Cur. Vel: " << IP->CurrentVelocityVector->VecData[i]
                << " \t Cur. Acc: " << IP->CurrentAccelerationVector->VecData[i]
                << "\n";
    }
  };

  void setTarget(const VectorXd TargetPos, VectorXd TargetVel)
  {

    for (int i = 0; i < n_dofs_; i++) {
      IP->TargetPositionVector->VecData[i] = TargetPos[i];
      IP->TargetVelocityVector->VecData[i] = TargetVel[i];
    }
    int resultValue = RML->RMLPosition(*IP, OP, Flags);
    state_          = resultValue;
    // ROS_WARN_STREAM("state_: " << state_ << " -> "
    //                          << ReflexxesAPI::RML_FINAL_STATE_REACHED);
  };

  void printTarget() const
  {
    std::cout << "Printing Target Joint Values" << std::endl;

    for (int i = 0; i < n_dofs_; i++) {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\tDes. Pos: " << IP->TargetPositionVector->VecData[i]
                << " \tDes. Vel: " << IP->TargetVelocityVector->VecData[i]
                << "\n";
    }
  };

  void setLimit(const VectorXd MaxVel, VectorXd MaxAcc)
  {
    for (int i = 0; i < n_dofs_; i++) {
      IP->MaxVelocityVector->VecData[i]     = MaxVel[i];
      IP->MaxAccelerationVector->VecData[i] = MaxAcc[i];
    }
  };

  void setLimit(const VectorXd MaxVel, VectorXd MaxAcc, VectorXd MaxJerk)
  {
    for (int i = 0; i < n_dofs_; i++) {
      IP->MaxVelocityVector->VecData[i]     = MaxVel[i];
      IP->MaxAccelerationVector->VecData[i] = MaxAcc[i];
      IP->MaxJerkVector->VecData[i]         = MaxJerk[i];
    }
  };

  void printLimits() const
  {
    std::cout << "Printing  Joint Limits" << std::endl;

    for (int i = 0; i < n_dofs_; i++) {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\tMax. Vel: " << IP->MaxVelocityVector->VecData[i]
                << " \tMax. Acc: " << IP->MaxAccelerationVector->VecData[i]
                << " \tMax. Jerk: " << IP->MaxJerkVector->VecData[i] << "\n";
    }
  };

  void setStateReset() { state_ = 0; }

  int getState() { return state_; }

private:
  int    n_dofs_;
  double cycle_time_s_;
  int    state_; //[0] Ready [1] Interpolation Ended

  // Reflexxes
  ReflexxesAPI *               RML;
  RMLPositionInputParameters * IP;
  RMLPositionOutputParameters *OP;
  RMLPositionFlags             Flags;
};

} // namespace rt_motion_planner

#endif
