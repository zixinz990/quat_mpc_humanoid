#include "RobotState.h"

namespace robot {
class Kinematics {
   public:
    Kinematics(){};

    static Eigen::Matrix<double, 3, LEG_DOF> cal_left_foot_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
    static Eigen::Matrix<double, 3, LEG_DOF> cal_right_foot_jac(const Eigen::Matrix<double, LEG_DOF, 1> joint_pos);
};
}  // namespace robot
