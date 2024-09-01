#include "ompl/base/objectives/CustomOptimizationObjective.h"
#include "ompl/base/spaces/SE3StateSpace.h" // 根据你的需求包含适当的头文件

namespace ompl
{
    namespace base
    {
        CustomOptimizationObjective::CustomOptimizationObjective(const SpaceInformationPtr &si)
          : PathLengthOptimizationObjective(si)
        {
            // 这里可以进行一些自定义初始化（如果需要）
        }

        ompl::base::Cost ompl::base::CustomOptimizationObjective::motionCost(const State *s1, const State *s2) const
        {
            // 获取关节空间路径长度的成本
            Cost jointSpaceCost = PathLengthOptimizationObjective::motionCost(s1, s2);

            // 计算笛卡尔空间路径长度的成本
            Cost cartesianSpaceCost = cartesianCost(s1, s2);

            // 使用 combineCosts 方法来合并成本
            return this->combineCosts(jointSpaceCost, cartesianSpaceCost);
        }


        Cost CustomOptimizationObjective::cartesianCost(const State *s1, const State *s2) const
        {
            // 获取笛卡尔坐标（假设使用 SE3 状态空间，如果使用其他空间，需根据实际情况调整）
            const auto *se3state1 = s1->as<SE3StateSpace::StateType>();
            const auto *se3state2 = s2->as<SE3StateSpace::StateType>();

            // 计算笛卡尔空间中的欧几里得距离
            double dx = se3state1->getX() - se3state2->getX();
            double dy = se3state1->getY() - se3state2->getY();
            double dz = se3state1->getZ() - se3state2->getZ();
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            // 返回一个 Cost 对象，该对象表示在笛卡尔空间中的距离
            return Cost(distance);
        }
    }
}
