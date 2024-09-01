#ifndef CUSTOM_OPTIMIZATION_OBJECTIVE_H
#define CUSTOM_OPTIMIZATION_OBJECTIVE_H

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"

namespace ompl
{
    namespace base
    {
        class CustomOptimizationObjective : public PathLengthOptimizationObjective
        {
        public:
            CustomOptimizationObjective(const SpaceInformationPtr &si);

            Cost motionCost(const State *s1, const State *s2) const override;

        private:
            Cost cartesianCost(const State *s1, const State *s2) const;
        };
    }
}

#endif // CUSTOM_OPTIMIZATION_OBJECTIVE_H
