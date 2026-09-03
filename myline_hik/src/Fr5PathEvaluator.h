#ifndef MYLINE_HIK_FR5_PATH_EVALUATOR_H
#define MYLINE_HIK_FR5_PATH_EVALUATOR_H

#include "AdaptiveScanPlanner.h"

#include <QMetaType>

#include <cstddef>
#include <vector>

namespace hik_fr5 {

struct PathEvaluationRequest {
    int actionId{-1};
    // When true, this request must continue from the preceding request's
    // terminal IK solution instead of selecting a fresh branch from the
    // robot's actual joint position.
    bool chainWithPrevious{false};
    hik_scan::Pose6D currentPose;
    hik_adaptive::ScanSegment segment;
};

struct PathEvaluationOptions {
    double maximumCartesianSampleStepMm{10.0};
    double maximumAngularSampleStepDeg{5.0};
    double minimumJointLimitMarginDeg{5.0};
    double minimumNormalizedSingularValue{0.02};
    double jacobianCharacteristicLengthMm{100.0};
    std::size_t maximumJacobianSamples{16U};
    std::size_t maximumPathSamples{512U};
};

}  // namespace hik_fr5

Q_DECLARE_METATYPE(hik_adaptive::RobotPathEvaluation)

#endif  // MYLINE_HIK_FR5_PATH_EVALUATOR_H
