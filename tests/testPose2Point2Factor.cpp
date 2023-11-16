#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "Pose2Point2Factor.h"

using namespace gtsam;

TEST(PoseFactor, error) {
  // create functor

  Pose2 pose(1, 1, M_PI_2);
  Point2 point(0, 2);
  Point2 measure(1, 1);


  Key pose_key = Symbol('p', 0);
  Key point_key = Symbol('p', 1);
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Gaussian::Covariance(gtsam::I_2x2);

  Pose2Point2Factor factor(pose_key, point_key, measure, cost_model);
  auto actual_errors = factor.evaluateError(pose, point);

  // check value
  auto expected_errors = (gtsam::Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(pose_key, pose);
  values.insert(point_key, point);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-6);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
