#pragma once

#include "MRISAM2.h"

#include <gtsam/geometry/Pose2.h>

namespace gtsam {
class FrontEnd {
 public: 
  FrontEnd() {}

  struct StepData {
    MRISAM2::RootID root_id;
    NonlinearFactorGraph graph; // TODO: use shared_ptr
    Values values;

    void print() const {
      std::cout << "root id: " << root_id << "\n";
      graph.print("new factors:", MultiRobotKeyFormatter);
      values.print("new values:", MultiRobotKeyFormatter);
    }

  };

  struct InitCondition {
    NonlinearFactorGraph graph;
    Values values;
    Ordering order;
    MRISAM2::RootID root_id;
    MRISAM2::RootKeySetMap other_root_keys_map;
    MRISAM2::RootKeyMap root_recent_state_map;
  };

  static Key RobotPoseKey(const MRISAM2::RootID root_id, const size_t step);

  static Key LandmarkKey(const size_t landmark_id);

  virtual std::vector<StepData> step(const Values& old_values, const size_t first_step, const size_t num_steps) const = 0;

  virtual InitCondition initialCondition() const = 0;

  virtual const Values& gtValues() const = 0;

  virtual size_t maxSteps() const = 0;
};

class FrontEndCityTrees : public FrontEnd {
 public:
  FrontEndCityTrees(size_t num_robots, bool load_gt = false);

  std::vector<StepData> step(const Values& old_values, const size_t first_step, const size_t num_steps) const override;

  InitCondition initialCondition() const override;

  size_t maxSteps() const override {return max_steps_; }

  Key RobotPoseKeyFromId(const size_t pose_id) const;

  const NonlinearFactorGraph& odomGraph() const {return odom_graph_; }

  const NonlinearFactorGraph& landmarkMeasGraph() const {return landmark_meas_graph_; }

  const Values& gtValues() const override {return gt_values_; }

 private:
  void loadDataSet(bool load_gt);

  size_t num_robots_;
  size_t max_steps_;
  size_t states_per_robot_;
  std::vector<size_t> robot_initial_index_vec_;
  std::vector<Pose2> odom_poses_;
  std::vector<Point2> landmark_measures_;
  NonlinearFactorGraph odom_graph_, landmark_meas_graph_;
  VariableIndex vi_odom_, vi_landmark_meas_;
  Values gt_values_;
  noiseModel::Diagonal::shared_ptr pose_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 1.0/50.0, 1.0/50.0, 1.0/100.0).finished());
  noiseModel::Diagonal::shared_ptr point_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(2) << 1.0/10.0, 1.0/10.0).finished());
  noiseModel::Diagonal::shared_ptr prior_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0001, 0.0001, 0.0001).finished());
};

class FrontEndUTIAS : public FrontEnd {
 public: 
  typedef std::pair<double, double> BRMeasure;

  FrontEndUTIAS(size_t dataset_id, bool include_inter_robot_measurements);

  size_t maxSteps() const override {return max_steps_; }

  const Values& gtValues() const override {return gt_values_; }

  InitCondition initialCondition() const override;

  InitCondition initialConditionNoAnchor() const;

  std::vector<StepData> step(const Values& old_values, const size_t first_step, const size_t num_steps) const override;

  struct Measurements {
    NonlinearFactorGraph landmark_meas_graph;
    NonlinearFactorGraph odom_graph;
    VariableIndex vi_landmark_meas;
    std::vector<Pose2> odom_poses;
    std::vector<BRMeasure> landmark_measures;

    Measurements() : landmark_meas_graph(), odom_graph(), odom_poses(), landmark_measures() {} 
  };

 private:
  Point2 calcFromBRMeasure(const Pose2& pose, const BRMeasure& br_measure) const;
  void readLandmarksGT(size_t dataset_id);
  void readPosesGT(size_t dataset_id);
  void readOdometry(size_t dataset_id);
  void readLandmarkMeas(size_t dataset_id);
  void readRobotMeas(size_t datset_id);
  void loadDataSet(size_t dataset_id);

  size_t num_robots_;
  bool include_inter_robot_measurements_;
  NonlinearFactorGraph robot_meas_graph_;
  std::vector<BRMeasure> robot_measures_;
  VariableIndex vi_robot_meas_;
  size_t max_steps_;
  Values gt_values_;

  std::vector<Measurements> measurements_vec_;

  noiseModel::Diagonal::shared_ptr odom_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 4e-3, 8e-4, 2e-2).finished());
  noiseModel::Diagonal::shared_ptr landmark_meas_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(2) << 2e-2, 0.14).finished());
  noiseModel::Diagonal::shared_ptr robot_meas_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(2) << 2e-2, 0.14).finished());
  noiseModel::Diagonal::shared_ptr prior_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0001, 0.0001, 0.0001).finished());
};

}