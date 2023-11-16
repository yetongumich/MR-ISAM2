#include "FrontEnd.h"
#include "Pose2Point2Factor.h"

#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <fstream>
#include <string>

namespace gtsam {

typedef BearingRangeFactor<Pose2, Point2> BearingRangeFactor2D;
typedef BearingRangeFactor<Pose2, Pose2> PoseBearingRangeFactor2D;

/* ************************************************************************* */
Key FrontEnd::RobotPoseKey(const MRISAM2::RootID root_id, const size_t step) {
  return LabeledSymbol('X', 97+root_id, step);
}

Key FrontEnd::LandmarkKey(const size_t landmark_id) {
  return LabeledSymbol('L', 0, landmark_id);
}

Key FrontEndCityTrees::RobotPoseKeyFromId(const size_t pose_id) const {
  size_t root_id = pose_id / states_per_robot_;
  size_t step = pose_id % states_per_robot_;
  return FrontEnd::RobotPoseKey(root_id, step);
}

/* ************************************************************************* */
void FrontEndCityTrees::loadDataSet(bool load_gt) {
  std::string data_file_name = load_gt ? "../../data/cityTrees10000_groundtruth.txt" : "../../data/cityTrees10000.txt";
  std::string gt_file_name = "../../data/cityTrees10000_gtvalue.txt";
  std::ifstream data_file(data_file_name);

  // read data file
  std::string line;
  while (getline(data_file, line)) {
    std::vector<std::string> parts;
    split(parts, line, boost::is_any_of(" "));

    if (parts[0] == "EDGE2") {   
      size_t pose_id_s = stoi(parts[1]);
      size_t pose_id_t = stoi(parts[2]);

      double odom_x, odom_y, odom_rad;
      odom_x = stod(parts[3]);
      odom_y = stod(parts[4]);
      odom_rad = stod(parts[5]);
      Pose2 odom_pose = Pose2(odom_x, odom_y, odom_rad);
      odom_poses_.push_back(odom_pose);
      odom_graph_.add(BetweenFactor<Pose2>(RobotPoseKeyFromId(pose_id_s), RobotPoseKeyFromId(pose_id_t), odom_pose, pose_noise_model_));
    }

    else if (parts[0] == "LANDMARK") {
      size_t pose_id = stoi(parts[1]);
      size_t landmark_id = stoi(parts[2]);
      double rel_x = stod(parts[3]);
      double rel_y = stod(parts[4]);
      Point2 measured_point(rel_x, rel_y);
      landmark_measures_.push_back(measured_point);
      landmark_meas_graph_.add(Pose2Point2Factor(RobotPoseKeyFromId(pose_id), LandmarkKey(landmark_id), measured_point, point_noise_model_));
    }
  }
  data_file.close();
  vi_odom_ = VariableIndex(odom_graph_);
  vi_landmark_meas_ = VariableIndex(landmark_meas_graph_);


  // read gt_file
  std::ifstream gt_file(gt_file_name);
  while (getline(gt_file, line)) {
    std::vector<std::string> parts;
    split(parts, line, boost::is_any_of(" "));

    if (parts[0] == "POSE") {   
      size_t pose_id = stoi(parts[1]);
      double pose_x, pose_y, pose_rad;
      pose_x = stod(parts[2]);
      pose_y = stod(parts[3]);
      pose_rad = stod(parts[4]);
      Pose2 pose = Pose2(pose_x, pose_y, pose_rad);
      gt_values_.insert(RobotPoseKeyFromId(pose_id), pose);
    }

    else if (parts[0] == "LANDMARK") {
      size_t landmark_id = stoi(parts[1]);
      double point_x = stod(parts[2]);
      double point_y = stod(parts[3]);
      Point2 point(point_x, point_y);
      gt_values_.insert(LandmarkKey(landmark_id), point);
    }
  }
  gt_file.close();
  // gt_values_.print("", MultiRobotKeyFormatter);
}

// /* ************************************************************************* */
FrontEndCityTrees::FrontEndCityTrees(size_t num_robots, bool load_gt) : FrontEnd(), num_robots_(num_robots) {

  size_t num_states = 10000;
  states_per_robot_ = num_states / num_robots;
  max_steps_ = states_per_robot_ - 1;

  robot_initial_index_vec_.resize(0);
  for (size_t r = 0; r<num_robots; r++) {
    robot_initial_index_vec_.push_back(r * states_per_robot_);
  }

  loadDataSet(load_gt);
}

/* ************************************************************************* */
FrontEnd::InitCondition FrontEndCityTrees::initialCondition() const {
  InitCondition init_condition;
  Key origin_key = LabeledSymbol('O', 0, 0);
  Pose2 origin_pose = Pose2(0, 0, 0);
  init_condition.graph.add(PriorFactor<Pose2>(origin_key, origin_pose, prior_noise_model_));
  init_condition.values.insert(origin_key, origin_pose);
  KeyVector keys_order;
  keys_order.push_back(origin_key);
  for (size_t r=0; r<num_robots_; r++) {
    Pose2 groundtruth_pose = gt_values_.at<Pose2>(RobotPoseKey(r, 0));
    init_condition.graph.add(BetweenFactor<Pose2>(origin_key, RobotPoseKey(r, 0), groundtruth_pose, prior_noise_model_));
    init_condition.values.insert(RobotPoseKey(r, 0), groundtruth_pose);
    if (r!=num_robots_-1) {
      KeySet root_keys;
      root_keys.insert(RobotPoseKey(r, 0));
      keys_order.push_back(RobotPoseKey(r, 0));
      init_condition.other_root_keys_map[r] = root_keys;
    }
  }
  init_condition.order = Ordering(keys_order);
  init_condition.root_id = num_robots_-1;
  return init_condition;
}


/* ************************************************************************* */
std::vector<FrontEnd::StepData> FrontEndCityTrees::step(const Values& old_values, const size_t first_step, const size_t num_steps) const {

  std::vector<FrontEnd::StepData> step_data_vec;

  Values all_new_values;

  for (size_t r=0; r<num_robots_; r++) {

    StepData step_data;
    step_data.root_id = r;
    size_t robot_first_step = states_per_robot_ * r + first_step;

    // add the odometry factors
    // std::cout << "add the odometry factors\n";
    for (size_t i = 0; i<num_steps; i++) {
      step_data.graph.push_back(odom_graph_.at(robot_first_step+i-1));
    }

    // add the pose values
    // std::cout << "add the pose values\n";
    Pose2 current_pose = old_values.at<Pose2>(RobotPoseKey(r, first_step-1));
    std::vector<Pose2> new_poses;
    Values new_pose_values;
    for (size_t i=0; i<num_steps; i++) {
      size_t pose_id = robot_first_step + i;
      Pose2 rel_pose = odom_poses_[pose_id-1];
      current_pose = current_pose.compose(rel_pose);
      new_pose_values.insert(RobotPoseKey(r, first_step+i), current_pose);
    }
    step_data.values.insert(new_pose_values);
    all_new_values.insert(new_pose_values);

    // add the landmark measurement factors
    // std::cout << "add the landmark measurement factors\n";
    KeySet landmark_keys;
    for (Key pose_key : new_pose_values.keys()) {
      if (vi_landmark_meas_.find(pose_key) != vi_landmark_meas_.end()) {
        auto factor_indices = vi_landmark_meas_[pose_key];
        for (auto factor_index : factor_indices) {
          step_data.graph.push_back(landmark_meas_graph_.at(factor_index));
          landmark_keys.insert(landmark_meas_graph_.at(factor_index)->keys()[1]);
        }
      }
    }

    // add the landmark values
    // std::cout << "add the landmark values\n";
    Values new_landmark_values;
    for (Key landmark_key : landmark_keys) {
      if (!old_values.exists(landmark_key) && (!all_new_values.exists(landmark_key))) {
        
        std::vector<Point2> landmark_point_vec;
        auto factor_indices = vi_landmark_meas_[landmark_key];
        
        for (auto factor_index : factor_indices) {
          Key pose_key = landmark_meas_graph_.at(factor_index)->keys()[0];
          Pose2 pose;
          if (old_values.exists(pose_key)) {
            pose = old_values.at<Pose2>(pose_key);
          }
          else if (all_new_values.exists(pose_key)) {
            pose = all_new_values.at<Pose2>(pose_key);
          }
          else {
            continue;
          }
          Point2 measurement = landmark_measures_[factor_index];
          landmark_point_vec.push_back(pose.transformFrom(measurement)); // TODO: check if this is correct
        }

        Point2 ave_point(0, 0);
        for (Point2& point : landmark_point_vec) {
          ave_point += point;
        }
        ave_point /= (double)landmark_point_vec.size();

        new_landmark_values.insert(landmark_key, ave_point);
      }
    }
    step_data.values.insert(new_landmark_values);
    all_new_values.insert(new_landmark_values);

    step_data_vec.push_back(step_data);
  }
  return step_data_vec;
}

/* ************************************************************************* */
FrontEndUTIAS::FrontEndUTIAS(size_t dataset_id,
                             bool include_inter_robot_measurements)
    : FrontEnd(),
      num_robots_(5),
      include_inter_robot_measurements_(include_inter_robot_measurements) {
  loadDataSet(dataset_id);
}

/* ************************************************************************* */
void FrontEndUTIAS::readLandmarksGT(size_t dataset_id) {
  std::string landmark_file_name = "../../data/Dataset" + std::to_string(dataset_id) + "/gt_landmark.txt";
  std::ifstream data_file(landmark_file_name);
  std::string line;
  while (getline(data_file, line)) {
    std::vector<std::string> parts;
    split(parts, line, boost::is_any_of(" "));
    size_t landmark_id = stoi(parts[0]);
    double landmark_x = stod(parts[1]);
    double landmark_y = stod(parts[2]);
    gt_values_.insert(LandmarkKey(landmark_id), Point2(landmark_x, landmark_y));
  }
  data_file.close();
}

/* ************************************************************************* */
void FrontEndUTIAS::readPosesGT(size_t dataset_id) {
  max_steps_ = 0;
  for (size_t r = 1; r<=num_robots_; r++) {
    std::string pose_gt_file_name = "../../data/Dataset" + std::to_string(dataset_id) + "/gt_" + std::to_string(r) + ".txt";
    std::ifstream data_file(pose_gt_file_name);
    std::string line;
    while (getline(data_file, line)) {
      std::vector<std::string> parts;
      split(parts, line, boost::is_any_of(" "));
      size_t step_id = stoi(parts[0]);
      double pose_x = stod(parts[1]);
      double pose_y = stod(parts[2]);
      double pose_theta = stod(parts[3]);
      gt_values_.insert(RobotPoseKey(r, step_id), Pose2(pose_x, pose_y, pose_theta));
      max_steps_ = std::max(max_steps_, step_id);
    }
    data_file.close();
  }
}

/* ************************************************************************* */
void FrontEndUTIAS::readOdometry(size_t dataset_id) {
  for (size_t r = 1; r<=num_robots_; r++) {
    NonlinearFactorGraph odom_graph;
    std::string odom_file_name = "../../data/Dataset" + std::to_string(dataset_id) + "/odometry_" + std::to_string(r) + ".txt";
    std::ifstream data_file(odom_file_name);
    std::string line;
    while (getline(data_file, line)) {
      
      std::vector<std::string> parts;
      split(parts, line, boost::is_any_of(" "));
      size_t step1_id = stoi(parts[0]);
      size_t step2_id = stoi(parts[1]);
      double odom_x = stod(parts[2]);
      double odom_y = stod(parts[3]);
      double odom_rad = stod(parts[4]);

      Pose2 odom_pose = Pose2(odom_x, odom_y, odom_rad);
      measurements_vec_[r-1].odom_poses.push_back(odom_pose);
      measurements_vec_[r-1].odom_graph.add(BetweenFactor<Pose2>(RobotPoseKey(r, step1_id), RobotPoseKey(r, step2_id), odom_pose, odom_noise_model_));
    }
    data_file.close();
  }
}

/* ************************************************************************* */
void FrontEndUTIAS::readLandmarkMeas(size_t dataset_id) {
  for (size_t r = 1; r<=num_robots_; r++) {
    NonlinearFactorGraph landmark_meas_graph;
    std::string landmark_meas_file_name = "../../data/Dataset" + std::to_string(dataset_id) + "/landmark_measurements_" + std::to_string(r) + ".txt";
    std::ifstream data_file(landmark_meas_file_name);
    std::string line;
    while (getline(data_file, line)) {
      std::vector<std::string> parts;
      split(parts, line, boost::is_any_of(" "));
      size_t step_id = stoi(parts[0]);
      size_t landmark_id = stoi(parts[1]);
      double range_meas = stod(parts[2]);
      double bearing_meas = stod(parts[3]);

      auto factor = BearingRangeFactor2D(RobotPoseKey(r, step_id), LandmarkKey(landmark_id), bearing_meas, range_meas, landmark_meas_noise_model_);
      measurements_vec_[r-1].landmark_meas_graph.add(factor);
      measurements_vec_[r-1].landmark_measures.push_back(std::make_pair(bearing_meas, range_meas));
    }
    data_file.close();
    measurements_vec_[r-1].vi_landmark_meas = VariableIndex(measurements_vec_[r-1].landmark_meas_graph);
  }
}

/* ************************************************************************* */
void FrontEndUTIAS::readRobotMeas(size_t dataset_id) {
  robot_meas_graph_ = NonlinearFactorGraph();
  for (size_t r = 1; r<=num_robots_; r++) {
    NonlinearFactorGraph robot_meas_graph;
    std::string robot_meas_file_name = "../../data/Dataset" + std::to_string(dataset_id) + "/robot_measurements_" + std::to_string(r) + ".txt";
    std::ifstream data_file(robot_meas_file_name);
    std::string line;
    while (getline(data_file, line)) {
      std::vector<std::string> parts;
      split(parts, line, boost::is_any_of(" "));
      size_t step_id = stoi(parts[0]);
      size_t other_robot_id = stoi(parts[1]);
      double range_meas = stod(parts[2]);
      double bearing_meas = stod(parts[3]);

      auto factor = PoseBearingRangeFactor2D(RobotPoseKey(r, step_id), RobotPoseKey(other_robot_id, step_id), bearing_meas, range_meas, robot_meas_noise_model_);
      robot_meas_graph_.add(factor);
      robot_measures_.push_back(std::make_pair(bearing_meas, range_meas));
    }
    data_file.close();
  }
  vi_robot_meas_ = VariableIndex(robot_meas_graph_);
}


/* ************************************************************************* */
void FrontEndUTIAS::loadDataSet(size_t dataset_id) {
  measurements_vec_ = std::vector<Measurements>(num_robots_, Measurements());
  readLandmarksGT(dataset_id);
  readPosesGT(dataset_id);
  readOdometry(dataset_id);
  readLandmarkMeas(dataset_id);
  readRobotMeas(dataset_id);
}


/* ************************************************************************* */
FrontEnd::InitCondition FrontEndUTIAS::initialCondition() const {
  InitCondition init_condition;
  Key origin_key = LabeledSymbol('O', 0, 0);
  Pose2 origin_pose = Pose2(0, 0, 0);
  init_condition.graph.add(PriorFactor<Pose2>(origin_key, origin_pose, prior_noise_model_));
  init_condition.values.insert(origin_key, origin_pose);
  KeyVector keys_order;
  keys_order.push_back(origin_key);
  for (size_t r=1; r<=num_robots_; r++) {
    Pose2 groundtruth_pose = gt_values_.at<Pose2>(RobotPoseKey(r, 0));
    init_condition.graph.add(BetweenFactor<Pose2>(origin_key, RobotPoseKey(r, 0), groundtruth_pose, prior_noise_model_));
    init_condition.values.insert(RobotPoseKey(r, 0), groundtruth_pose);
    if (r!=num_robots_-1) {
      KeySet root_keys;
      root_keys.insert(RobotPoseKey(r, 0));
      keys_order.push_back(RobotPoseKey(r, 0));
      init_condition.other_root_keys_map[r] = root_keys;
    }
  }
  init_condition.order = Ordering(keys_order);
  init_condition.root_id = num_robots_-1;
  return init_condition;
}


/* ************************************************************************* */
FrontEnd::InitCondition FrontEndUTIAS::initialConditionNoAnchor() const {
  InitCondition init_condition;

  for (size_t r=1; r<=num_robots_; r++) {
    Pose2 groundtruth_pose = gt_values_.at<Pose2>(RobotPoseKey(r, 0));
    init_condition.graph.add(PriorFactor<Pose2>(RobotPoseKey(r, 0), groundtruth_pose, prior_noise_model_));
    init_condition.values.insert(RobotPoseKey(r, 0), groundtruth_pose);
  }

  return init_condition;
}

/* ************************************************************************* */
Point2 FrontEndUTIAS::calcFromBRMeasure(const Pose2& pose, const BRMeasure& br_measure) const {
  double bearing_measure = br_measure.first;
  double range_measure = br_measure.second;
  double x = range_measure * cos(bearing_measure);
  double y = range_measure * sin(bearing_measure);
  Point2 rel_point(x, y);
  return pose.transformFrom(rel_point);
}


/* ************************************************************************* */
std::vector<FrontEnd::StepData> FrontEndUTIAS::step(const Values& old_values, const size_t first_step, const size_t num_steps) const {

  std::vector<FrontEnd::StepData> step_data_vec;

  Values all_new_values;

  for (size_t r=1; r<=num_robots_; r++) {

    StepData step_data;
    step_data.root_id = r;

    // add the odometry factors
    // std::cout << "add the odometry factors\n";
    for (size_t i = 0; i<num_steps; i++) {
      step_data.graph.push_back(measurements_vec_[r-1].odom_graph.at(first_step+i-1));
    }

    // add the pose values
    // std::cout << "add the pose values\n";
    Pose2 current_pose = old_values.at<Pose2>(RobotPoseKey(r, first_step-1));
    std::vector<Pose2> new_poses;
    Values new_pose_values;
    for (size_t i=0; i<num_steps; i++) {
      size_t pose_id = first_step + i;
      Pose2 rel_pose = measurements_vec_[r-1].odom_poses[pose_id-1];
      current_pose = current_pose.compose(rel_pose);
      new_pose_values.insert(RobotPoseKey(r, pose_id), current_pose);
    }
    step_data.values.insert(new_pose_values);
    all_new_values.insert(new_pose_values);

    // add the landmark measurement factors
    // std::cout << "add the landmark measurement factors\n";
    const VariableIndex& vi_landmark_meas = measurements_vec_[r-1].vi_landmark_meas;
    const NonlinearFactorGraph& landmark_meas_graph = measurements_vec_[r-1].landmark_meas_graph;
    KeySet landmark_keys;
    for (Key pose_key : new_pose_values.keys()) {
      if (vi_landmark_meas.find(pose_key) != vi_landmark_meas.end()) {
        auto factor_indices = vi_landmark_meas[pose_key];
        for (auto factor_index : factor_indices) {
          step_data.graph.push_back(landmark_meas_graph.at(factor_index));
          landmark_keys.insert(landmark_meas_graph.at(factor_index)->keys()[1]);
        }
      }
    }

    // add the landmark values
    // std::cout << "add the landmark values\n";
    Values new_landmark_values;
    for (Key landmark_key : landmark_keys) {
      if (!old_values.exists(landmark_key) && (!all_new_values.exists(landmark_key))) {
        
        std::vector<Point2> landmark_point_vec;
        auto factor_indices = vi_landmark_meas[landmark_key];
        
        for (auto factor_index : factor_indices) {
          Key pose_key = landmark_meas_graph.at(factor_index)->keys()[0];
          Pose2 pose;
          if (old_values.exists(pose_key)) {
            pose = old_values.at<Pose2>(pose_key);
          }
          else if (all_new_values.exists(pose_key)) {
            pose = all_new_values.at<Pose2>(pose_key);
          }
          else {
            continue;
          }
          BRMeasure measurement = measurements_vec_[r-1].landmark_measures[factor_index];
          landmark_point_vec.push_back(calcFromBRMeasure(pose, measurement));
        }

        Point2 ave_point(0, 0);
        for (Point2& point : landmark_point_vec) {
          ave_point += point;
        }
        ave_point /= (double)landmark_point_vec.size();

        new_landmark_values.insert(landmark_key, ave_point);
      }
    }
    step_data.values.insert(new_landmark_values);
    all_new_values.insert(new_landmark_values);

    // add inter robot measurements
    if (include_inter_robot_measurements_) {
      for (Key pose_key : new_pose_values.keys()) {
        if (vi_robot_meas_.find(pose_key) != vi_robot_meas_.end()) {
          auto factor_indices = vi_robot_meas_[pose_key];
          for (auto factor_index : factor_indices) {
            bool to_add = true;
            for (Key key : robot_meas_graph_.at(factor_index)->keys()) {
              LabeledSymbol symb(key);
              if (symb.label() - 97 > r) {
                to_add = false;
              }
            }
            if (to_add) {
              // std::cout << "adding inter-robot measurement\n";
              step_data.graph.push_back(robot_meas_graph_.at(factor_index));
            }
          }
        }
      }
    }
    step_data_vec.push_back(step_data);
  }
  return step_data_vec;
}


}