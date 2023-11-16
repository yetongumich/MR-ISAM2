#include "Experiment.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/LabeledSymbol.h>

#include <vector>
#include <fstream>
#include <string>
#include <time.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;


#include <fstream>
#include <string>
#include <time.h>


void compute_gt () {
  size_t num_robots = 5;
  auto front_end = FrontEndCityTrees(num_robots, true);
  size_t steps_per_iter = 10;
  size_t max_steps = front_end.maxSteps();

  ISAM2CopyParams isam2_params;
  ISAM2Copy isam2(isam2_params);

  auto init_condition = front_end.initialCondition();
  isam2.update(init_condition.graph, init_condition.values);

  for (size_t first_step=1; first_step<=max_steps; first_step+=steps_per_iter) {
    size_t num_steps = first_step + steps_per_iter-1 <= max_steps ? steps_per_iter : max_steps - first_step+1;
    std::cout << "from step " << first_step << " to " << first_step + num_steps - 1 << "\n";
    auto step_data_vec = front_end.step(isam2.calculateBestEstimate(), first_step, num_steps);
    for (auto&& step_data : step_data_vec) {
      isam2.update(step_data.graph, step_data.values);
    }
  }

  Values isam2_result = isam2.calculateBestEstimate();
  
  NonlinearFactorGraph graph;
  graph.add(init_condition.graph);
  graph.add(front_end.odomGraph());
  graph.add(front_end.landmarkMeasGraph());
  GaussNewtonParams params;
  params.setRelativeErrorTol(1e-10);
  params.setAbsoluteErrorTol(1e-10);
  GaussNewtonOptimizer optimizer(graph, isam2_result, params);
  
  Values result = optimizer.optimize();
  result.print("", MultiRobotKeyFormatter);

  // output to gt file
  std::string gt_file_name = "../data/cityTrees10000_gtvalue.txt";
  std::ofstream gt_file(gt_file_name, std::ofstream::out);
  for (Key key : result.keys()) {
    LabeledSymbol symbol(key);
    // std::cout << "symbol: " << symbol.label() << "\n";
    if (symbol.chr() == 'X') {
      size_t pose_id = symbol.index();
      Pose2 pose = result.at<Pose2>(key);
      gt_file << "POSE " << pose_id << " " << pose.x() << " " << pose.y() << " " << pose.theta() << std::endl;
    }
  }
  for (Key key : result.keys()) {
    LabeledSymbol symbol(key);
    if (symbol.chr() == 'L') {
      size_t landmark_id = symbol.index();
      Point2 point = result.at<Point2>(key);
      gt_file << "LANDMARK " << landmark_id << " " << point(0) << " " << point(1) << std::endl;
    }
  }
  gt_file.close();
}


void createMRBTwithoutAnchor() {
  double relin_threshold = 0.2;
  double delta_threshold = 0.05;
  double marginal_threshold = 10;

  FrontEndUTIAS front_end(1, true);
  auto init_condition = front_end.initialConditionNoAnchor();

  size_t last_step = 5;
  size_t final_step = 100;

  auto step_data_vec = front_end.step(init_condition.values, 1, last_step);

  NonlinearFactorGraph graph = init_condition.graph;
  Values values = init_condition.values;
  for (auto&& step_data : step_data_vec) {
    graph.add(step_data.graph);
    values.insert(step_data.values);
  }

  MRISAM2::RootID root_id = 1;
  MRISAM2::RootKeySetMap other_root_keys_map;
  Key root_key = FrontEnd::RobotPoseKey(1, last_step);
  for (size_t r=2; r<=5; r++) {
    KeySet root_keys;
    root_keys.insert(FrontEnd::RobotPoseKey(r, last_step));
    other_root_keys_map[r] = root_keys;
  }

  VariableIndex vi(graph);
  FastMap<Key, int> constraint_groups;
  int group = 1;
  constraint_groups.insert(std::make_pair(root_key, group));
  Ordering order =
      Ordering::ColamdConstrained(vi, constraint_groups);
  MRISAM2Params mrisam2_params;
  mrisam2_params.marginal_update_threshold = marginal_threshold;
  mrisam2_params.delta_update_threshold = delta_threshold;
  mrisam2_params.relinearization_threshold = relin_threshold;

  // create MRiSAM2
  MRISAM2 mr_isam2(graph, values, order,
        root_id, other_root_keys_map,
        mrisam2_params);

  ISAM2CopyParams isam2_params(ISAM2CopyGaussNewtonParams(delta_threshold), relin_threshold, 1);
  isam2_params.enablePartialRelinearizationCheck = true;
  ISAM2Copy isam2(isam2_params);
  isam2.update(graph, values);
  
  mr_isam2.saveGraph("../../results/mrbt_" + std::to_string(last_step) + ".dot");
  for (size_t i=last_step+1; i<=final_step; i++) {
    auto step_data_vec = front_end.step(mr_isam2.calculateBestEstimate(), i, 1);
    for (auto&& step_data : step_data_vec) {
      MRISAM2Result result = mr_isam2.updateRoot(
          step_data.root_id, step_data.graph, step_data.values, false);
      ISAM2CopyResult result_isam2 = isam2.update(step_data.graph, step_data.values);
      mr_isam2.saveGraph("../../results/mrbt_" + std::to_string(i) + "_" + std::to_string(step_data.root_id) + ".dot", result.top_cliques);
      isam2.saveGraphNew("../../results/bt_" + std::to_string(i) + "_" + std::to_string(step_data.root_id) + ".dot", result_isam2.reelim_keyset);
    }
    
    mr_isam2.saveGraph("../../results/mrbt_" + std::to_string(i) + ".dot");
    isam2.saveGraphNew("../../results/bt_" + std::to_string(i) + ".dot");
  }
}


void runExpCT() {

 double relin_threshold = 0.1;
 double delta_threshold = 0.02;
 double marginal_threshold = 10;

  Experiment::ExpSetting exp_setting;
  exp_setting.use_gt = false;
  exp_setting.is_ct = true;
  exp_setting.isam2_params = ISAM2CopyParams(ISAM2CopyGaussNewtonParams(delta_threshold), relin_threshold, 1);
  exp_setting.isam2_params.enablePartialRelinearizationCheck = true;
  exp_setting.mrisam2_params = MRISAM2Params();
  exp_setting.mrisam2_params.marginal_update_threshold = marginal_threshold;
  exp_setting.mrisam2_params.delta_update_threshold = delta_threshold;
  exp_setting.mrisam2_params.relinearization_threshold = relin_threshold;
  exp_setting.mrisam2_params.marginal_change_use_gradient = false;
  exp_setting.run_isam2 = true;
  exp_setting.run_mrisam2 = true;
  exp_setting.store_result_per_update = true;
  exp_setting.store_mrbt = false;
  exp_setting.store_bayes_tree = false;
  exp_setting.max_steps = 0;


  // exp_setting.num_robots = 1;
  // exp_setting.steps_per_iter = 1;
  // Experiment::runExperiment(exp_setting);

  // exp_setting.num_robots = 2;
  // exp_setting.steps_per_iter = 10;
  // Experiment::runExperiment(exp_setting);

  // exp_setting.num_robots = 5;
  // exp_setting.steps_per_iter = 1;
  // Experiment::runExperiment(exp_setting);

  // exp_setting.num_robots = 5;
  // exp_setting.steps_per_iter = 10;
  // Experiment::runExperiment(exp_setting);

  exp_setting.num_robots = 50;
  exp_setting.steps_per_iter = 1;
  Experiment::runExperiment(exp_setting);
}


void runExpUTIAS() {
  double relin_threshold = 0.1;
  double delta_threshold = 0.02;
  double marginal_threshold = 10;

  Experiment::ExpSetting exp_setting;
  exp_setting.is_ct = false;
  exp_setting.isam2_params = ISAM2CopyParams(ISAM2CopyGaussNewtonParams(delta_threshold), relin_threshold, 1);
  exp_setting.isam2_params.enablePartialRelinearizationCheck = true;
  exp_setting.mrisam2_params = MRISAM2Params();
  exp_setting.mrisam2_params.marginal_update_threshold = marginal_threshold;
  exp_setting.mrisam2_params.delta_update_threshold = delta_threshold;
  exp_setting.mrisam2_params.relinearization_threshold = relin_threshold;
  exp_setting.mrisam2_params.marginal_change_use_gradient = false;
  exp_setting.run_isam2 = true;
  exp_setting.run_mrisam2 = true;
  exp_setting.store_result_per_update = true;
  exp_setting.store_mrbt = false;
  exp_setting.store_bayes_tree = false;
  exp_setting.max_steps = 5000;

  exp_setting.dataset_id = 1;
  exp_setting.steps_per_iter = 1;
  exp_setting.include_inter_robot_measurements = true;
  // for (size_t i = 2; i<=9; i++) {
    // exp_setting.dataset_id = i;
    Experiment::runExperiment(exp_setting);
  // }
  
}

int main() {
  // compute_gt();

  // runExpCT();
  // runExpUTIAS();
  createMRBTwithoutAnchor();

  return 0;
}