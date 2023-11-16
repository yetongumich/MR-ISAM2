#include "Experiment.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>

namespace gtsam {

/* ************************************************************************* */
double cAverage(const std::vector<double>& data_list) {
  double sum = 0;
  for (const double& duration : data_list) {
    sum += duration;
  }
  return sum / (double)data_list.size();
}

/* ************************************************************************* */
double cSum(const std::vector<double>& data_list) {
  double sum = 0;
  for (const double& duration : data_list) {
    sum += duration;
  }
  return sum;
}

/* ************************************************************************* */
double cAverage(const std::vector<size_t>& data_list) {
  double sum = 0;
  for (const double& duration : data_list) {
    sum += duration;
  }
  return sum / (double)data_list.size();
}

/* ************************************************************************* */
size_t cASum(const std::vector<size_t>& data_list) {
  size_t sum = 0;
  for (const double& duration : data_list) {
    sum += duration;
  }
  return sum;
}

/* ************************************************************************* */
std::pair<double, double> evaluateError(const Values& gt_values,
                                        const Values& values) {
  std::vector<double> rot_error_vec, trans_error_vec;
  for (Key key : values.keys()) {
    LabeledSymbol symbol(key);
    if (symbol.chr() == 'X') {
      Pose2 gt_pose = gt_values.at<Pose2>(key);
      Pose2 pose = values.at<Pose2>(key);
      double x_diff = pose.x() - gt_pose.x();
      double y_diff = pose.y() - gt_pose.y();
      double theta_diff =
          fmod(pose.theta() - gt_pose.theta() + 2 * M_PI, 2 * M_PI);
      if (theta_diff > M_PI) {
        theta_diff = 2 * M_PI - theta_diff;
      }
      rot_error_vec.push_back(theta_diff);
      trans_error_vec.push_back(sqrt(pow(x_diff, 2) + pow(y_diff, 2)));
    }
  }
  return std::make_pair(cAverage(trans_error_vec),
                        cAverage(rot_error_vec));
}

/* ************************************************************************* */
void storeStepResult(const Experiment::ExpSetting& exp_setting,
                     const std::string exp_name, size_t last_step,
                     const Values& current_values,
                     std::vector<FrontEnd::StepData>& step_data_vec) {
  KeySet new_keys;
  for (const auto& step_data : step_data_vec) {
    for (Key key : step_data.values.keys()) {
      new_keys.insert(key);
    }
  }

  std::string out_file_name = exp_setting.folderPath() + exp_name + "_" +
                              std::to_string(last_step) + ".txt";
  std::ofstream out_file(out_file_name, std::ofstream::out);

  for (Key key : current_values.keys()) {
    LabeledSymbol symbol(key);
    size_t step = symbol.index();
    int is_new = new_keys.exists(key) ? 1 : 0;
    if (symbol.chr() == 'X') {
      size_t robot_id = symbol.label() - 97;
      Pose2 pose = current_values.at<Pose2>(key);
      out_file << "POSE " << robot_id << " " << step << " " << pose.x() << " "
               << pose.y() << " " << pose.theta() << " " << is_new << std::endl;
    } else if (symbol.chr() == 'L') {
      Point2 point = current_values.at<Point2>(key);
      out_file << "LANDMARK " << point(0) << " " << point(1) << " " << is_new
               << std::endl;
    }
  }

  out_file.close();
}

std::pair<size_t, double> getCliqueSize(boost::shared_ptr<ISAM2Copy>& isam2) {
  std::set<boost::shared_ptr<ISAM2CopyClique>> isam2_cliques;
  auto nodes = isam2->nodes();
  for (const auto& it : nodes) {
    isam2_cliques.insert(it.second);
  }
  size_t max_clique_size = 0;
  size_t total_clique_size = 0;
  for (auto& isam2_clique : isam2_cliques) {
    size_t clique_size = isam2_clique->conditional()->frontals().size() +
                         isam2_clique->conditional()->parents().size();
    total_clique_size += clique_size;
    max_clique_size = std::max(max_clique_size, clique_size);
  }
  double avg_clique_size =
      (double)total_clique_size / (double)isam2_cliques.size();
  return std::make_pair(max_clique_size, avg_clique_size);
}

/* ************************************************************************* */
Experiment::IterationResult updateISAM2(
    boost::shared_ptr<ISAM2Copy>& isam2,
    const std::vector<FrontEnd::StepData>& step_data_vec,
    const Values& gt_values,
    const Experiment::ExpSetting& setting,
    size_t last_step) {

  Experiment::IterationResult iter_result;
  for (auto&& step_data : step_data_vec) {
    clock_t time_s = clock();
    ISAM2CopyResult result = isam2->update(step_data.graph, step_data.values);
    auto duration = clock() - time_s;
    iter_result.durations.push_back(duration);
    iter_result.reelim_vars.push_back(result.variablesReeliminated);
    iter_result.relin_factors.push_back(result.factorsRecalculated);
    iter_result.recal_cliques.push_back(result.cliques_in_top);
  }
  boost::tie(iter_result.max_clique_size, iter_result.avg_clique_size) = getCliqueSize(isam2);

  iter_result.current_values = isam2->calculateEstimate();
  Values new_values;
  for (auto&& step_data : step_data_vec) {
    for (Key key : step_data.values.keys()) {
      new_values.insert(key, iter_result.current_values.at(key));
    }
  }
  boost::tie(iter_result.trans_error, iter_result.rot_error) = evaluateError(gt_values, new_values);
  return iter_result;
}

/* ************************************************************************* */
Experiment::IterationResult updateMRISAM2(
    boost::shared_ptr<MRISAM2>& mr_isam2,
    const std::vector<FrontEnd::StepData>& step_data_vec,
    const Values& gt_values,
    const Experiment::ExpSetting& setting,
    size_t last_step) {
  
  Experiment::IterationResult iter_result;

  std::vector<double> total_times(7, 0);
  for (auto&& step_data : step_data_vec) {
    clock_t time_s = clock();
    MRISAM2Result result = mr_isam2->updateRoot(
        step_data.root_id, step_data.graph, step_data.values, true);

    mr_isam2->saveGraph(
        setting.folderPath() + "mrbt" + std::to_string(last_step) + "_" + std::to_string(step_data.root_id) + ".dot",
        result.top_cliques);

    auto duration = clock() - time_s;
    iter_result.durations.push_back(duration);
    iter_result.reelim_vars.push_back(result.variables_reeliminated.size());
    iter_result.relin_factors.push_back(result.top_factor_indices.size());
    iter_result.recal_cliques.push_back(result.new_top_clique_size);

    iter_result.prop_marginals.push_back(result.propagated_marginal);
    iter_result.prop_deltas.push_back(result.propagated_delta);

    iter_result.process_durations = std::vector<std::vector<double>>(7, std::vector<double>());
    for (size_t i = 0; i < result.durations.size(); i++) {
      iter_result.process_durations[i].push_back(result.durations[i]);
    }
  }

  MRISAM2::CliqueVector mrisam2_cliques = mr_isam2->allCliques();
  iter_result.max_clique_size = 0;
  size_t total_clique_size = 0;
  for (auto& clique : mrisam2_cliques) {
    size_t clique_size = clique->size();
    total_clique_size += clique_size;
    iter_result.max_clique_size = std::max(iter_result.max_clique_size, clique_size);
  }
  iter_result.avg_clique_size =
      (double)total_clique_size / (double)mrisam2_cliques.size();

  iter_result.current_values = mr_isam2->calculateBestEstimate();
  Values new_values;
  for (auto&& step_data : step_data_vec) {
    for (Key key : step_data.values.keys()) {
      new_values.insert(key, iter_result.current_values.at(key));
    }
  }
  boost::tie(iter_result.trans_error, iter_result.rot_error) = evaluateError(gt_values, new_values);
  return iter_result;
}

/* ************************************************************************* */
Experiment::ExpResult Experiment::runExperiment(const ExpSetting& setting) {
  // initialization
  ExpResult exp_result;
  setting.createFolder();
  boost::shared_ptr<FrontEnd> front_end;
  if (setting.is_ct) {
    front_end = boost::make_shared<FrontEndCityTrees>(setting.num_robots, setting.use_gt);
  }
  else {
    front_end = boost::make_shared<FrontEndUTIAS>(setting.dataset_id, setting.include_inter_robot_measurements);
  }
  
  size_t max_steps =
      setting.max_steps == 0 ? front_end->maxSteps() : setting.max_steps;
  if (max_steps>front_end->maxSteps()) {
    max_steps = front_end->maxSteps();
  }
  auto init_condition = front_end->initialCondition();
  boost::shared_ptr<ISAM2Copy> isam2;
  boost::shared_ptr<MRISAM2> mr_isam2;
  std::ofstream out_file_isam2, out_file_mrisam2;
  if (setting.run_isam2) {
    std::string out_file_name = setting.folderPath() + "isam2_summary.txt";
    out_file_isam2 = std::ofstream(out_file_name, std::ofstream::out);
    isam2 = boost::make_shared<ISAM2Copy>(setting.isam2_params);
    isam2->update(init_condition.graph, init_condition.values);
  }
  if (setting.run_mrisam2) {
    std::string out_file_name = setting.folderPath() + "mrisam2_summary.txt";
    out_file_mrisam2 = std::ofstream(out_file_name, std::ofstream::out);
    mr_isam2 = boost::make_shared<MRISAM2>(
        init_condition.graph, init_condition.values, init_condition.order,
        init_condition.root_id, init_condition.other_root_keys_map,
        setting.mrisam2_params);
  }
  double accum_time_isam2 = 0;
  double accum_time_mrisam2 = 0;

  // run each iteration
  for (size_t first_step = 1; first_step <= max_steps;
       first_step += setting.steps_per_iter) {
    size_t last_step =
        std::min(first_step + setting.steps_per_iter - 1, max_steps);
    size_t num_steps = last_step - first_step + 1;
    std::cout << "from step " << first_step << " to " << last_step << "\n";

    /* ******************************* isam2 ******************************* */
    if (setting.run_isam2) {
      // run frontend and isam2
      Values old_values = isam2->calculateEstimate();
      auto step_data_vec = front_end->step(old_values, first_step, num_steps);
      IterationResult iter_result = updateISAM2(isam2, step_data_vec, front_end->gtValues(), setting, last_step);

      // store iteration results
      std::cout << "\tisam2:\t\tduration: " << cSum(iter_result.durations) << "\n";
      accum_time_isam2 += cSum(iter_result.durations);
      out_file_isam2 << first_step << " " << last_step 
                     << " " << cSum(iter_result.durations)
                     << " " << accum_time_isam2
                     << " " << cAverage(iter_result.reelim_vars) 
                     << " " << cAverage(iter_result.relin_factors) 
                     << " " << cAverage(iter_result.recal_cliques)
                     << " " << iter_result.max_clique_size 
                     << " " << iter_result.avg_clique_size 
                     << " " << iter_result.trans_error 
                     << " " << iter_result.rot_error 
                     << std::endl;

      if (setting.store_result_per_update) {
        storeStepResult(setting, "isam2", last_step, iter_result.current_values,
                        step_data_vec);
      }
      if (setting.store_bayes_tree) {
        isam2->saveGraph(
            setting.folderPath() + "bt" + std::to_string(last_step) + ".dot",
            MultiRobotKeyFormatter);
      }
    }

    /* ****************************** mr-isam2 ***************************** */
    if (setting.run_mrisam2) {
      // run frontend and mrisam2
      Values old_values = mr_isam2->calculateBestEstimate();
      auto step_data_vec = front_end->step(old_values, first_step, num_steps);
      IterationResult iter_result = updateMRISAM2(mr_isam2, step_data_vec, front_end->gtValues(), setting, last_step);

      // store iteration results
      std::cout << "\tmrisam2:\tduration: " << cSum(iter_result.durations) << "\n";
      accum_time_mrisam2 += cSum(iter_result.durations);
      out_file_mrisam2 << first_step << " " << last_step 
                       << " " << cSum(iter_result.durations)
                       << " " << accum_time_mrisam2
                       << " " << cAverage(iter_result.reelim_vars) 
                       << " " << cAverage(iter_result.relin_factors) 
                       << " " << cAverage(iter_result.recal_cliques)
                       << " " << iter_result.max_clique_size 
                       << " " << iter_result.avg_clique_size 
                       << " " << iter_result.trans_error 
                       << " " << iter_result.rot_error 
                       << " " << cAverage(iter_result.prop_marginals)
                       << " " << cAverage(iter_result.prop_deltas);
      for (const auto& process_duration : iter_result.process_durations) {
        out_file_mrisam2 << " " << cSum(process_duration);
      }
      out_file_mrisam2 << std::endl;

      if (setting.store_result_per_update) {
        storeStepResult(setting, "mrisam2", last_step, iter_result.current_values,
                        step_data_vec);
      }
      if (setting.store_mrbt) {
        mr_isam2->saveGraph(
            setting.folderPath() + "mrbt" + std::to_string(last_step) + ".dot");
      }
    }
  }
  if (setting.run_isam2) {
    out_file_isam2.close();
  }
  if (setting.run_mrisam2) {
    out_file_mrisam2.close();
  }

  return exp_result;
}

}  // namespace gtsam