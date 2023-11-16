#pragma once

#include "ISAM2Copy.h"
#include "FrontEnd.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <filesystem>

namespace fs = std::__fs::filesystem;

namespace gtsam {

class Experiment {
 public:
  struct ExpSetting {
    bool is_ct;
    bool use_gt;            // only for CityTrees
    size_t num_robots;      // only for CityTrees
    size_t dataset_id;      // only for UTIAS
    bool include_inter_robot_measurements; // only for UTIAS
    size_t steps_per_iter;
    size_t max_steps;
    ISAM2CopyParams isam2_params;
    MRISAM2Params mrisam2_params;
    bool store_result_per_update;
    bool run_isam2;
    bool run_mrisam2;
    bool store_bayes_tree = false;
    bool store_mrbt = false;
    std::string folderPath() const {
      if (is_ct) {
        std::string folder_path = "../../results_CT_";
        folder_path += std::to_string(num_robots) + "_" + std::to_string(steps_per_iter) + "/";
        return folder_path;
      }
      else {
        std::string folder_path = "../../results_UTIAS_";
        folder_path += std::to_string(dataset_id) + "/";
        return folder_path;
      }
    }

    void createFolder() const {
      std::string folder_path = folderPath();
      fs::path fs_folder_path(folder_path);
      if (!fs::exists(fs_folder_path)) {
        fs::create_directory(folder_path);
      }
    }

  };

  struct IterationResult {
    std::vector<double> durations;
    std::vector<double> reelim_vars;
    std::vector<double> relin_factors;
    std::vector<double> recal_cliques;
    std::vector<size_t> prop_marginals;
    std::vector<size_t> prop_deltas;
    std::vector<std::vector<double>> process_durations;

    size_t max_clique_size;
    double avg_clique_size;
    double trans_error;
    double rot_error;
    Values current_values;
  };

  struct ExpResult {
    std::vector<double> time_list_isam2;
    std::vector<double> time_list_mrisam2;
    double trans_accuracy;
    double rot_accuracy;
  };

  static ExpResult runExperiment(const ExpSetting& exp_setting);

};


}