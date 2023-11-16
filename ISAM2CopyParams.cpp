/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2CopyParams.cpp
 * @brief   Parameters for iSAM 2.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#include "ISAM2CopyParams.h"

#include <boost/algorithm/string.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string ISAM2CopyDoglegParams::adaptationModeTranslator(
    const DoglegOptimizerImpl::TrustRegionAdaptationMode& adaptationMode)
    const {
  string s;
  switch (adaptationMode) {
    case DoglegOptimizerImpl::SEARCH_EACH_ITERATION:
      s = "SEARCH_EACH_ITERATION";
      break;
    case DoglegOptimizerImpl::ONE_STEP_PER_ITERATION:
      s = "ONE_STEP_PER_ITERATION";
      break;
    default:
      s = "UNDEFINED";
      break;
  }
  return s;
}

/* ************************************************************************* */
DoglegOptimizerImpl::TrustRegionAdaptationMode
ISAM2CopyDoglegParams::adaptationModeTranslator(
    const string& adaptationMode) const {
  string s = adaptationMode;
  boost::algorithm::to_upper(s);
  if (s == "SEARCH_EACH_ITERATION")
    return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
  if (s == "ONE_STEP_PER_ITERATION")
    return DoglegOptimizerImpl::ONE_STEP_PER_ITERATION;

  /* default is SEARCH_EACH_ITERATION */
  return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
}

/* ************************************************************************* */
ISAM2CopyParams::Factorization ISAM2CopyParams::factorizationTranslator(
    const string& str) {
  string s = str;
  boost::algorithm::to_upper(s);
  if (s == "QR") return ISAM2CopyParams::QR;
  if (s == "CHOLESKY") return ISAM2CopyParams::CHOLESKY;

  /* default is CHOLESKY */
  return ISAM2CopyParams::CHOLESKY;
}

/* ************************************************************************* */
string ISAM2CopyParams::factorizationTranslator(
    const ISAM2CopyParams::Factorization& value) {
  string s;
  switch (value) {
    case ISAM2CopyParams::QR:
      s = "QR";
      break;
    case ISAM2CopyParams::CHOLESKY:
      s = "CHOLESKY";
      break;
    default:
      s = "UNDEFINED";
      break;
  }
  return s;
}

}  // namespace gtsam
