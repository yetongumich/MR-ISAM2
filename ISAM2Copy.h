/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2Copy.h
 * @brief   Incremental update functionality (ISAM2Copy) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

// \callgraph

#pragma once

#include "ISAM2CopyClique.h"
#include "ISAM2CopyParams.h"
#include "ISAM2CopyResult.h"
#include "ISAM2CopyUpdateParams.h"

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace gtsam {

/**
 * @addtogroup ISAM2Copy
 * Implementation of the full ISAM2Copy algorithm for incremental nonlinear
 * optimization.
 *
 * The typical cycle of using this class to create an instance by providing
 * ISAM2CopyParams to the constructor, then add measurements and variables as they
 * arrive using the update() method.  At any time, calculateEstimate() may be
 * called to obtain the current estimate of all variables.
 *
 */
class GTSAM_EXPORT ISAM2Copy : public BayesTree<ISAM2CopyClique> {
 protected:
  /** The current linearization point */
  Values theta_;

  /** VariableIndex lets us look up factors by involved variable and keeps track
   * of dimensions */
  VariableIndex variableIndex_;

  /** The linear delta from the last linear solution, an update to the estimate
   * in theta
   *
   * This is \c mutable because it is a "cached" variable - it is not updated
   * until either requested with getDelta() or calculateEstimate(), or needed
   * during update() to evaluate whether to relinearize variables.
   */
  mutable VectorValues delta_;

  mutable VectorValues deltaNewton_;  // Only used when using Dogleg - stores
                                      // the Gauss-Newton update
  mutable VectorValues RgProd_;  // Only used when using Dogleg - stores R*g and
                                 // is updated incrementally

  /** A cumulative mask for the variables that were replaced and have not yet
   * been updated in the linear solution delta_, this is only used internally,
   * delta will always be updated if necessary when requested with getDelta()
   * or calculateEstimate().
   *
   * This is \c mutable because it is used internally to not update delta_
   * until it is needed.
   */
  mutable KeySet deltaReplacedMask_;  // TODO(dellaert): Make sure accessed in
                                      // the right way

  /** All original nonlinear factors are stored here to use during
   * relinearization */
  NonlinearFactorGraph nonlinearFactors_;

  /** The current linear factors, which are only updated as needed */
  mutable GaussianFactorGraph linearFactors_;

  /** The current parameters */
  ISAM2CopyParams params_;

  /** The current Dogleg Delta (trust region radius) */
  mutable boost::optional<double> doglegDelta_;

  /** Set of variables that are involved with linear factors from marginalized
   * variables and thus cannot have their linearization points changed. */
  KeySet fixedVariables_;

  int update_count_;  ///< Counter incremented every update(), used to determine
                      ///< periodic relinearization

 public:
  using This = ISAM2Copy;                       ///< This class
  using Base = BayesTree<ISAM2CopyClique>;      ///< The BayesTree base class
  using Clique = Base::Clique;              ///< A clique
  using sharedClique = Base::sharedClique;  ///< Shared pointer to a clique
  using Cliques = Base::Cliques;            ///< List of Cliques

  /** Create an empty ISAM2Copy instance */
  explicit ISAM2Copy(const ISAM2CopyParams& params);

  /** Create an empty ISAM2Copy instance using the default set of parameters (see
   * ISAM2CopyParams) */
  ISAM2Copy();

  /** default virtual destructor */
  virtual ~ISAM2Copy() {}

  /** Compare equality */
  virtual bool equals(const ISAM2Copy& other, double tol = 1e-9) const;

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Optionally, this function remove existing factors from the system to enable
   * behaviors such as swapping existing factors with new ones.
   *
   * Add new measurements, and optionally new variables, to the current system.
   * This runs a full step of the ISAM2Copy algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param removeFactorIndices Indices of factors to remove from system
   * @param force_relinearize Relinearize any variables whose delta magnitude is
   * sufficiently large (Params::relinearizeThreshold), regardless of the
   * relinearization interval (Params::relinearizeSkip).
   * @param constrainedKeys is an optional map of keys to group labels, such
   * that a variable can be constrained to a particular grouping in the
   * BayesTree
   * @param noRelinKeys is an optional set of nonlinear keys that iSAM2 will
   * hold at a constant linearization point, regardless of the size of the
   * linear delta
   * @param extraReelimKeys is an optional set of nonlinear keys that iSAM2 will
   * re-eliminate, regardless of the size of the linear delta. This allows the
   * provided keys to be reordered.
   * @return An ISAM2CopyResult struct containing information about the update
   */
  virtual ISAM2CopyResult update(
      const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      const Values& newTheta = Values(),
      const FactorIndices& removeFactorIndices = FactorIndices(),
      const boost::optional<FastMap<Key, int> >& constrainedKeys = boost::none,
      const boost::optional<FastList<Key> >& noRelinKeys = boost::none,
      const boost::optional<FastList<Key> >& extraReelimKeys = boost::none,
      bool force_relinearize = false);

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Alternative signature of update() (see its documentation above), with all
   * additional parameters in one structure. This form makes easier to keep
   * future API/ABI compatibility if parameters change.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param updateParams Additional parameters to control relinearization,
   * constrained keys, etc.
   * @return An ISAM2CopyResult struct containing information about the update
   * @note No default parameters to avoid ambiguous call errors.
   */
  virtual ISAM2CopyResult update(const NonlinearFactorGraph& newFactors,
                             const Values& newTheta,
                             const ISAM2CopyUpdateParams& updateParams);

  /** Marginalize out variables listed in leafKeys.  These keys must be leaves
   * in the BayesTree.  Throws MarginalizeNonleafException if non-leaves are
   * requested to be marginalized.  Marginalization leaves a linear
   * approximation of the marginal in the system, and the linearization points
   * of any variables involved in this linear marginal become fixed.  The set
   * fixed variables will include any key involved with the marginalized
   * variables in the original factors, and possibly additional ones due to
   * fill-in.
   *
   * If provided, 'marginalFactorsIndices' will be augmented with the factor
   * graph indices of the marginal factors added during the 'marginalizeLeaves'
   * call
   *
   * If provided, 'deletedFactorsIndices' will be augmented with the factor
   * graph indices of any factor that was removed during the 'marginalizeLeaves'
   * call
   */
  void marginalizeLeaves(
      const FastList<Key>& leafKeys,
      boost::optional<FactorIndices&> marginalFactorsIndices = boost::none,
      boost::optional<FactorIndices&> deletedFactorsIndices = boost::none);

  /// Access the current linearization point
  const Values& getLinearizationPoint() const { return theta_; }

  /// Check whether variable with given key exists in linearization point
  bool valueExists(Key key) const { return theta_.exists(key); }

  /** Compute an estimate from the incomplete linear delta computed during the
   * last update. This delta is incomplete because it was not updated below
   * wildfire_threshold.  If only a single variable is needed, it is faster to
   * call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const;

  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template <class VALUE>
  VALUE calculateEstimate(Key key) const {
    const Vector& delta = getDelta()[key];
    return traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
  }

  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * This is a non-templated version that returns a Value base class for use
   * with the MATLAB wrapper.
   * @param key
   * @return
   */
  const Value& calculateEstimate(Key key) const;

  /** Return marginal on any variable as a covariance matrix */
  Matrix marginalCovariance(Key key) const;

  /// @name Public members for non-typical usage
  /// @{

  /** Compute an estimate using a complete delta computed by a full
   * back-substitution.
   */
  Values calculateBestEstimate() const;

  /** Access the current delta, computed during the last call to update */
  const VectorValues& getDelta() const;

  /** Compute the linear error */
  double error(const VectorValues& x) const;

  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph& getFactorsUnsafe() const {
    return nonlinearFactors_;
  }

  /** Access the nonlinear variable index */
  const VariableIndex& getVariableIndex() const { return variableIndex_; }

  /** Access the nonlinear variable index */
  const KeySet& getFixedVariables() const { return fixedVariables_; }

  const ISAM2CopyParams& params() const { return params_; }

  /** prints out clique statistics */
  void printStats() const { getCliqueData().getStats().print(); }

  /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert
   * \Sigma^{-1} R x - d \right\Vert^2 \f$, centered around zero. The gradient
   * about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&,
   * const VectorValues&).
   *
   * @return A VectorValues storing the gradient.
   */
  VectorValues gradientAtZero() const;

  void saveGraphNew(const std::string &s, const KeySet &marked_keys=KeySet(), const KeyFormatter& keyFormatter=MultiRobotKeyFormatter) const;

  void saveGraphByClique(std::ostream &s, const KeySet &marked_keys, sharedClique clique, const KeyFormatter& indexFormatter, int parentnum=0) const;
  /// @}

 protected:
  /// Remove marked top and either recalculate in batch or incrementally.
  void recalculate(const ISAM2CopyUpdateParams& updateParams,
                   const KeySet& relinKeys, ISAM2CopyResult* result);

  // Do a batch step - reorder and relinearize all variables
  void recalculateBatch(const ISAM2CopyUpdateParams& updateParams,
                        KeySet* affectedKeysSet, ISAM2CopyResult* result);

  // retrieve all factors that ONLY contain the affected variables
  // (note that the remaining stuff is summarized in the cached factors)
  GaussianFactorGraph relinearizeAffectedFactors(
      const ISAM2CopyUpdateParams& updateParams, const FastList<Key>& affectedKeys,
      const KeySet& relinKeys);

  void recalculateIncremental(const ISAM2CopyUpdateParams& updateParams,
                              const KeySet& relinKeys,
                              const FastList<Key>& affectedKeys,
                              KeySet* affectedKeysSet, Cliques* orphans,
                              ISAM2CopyResult* result);

  /**
   * Add new variables to the ISAM2Copy system.
   * @param newTheta Initial values for new variables
   * @param variableStatus optional detailed result structure
   */
  void addVariables(const Values& newTheta,
                    ISAM2CopyResult::DetailedResults* detail = 0);

  /**
   * Remove variables from the ISAM2Copy system.
   */
  void removeVariables(const KeySet& unusedKeys);

  void updateDelta(bool forceFullSolve = false) const;
};  // ISAM2Copy

/// traits
template <>
struct traits<ISAM2Copy> : public Testable<ISAM2Copy> {};

}  // namespace gtsam
