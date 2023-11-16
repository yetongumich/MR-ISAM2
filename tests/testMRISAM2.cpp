#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "mr_isam2/MRISAM2.h"
#include "mr_isam2/Utils.h"

using namespace std;
using namespace gtsam;

#define a1 LabeledSymbol('X', 'a', 1)
#define a2 LabeledSymbol('X', 'a', 2)
#define a3 LabeledSymbol('X', 'a', 3)
#define a4 LabeledSymbol('X', 'a', 4)
#define a5 LabeledSymbol('X', 'a', 5)
#define a6 LabeledSymbol('X', 'a', 6)
#define b1 LabeledSymbol('X', 'b', 1)
#define b2 LabeledSymbol('X', 'b', 2)
#define b3 LabeledSymbol('X', 'b', 3)
#define b4 LabeledSymbol('X', 'b', 4)
#define b5 LabeledSymbol('X', 'b', 5)
#define b6 LabeledSymbol('X', 'b', 6)

typedef boost::shared_ptr<MRISAM2> SharedMRISAM2;
void getOriginalGraphTree(NonlinearFactorGraph& graph, Values& values, SharedMRISAM2& mr_isam2) {
  SharedDiagonal model = noiseModel::Unit::Create(1);
  graph += PriorFactor<double>(a1, 1.0, model),   // 0
      PriorFactor<double>(b1, 1.0, model),        // 1
      BetweenFactor<double>(a1, a2, 1.0, model),  // 2
      BetweenFactor<double>(b1, b2, 1.0, model),  // 3
      BetweenFactor<double>(a2, a3, 1.0, model),  // 4
      BetweenFactor<double>(b2, b3, 1.0, model),  // 5
      BetweenFactor<double>(a3, a4, 1.0, model),  // 6
      BetweenFactor<double>(b3, b4, 1.0, model),  // 7
      BetweenFactor<double>(a4, a5, 1.0, model),  // 8
      BetweenFactor<double>(b4, b5, 1.0, model),  // 9
      BetweenFactor<double>(a2, b3, 1.0, model),  // 10
      BetweenFactor<double>(a3, b3, 0.0, model),  // 11
      BetweenFactor<double>(a3, b4, 1.0, model);  // 12

  values.insert(a1, 0.0);
  values.insert(a2, 0.0);
  values.insert(a3, 0.0);
  values.insert(a4, 0.0);
  values.insert(a5, 0.0);
  values.insert(b1, 0.0);
  values.insert(b2, 0.0);
  values.insert(b3, 0.0);
  values.insert(b4, 0.0);
  values.insert(b5, 0.0);

  Ordering order;
  order += b1, b2, a1, a2, b5, b3, b4, a3, a4, a5;

  GaussianMRBayesTree::RootKeySetMap root_keys_map;
  KeySet keys_a, keys_b;
  // keys_a.insert(a5);
  keys_b.insert(b5);
  // root_keys_map[0] = keys_a;
  root_keys_map[1] = keys_b;
  mr_isam2 = boost::make_shared<MRISAM2>(graph, values, order, 0, root_keys_map);
}

/* Following 4 tests use the same scenario:
 * Test Scenario:
 * graph:
 *  b1 - b2 - b3 - b4 - b5
 *          / |  /
 *  a1 - a2 - a3 - a4 - a5
 *
 *
 * new graph:
 *  b1 - b2 - b3 - b4 - b5 - b6
 *          / |  /
 *  a1 - a2 - a3 - a4 - a5
 *               \    /
 *                 a6
 *
 * mrbt:
 *                 [a4,a5]
 *                   ||
 *    (b4,b5)      [a3,a4]
 *       \\         //
 *         (a3,b3,b4)
 *         /        \
 *     (b2,b3)     (a2,a3,b3)
 *        |          |
 *     (b1,b2)      (a1,a2)
 *
 * new mrbt:
 *    (b5,b6)
 *      ||
 *    (b4,b5)       [a3,a5,a6]
 *        \\        //      \
 *         (a3,b3,b4)       [a3,a4,a5]
 *         /        \
 *     (b2,b3)     (a2,a3,b3)
 *        |          |
 *     (b1,b2)      (a1,a2)
 *
 *
 */

/** check member functions */
TEST(MRISAM2, Case1_member_functions) {
  // check static function
  KeySet clique_keys;
  clique_keys.insert(a1);
  clique_keys.insert(a4);
  clique_keys.insert(b2);
  clique_keys.insert(b5);
  MRISAM2::SharedClique clique = boost::make_shared<MRISAM2::Clique>(clique_keys);

  std::cout << MultiRobotKeyFormatter(MRISAM2::findMostRecentStateKey(0, clique)) << "\n";
  std::cout << MultiRobotKeyFormatter(MRISAM2::findMostRecentStateKey(1, clique)) << "\n";
  EXPECT(MRISAM2::findMostRecentStateKey(0, clique) == a4);
  EXPECT(MRISAM2::findMostRecentStateKey(1, clique) == b5);

  // construct original MRBT
  SharedDiagonal model = noiseModel::Unit::Create(1);
  NonlinearFactorGraph graph;
  Values values;
  SharedMRISAM2 mr_isam2;
  getOriginalGraphTree(graph, values, mr_isam2);
  auto name_clique_map = mr_isam2->getNameCliqueMap();
  auto name_edge_map = mr_isam2->getNameEdgeMap();

  // incremental update
  NonlinearFactorGraph new_factors_a;
  new_factors_a += PriorFactor<double>(a6, 6.0, model), BetweenFactor<double>(a5, a6, 1.0, model),
      BetweenFactor<double>(a3, a6, 3.0, model);
  Values new_values_a;
  new_values_a.insert(a6, 0.0);

  // check gatherInvolvedKeys
  KeySet expected_involved_keys;
  expected_involved_keys.insert(a3);
  expected_involved_keys.insert(a5);
  KeySet involved_keys = mr_isam2->gatherInvolvedKeys(new_factors_a);
  EXPECT(ContainerEqual<KeySet>(expected_involved_keys, involved_keys));

  // check gatherRelinearizeKeys
  mr_isam2->params().relinearization_threshold = 2.5;
  KeySet expected_relin_keys;
  expected_relin_keys.insert(a5);
  expected_relin_keys.insert(a4);
  expected_relin_keys.insert(a3);
  expected_relin_keys.insert(b5);
  expected_relin_keys.insert(b4);
  expected_relin_keys.insert(b3);
  MRISAM2::SharedClique root_clique_a = mr_isam2->roots().at(0);
  KeySet relin_keys = mr_isam2->gatherRelinearizeKeys(root_clique_a);
  EXPECT(ContainerEqual<KeySet>(expected_relin_keys, relin_keys));

  mr_isam2->params().relinearization_threshold = 3.5;
  expected_relin_keys = KeySet();
  expected_relin_keys.insert(a5);
  expected_relin_keys.insert(a4);
  relin_keys = mr_isam2->gatherRelinearizeKeys(root_clique_a);
  EXPECT(ContainerEqual<KeySet>(expected_relin_keys, relin_keys));

  // check markCliques
  relin_keys = KeySet();
  involved_keys = KeySet();
  involved_keys.insert(a3);
  involved_keys.insert(a5);
  relin_keys.insert(a3);
  MRISAM2::CliqueSet expected_top_cliques;
  expected_top_cliques.insert(name_clique_map.at("Xa4 Xa5"));
  expected_top_cliques.insert(name_clique_map.at("Xa3 Xa4"));
  expected_top_cliques.insert(name_clique_map.at("Xa3 Xb3 Xb4"));
  expected_top_cliques.insert(name_clique_map.at("Xa2 Xa3 Xb3"));
  MRISAM2::CliqueSet top_cliques = mr_isam2->markCliques(root_clique_a, involved_keys, relin_keys);
  EXPECT(ContainerEqual<MRISAM2::CliqueSet>(expected_top_cliques, top_cliques));

  relin_keys = KeySet();
  involved_keys = KeySet();
  relin_keys.insert(a3);
  top_cliques = mr_isam2->markCliques(root_clique_a, involved_keys, relin_keys);
  EXPECT(ContainerEqual<MRISAM2::CliqueSet>(expected_top_cliques, top_cliques));

  relin_keys = KeySet();
  involved_keys = KeySet();
  involved_keys.insert(b4);
  expected_top_cliques = MRISAM2::CliqueSet();
  expected_top_cliques.insert(name_clique_map.at("Xa4 Xa5"));
  expected_top_cliques.insert(name_clique_map.at("Xa3 Xa4"));
  expected_top_cliques.insert(name_clique_map.at("Xa3 Xb3 Xb4"));
  top_cliques = mr_isam2->markCliques(root_clique_a, involved_keys, relin_keys);
  EXPECT(ContainerEqual<MRISAM2::CliqueSet>(expected_top_cliques, top_cliques));

  // check extractTop
  top_cliques = MRISAM2::CliqueSet();
  top_cliques.insert(name_clique_map.at("Xa4 Xa5"));
  top_cliques.insert(name_clique_map.at("Xa3 Xa4"));
  top_cliques.insert(name_clique_map.at("Xa3 Xb3 Xb4"));
  KeySet top_keys, expected_top_keys;
  FactorIndices top_factor_indices, expected_top_factor_indices;
  MRISAM2::EdgeSet expected_orphan_edges_set;
  expected_top_keys.insert(a5);
  expected_top_keys.insert(a4);
  expected_top_factor_indices = FactorIndices{6, 7, 8, 11, 12};
  expected_orphan_edges_set.insert(name_edge_map.at("Xa3 Xb3 Xb4->Xa2 Xa3 Xb3"));
  expected_orphan_edges_set.insert(name_edge_map.at("Xa3 Xb3 Xb4->Xb2 Xb3"));
  expected_orphan_edges_set.insert(name_edge_map.at("Xa3 Xb3 Xb4->Xb4 Xb5"));
  MRISAM2::EdgeVector orphan_edges = mr_isam2->extractTop(root_clique_a, top_cliques, top_factor_indices, top_keys);
  MRISAM2::EdgeSet orphan_edges_set(orphan_edges.begin(), orphan_edges.end());
  EXPECT(ContainerEqual<MRISAM2::EdgeSet>(expected_orphan_edges_set, orphan_edges_set));
  std::set<size_t> expected_top_factor_indices_set(expected_top_factor_indices.begin(),
                                                   expected_top_factor_indices.end());
  std::set<size_t> top_factor_indices_set(top_factor_indices.begin(), top_factor_indices.end());
  EXPECT(ContainerEqual<std::set<size_t>>(expected_top_factor_indices_set, top_factor_indices_set));
  EXPECT(ContainerEqual<KeySet>(expected_top_keys, top_keys));
}

/** check incremental update tree structure, marginal, conditional and delta by
 * comparing to a MRBT created from scratch */
TEST(MRISAM2, Case1_marginal_conditional_delta) {
  // construct original MRBT
  SharedDiagonal model = noiseModel::Unit::Create(1);
  NonlinearFactorGraph graph;
  Values values;
  SharedMRISAM2 mr_isam2;
  getOriginalGraphTree(graph, values, mr_isam2);

  // incremental update
  NonlinearFactorGraph new_factors_a;
  new_factors_a += PriorFactor<double>(a6, 6.0, model), BetweenFactor<double>(a5, a6, 1.0, model),
      BetweenFactor<double>(a3, a6, 3.0, model);
  Values new_values_a;
  new_values_a.insert(a6, 0.0);
  mr_isam2->params().delta_update_threshold = 0;
  mr_isam2->params().marginal_update_threshold = 0;
  mr_isam2->updateRoot(0, new_factors_a, new_values_a, false);
  EXPECT(mr_isam2->checkMarginals());

  // marginals and conditionals by comparing to a new mrbt
  graph.push_back(new_factors_a);
  values.insert(new_values_a);
  Ordering order_1;
  order_1 += b1, b2, a1, a2, b5, b3, b4, a4, a3, a5, a6;

  GaussianMRBayesTree::RootKeySetMap root_keys_map_1;
  KeySet keys_a_1, keys_b_1;
  // keys_a_1.insert(a6);
  keys_b_1.insert(b5);
  // root_keys_map_1[0] = keys_a_1;
  root_keys_map_1[1] = keys_b_1;
  MRISAM2 expected_mr_isam2_updatea(graph, values, order_1, 0, root_keys_map_1);

  EXPECT(mr_isam2->equals(expected_mr_isam2_updatea));

  // check delta
  VectorValues expected_delta;
  expected_delta.insert(a1, (Vector(1) << 1).finished());
  expected_delta.insert(a2, (Vector(1) << 2).finished());
  expected_delta.insert(a3, (Vector(1) << 3).finished());
  expected_delta.insert(a4, (Vector(1) << 4).finished());
  expected_delta.insert(a5, (Vector(1) << 5).finished());
  expected_delta.insert(a6, (Vector(1) << 6).finished());
  expected_delta.insert(b1, (Vector(1) << 1).finished());
  expected_delta.insert(b2, (Vector(1) << 2).finished());
  expected_delta.insert(b3, (Vector(1) << 3).finished());
  expected_delta.insert(b4, (Vector(1) << 4).finished());
  expected_delta.insert(b5, (Vector(1) << 5).finished());
  EXPECT(assert_equal(expected_delta, mr_isam2->delta()));
}

/** check incremental update with relinearization*/
TEST(MRISAM2, Case1_relinearization) {
  // construct original MRBT
  NonlinearFactorGraph graph;
  Values values;
  SharedMRISAM2 mr_isam2;
  getOriginalGraphTree(graph, values, mr_isam2);

  // incremental update
  NonlinearFactorGraph new_factors_a;
  SharedDiagonal model = noiseModel::Unit::Create(1);
  new_factors_a += PriorFactor<double>(a6, 6.0, model), BetweenFactor<double>(a5, a6, 1.0, model),
      BetweenFactor<double>(a3, a6, 3.0, model);
  Values new_values_a;
  new_values_a.insert(a6, 0.0);
  mr_isam2->params().relinearization_threshold = 3.5;
  mr_isam2->params().delta_update_threshold = 0;
  mr_isam2->params().marginal_update_threshold = 0;
  mr_isam2->updateRoot(0, new_factors_a, new_values_a, true);

  // marginals and conditionals by comparing to a new mrbt
  graph.push_back(new_factors_a);
  values.insert(new_values_a);
  values.update(a5, 5.0);
  values.update(a4, 4.0);
  Ordering order_1;
  order_1 += b1, b2, a1, a2, b5, b3, b4, a4, a3, a5, a6;

  GaussianMRBayesTree::RootKeySetMap root_keys_map_1;
  KeySet keys_a_1, keys_b_1;
  // keys_a_1.insert(a6);
  keys_b_1.insert(b5);
  // root_keys_map_1[0] = keys_a_1;
  root_keys_map_1[1] = keys_b_1;
  MRISAM2 expected_mr_isam2_updatea(graph, values, order_1, 0, root_keys_map_1);

  EXPECT(mr_isam2->equals(expected_mr_isam2_updatea));
}

/** check wildfire spread on marginal and delta */
TEST(MRISAM2, Case1_wildfire) {
  // construct original MRBT
  NonlinearFactorGraph graph;
  Values values;
  SharedMRISAM2 mr_isam2;
  getOriginalGraphTree(graph, values, mr_isam2);
  mr_isam2->params().relinearization_threshold = 0;
  mr_isam2->params().delta_update_threshold = 0.4;
  mr_isam2->params().marginal_update_threshold = 0.3;
  mr_isam2->params().show_details = true;

  // incremental update
  NonlinearFactorGraph new_factors_a, new_factors_b;
  SharedDiagonal model = noiseModel::Unit::Create(1);

  new_factors_b += BetweenFactor<double>(b5, b6, 1.0, model);
  Values new_values_b;
  new_values_b.insert(b6, 0.0);
  mr_isam2->updateRoot(1, new_factors_b, new_values_b, false);

  new_factors_a += PriorFactor<double>(a6, 7.0, model), BetweenFactor<double>(a5, a6, 1.0, model),
      BetweenFactor<double>(a3, a6, 3.0, model);
  Values new_values_a;
  new_values_a.insert(a6, 0.0);
  mr_isam2->updateRoot(0, new_factors_a, new_values_a, false);

  // check delta update
  graph.push_back(new_factors_a);
  graph.push_back(new_factors_b);
  values.insert(new_values_a);
  values.insert(new_values_b);
  GaussianFactorGraph linear_graph = *graph.linearize(values);
  VectorValues expected_delta = linear_graph.optimize();
  expected_delta[a1] = (Vector(1) << 1).finished();
  expected_delta[b1] = (Vector(1) << 1).finished();
  EXPECT(assert_equal(expected_delta, mr_isam2->delta()));

  // TODO: check marginal update
}

/* Test Scenario
 * graph:
 *  b1 - b2 - b3 - b4 - b5
 *          / |  /
 *  a1 - a2 - a3 - a4 - a5
 *
 *
 * new graph:
 *  b1 - b2 - b3 - b4 - b5
 *          / |  /
 *  a1 - a2 - a3 - a4 - a5
 *           \_      _/
 *               a6
 *
 * mrbt:
 *                 [a4,a5]
 *                   ||
 *    (b4,b5)      [a3,a4]
 *       \\         //
 *         [a3,b3,b4]
 *         /        \
 *     (b2,b3)     [a2,a3,b3]
 *        |          |
 *     (b1,b2)      (a1,a2)
 *
 * new mrbt:
 *    (b5,b6)
 *      ||
 *    (b4,b5)       [a3,a5,a6]
 *       \\         //      \
 *         (a3,b3,b4)       [a3,a4,a5]
 *         /        \
 *     (b2,b3)     (a2,a3,b3)
 *        |          |
 *     (b1,b2)      (a1,a2)
 *
 *
 */

TEST(MRISAM2, Case2_structure) {
  // construct original MRBT
  SharedDiagonal model = noiseModel::Unit::Create(1);
  NonlinearFactorGraph graph;
  Values values;
  SharedMRISAM2 mr_isam2;
  getOriginalGraphTree(graph, values, mr_isam2);

  // incremental update
  NonlinearFactorGraph new_factors_a;
  new_factors_a += PriorFactor<double>(a6, 6.0, model), BetweenFactor<double>(a5, a6, 1.0, model),
      BetweenFactor<double>(a2, a6, 4.0, model);
  Values new_values_a;
  new_values_a.insert(a6, 0.0);
  mr_isam2->params().delta_update_threshold = 0;
  mr_isam2->params().marginal_update_threshold = 0;
  mr_isam2->updateRoot(0, new_factors_a, new_values_a, false);
  mr_isam2->exportTree("../visualization/tree.json", mr_isam2->defaultLayout());

  // create a new mrbt from scratch
  std::cout << "creating new tree\n";
  graph.push_back(new_factors_a);
  values.insert(new_values_a);
  Ordering order;
  order += b1, b2, b5, b4, b3, a4, a1, a3, a2, a5, a6;

  GaussianMRBayesTree::RootKeySetMap root_keys_map;
  KeySet keys_a, keys_b;
  // keys_a.insert(a6);
  keys_b.insert(b5);
  // root_keys_map[0] = keys_a;
  root_keys_map[1] = keys_b;
  MRISAM2 expected_mr_isam2_updatea(graph, values, order, 0, root_keys_map);

  EXPECT(mr_isam2->equals(expected_mr_isam2_updatea));
}

// TEST(MRISAM2, IncrementalUpdate3) {
//   SharedDiagonal model = noiseModel::Unit::Create(1);
//   GaussianFactorGraph graph;
//   SharedMRISAM2 mr_isam2;
//   getOriginalGraphTree(graph, mr_isam2);

//   GaussianFactorGraph new_factors_a, new_factors_b;
//   new_factors_a +=
//     JacobianFactor(a6, 1.0*I_1x1, Vector1(7.0), model),
//     JacobianFactor(a5, -1.0*I_1x1, a6, 1.0*I_1x1, Vector1(1.0), model);
//   new_factors_b +=
//     JacobianFactor(b5, -1.0*I_1x1, b6, 1.0*I_1x1, Vector1(1.0), model),
//     JacobianFactor(a6, -1.0*I_1x1, b6, 1.0*I_1x1, Vector1(0.0), model);
//   mr_isam2->updateRoot(0, new_factors_a);
//   mr_isam2->updateRoot(1, new_factors_b);
//   mr_isam2->exportTree("../visualization/tree.json", mr_isam2->defaultLayout());
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}