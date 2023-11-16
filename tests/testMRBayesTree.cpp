#include "mr_isam2/MRBayesTree.h"
#include "mr_isam2/Utils.h"

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>

using namespace std;
using namespace gtsam;

/** In this test, member functions of clique and edge are tested 
 * 
 * Test Scenario: 
 * c1 <-> c2 -> c3
*/

TEST(MRBayesTree, clique_edge) {
  KeySet keys1, keys2, keys3;
  keys1.insert(0); keys1.insert(1);
  keys2.insert(1); keys2.insert(2);
  keys3.insert(2); keys3.insert(3);
  auto c1 = boost::make_shared<GaussianMRBayesTree::Clique>(keys1);
  auto c2 = boost::make_shared<GaussianMRBayesTree::Clique>(keys2);
  auto c3 = boost::make_shared<GaussianMRBayesTree::Clique>(keys3);

  auto e12 = boost::make_shared<GaussianMRBayesTree::Edge>(c1, c2);
  auto e21 = boost::make_shared<GaussianMRBayesTree::Edge>(c2, c1);
  auto e23 = boost::make_shared<GaussianMRBayesTree::Edge>(c2, c3);
  auto e32 = boost::make_shared<GaussianMRBayesTree::Edge>(c3, c2);
  c1->addEdge(e12);
  c1->addEdge(e21);
  c2->addEdge(e12);
  c2->addEdge(e21);
  c2->addEdge(e32);
  c2->addEdge(e23);
  c3->addEdge(e23);
  c3->addEdge(e32);
  c2->removeEdge(e32);
  c3->removeEdge(e32);

  // check nodes
  EXPECT(ContainerEqual<KeySet>(keys1, c1->allKeys()));
  EXPECT(c1->isParentOf(c2));
  EXPECT(c1->isChildOf(c2));
  EXPECT(c2->isParentOf(c1));
  EXPECT(c2->isChildOf(c1));
  EXPECT(c2->isParentOf(c3));
  EXPECT(c3->isChildOf(c2));
  EXPECT(!c2->isChildOf(c3));
  EXPECT(!c3->isParentOf(c2));

  GaussianMRBayesTree::EdgeVector expected_parent_c1{e21};
  GaussianMRBayesTree::EdgeVector expected_child_c1{e12};
  GaussianMRBayesTree::EdgeVector expected_parent_c2{e12};
  GaussianMRBayesTree::EdgeVector expected_child_c2{e21, e23};
  GaussianMRBayesTree::EdgeVector expected_parent_c3{e23};
  GaussianMRBayesTree::EdgeVector expected_child_c3{};
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_parent_c1, c1->parentEdges()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_child_c1, c1->childEdges()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_parent_c2, c2->parentEdges()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_child_c2, c2->childEdges()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_parent_c3, c3->parentEdges()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::EdgeVector>(expected_child_c3, c3->childEdges()));
  EXPECT(e12 == c2->parentEdge(c1));
  EXPECT(e21 == c2->childEdge(c1));

  GaussianMRBayesTree::CliqueVector expected_parent_cliques_c1{c2};
  GaussianMRBayesTree::CliqueVector expected_parent_cliques_c2{c1};
  GaussianMRBayesTree::CliqueVector expected_child_cliques_c1{c2};
  GaussianMRBayesTree::CliqueVector expected_child_cliques_c2{c1, c3};
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_parent_cliques_c1, c1->parentCliques()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_parent_cliques_c2, c2->parentCliques()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_child_cliques_c1, c1->childCliques()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_child_cliques_c2, c2->childCliques()));

  GaussianMRBayesTree::CliqueVector expected_neighbor_c1{c2};
  GaussianMRBayesTree::CliqueVector expected_neighbor_c2{c1, c3};
  GaussianMRBayesTree::CliqueVector expected_neighbor_c3{c2};
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_neighbor_c1, c1->neighborCliques()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_neighbor_c2, c2->neighborCliques()));
  EXPECT(ContainerEqual<GaussianMRBayesTree::CliqueVector>(expected_neighbor_c3, c3->neighborCliques()));

  // check edges
  EXPECT(e12->dualDirection());
  EXPECT(e21->dualDirection());
  EXPECT(!e23->dualDirection());
  KeySet expected_frontal_12, expected_separator_12;
  expected_frontal_12.insert(2);
  expected_separator_12.insert(1);
  EXPECT(ContainerEqual<KeySet>(expected_frontal_12, e12->frontalKeys()));
  EXPECT(ContainerEqual<KeySet>(expected_separator_12, e12->separatorKeys()));

  GaussianFactorGraph graph;
  SharedDiagonal model = noiseModel::Unit::Create(1);
  graph +=
    JacobianFactor(1, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(2, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(0, 1.0*I_1x1, 1, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(1, 1.0*I_1x1, 2, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(2, 1.0*I_1x1, 3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(2, 1.0*I_1x1, 4, 1.0*I_1x1, Vector1(1.0), model);
  VariableIndex vi(graph);
  auto elim_factors = e12->elimFactors(graph, vi);
  GaussianFactorGraph expected_elim_factors;
  expected_elim_factors.push_back(graph.at(1));
  expected_elim_factors.push_back(graph.at(3));
  EXPECT(ContainerEqual<GaussianFactorGraph>(expected_elim_factors, elim_factors));

  GaussianFactorGraph expected_factors_in_c2;
  expected_factors_in_c2.push_back(graph.at(0));
  expected_factors_in_c2.push_back(graph.at(1));
  expected_factors_in_c2.push_back(graph.at(3));
  auto factors_in_c2 = c2->factorsInClique(graph, vi);
  EXPECT(ContainerEqual<GaussianFactorGraph>(expected_factors_in_c2, factors_in_c2));
}

/** In this test, the constructor of MRBT is tested by comparing a MRBT
 * constructed MRBT from factor graph to another MRBT manually constructed by
 * specifying all marginals and conditionals. The computeDelta function is
 * tested as well
 *
 * Test Scenario
 * graph:
 *  l3 - l2 - x1 - r2 - r3
 *            |
 *            x0 - .
 *
 * junction tree:
 *  (l2, l3) <-> (x1, l2) <-> (x1, r2) <-> (r2, r3)
 *                |
 *                v
 *               (x0, x1)
 *
 */
TEST(MRBayesTree, Constructor) {
  Key x0 = Symbol('x', 0);
  Key x1 = Symbol('x', 1);
  Key l2 = Symbol('l', 2);
  Key l3 = Symbol('l', 3);
  Key r2 = Symbol('r', 2);
  Key r3 = Symbol('r', 3);

  GaussianFactorGraph graph;
  SharedDiagonal model = noiseModel::Unit::Create(1);

  graph +=
    JacobianFactor(x0, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(x0, -1.0*I_1x1, x1, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(x1, -1.0*I_1x1, l2, 1.0*I_1x1, Vector1(-1.0), model),
    JacobianFactor(l2, -1.0*I_1x1, l3, 1.0*I_1x1, Vector1(-1.0), model),
    JacobianFactor(x1, -1.0*I_1x1, r2, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(r2, -1.0*I_1x1, r3, 1.0*I_1x1, Vector1(1.0), model);
  
  Ordering order; 
  order += x0, r3, r2, x1, l2, l3;

  GaussianJunctionTree jt(GaussianEliminationTree(graph, order));

  GaussianMRBayesTree::RootKeySetMap other_root_keys_map;
  KeySet keys0, keys1;
  // keys0.insert(l3);
  keys1.insert(r3);
  // other_root_keys_map[0] = keys0;
  other_root_keys_map[1] = keys1;
  GaussianMRBayesTree mrbt(jt, graph, 0, other_root_keys_map);

  KeySet allKeys1, allKeys2, allKeys3, allKeys4, allKeys5;
  allKeys1.insert(l2); allKeys1.insert(l3);
  allKeys2.insert(l2); allKeys2.insert(x1);
  allKeys3.insert(x0); allKeys3.insert(x1);
  allKeys4.insert(r2); allKeys4.insert(x1);
  allKeys5.insert(r2); allKeys5.insert(r3);

  typedef GaussianMRBayesTree::Clique Clique;
  typedef GaussianMRBayesTree::Edge Edge;
  auto c1 = boost::make_shared<Clique>(allKeys1);
  auto c2 = boost::make_shared<Clique>(allKeys2);
  auto c3 = boost::make_shared<Clique>(allKeys3);
  auto c4 = boost::make_shared<Clique>(allKeys4);
  auto c5 = boost::make_shared<Clique>(allKeys5);

  auto e12 = boost::make_shared<Edge>(c1, c2);
  auto e21 = boost::make_shared<Edge>(c2, c1);
  auto e23 = boost::make_shared<Edge>(c2, c3);
  auto e24 = boost::make_shared<Edge>(c2, c4);
  auto e42 = boost::make_shared<Edge>(c4, c2);
  auto e45 = boost::make_shared<Edge>(c4, c5);
  auto e54 = boost::make_shared<Edge>(c5, c4);

  c1->addEdge(e12); c1->addEdge(e21);
  c2->addEdge(e12); c2->addEdge(e21); c2->addEdge(e23); c2->addEdge(e24); c2->addEdge(e42);  
  c3->addEdge(e23); 
  c4->addEdge(e24); c4->addEdge(e42); c4->addEdge(e45); c4->addEdge(e54);
  c5->addEdge(e45); c5->addEdge(e54);


  auto fx0 = graph.at(0);
  auto fx0x1 = graph.at(1);
  auto fx1l2 = graph.at(2);
  auto fl2l3 = graph.at(3);
  auto fx1r2 = graph.at(4);
  auto fr2r3 = graph.at(5);

  auto elim_fun = EliminateCholesky;

  GaussianFactorGraph graph21;
  graph21.add(fl2l3);
  Ordering ordering21; ordering21.push_back(l3);
  auto elim_result21 = elim_fun(graph21, ordering21);
  e21->setEliminationResult(elim_result21);

  GaussianFactorGraph graph23;
  graph23.add(fx0x1); graph23.add(fx0);
  Ordering ordering23; ordering23.push_back(x0);
  auto elim_result23 = elim_fun(graph23, ordering23);
  e23->setEliminationResult(elim_result23);

  GaussianFactorGraph graph45;
  graph45.add(fr2r3); 
  Ordering ordering45; ordering45.push_back(r3);
  auto elim_result45 = elim_fun(graph45, ordering45);
  e45->setEliminationResult(elim_result45);

  GaussianFactorGraph graph24;
  graph24.add(fx1r2); graph24.add(e45->marginal());
  Ordering ordering24; ordering24.push_back(r2);
  auto elim_result24 = elim_fun(graph24, ordering24);
  e24->setEliminationResult(elim_result24);

  GaussianFactorGraph graph12;
  graph12.add(fx1l2); graph12.add(e24->marginal()); graph12.add(e23->marginal());
  Ordering ordering12; ordering12.push_back(x1);
  auto elim_result12 = elim_fun(graph12, ordering12);
  e12->setEliminationResult(elim_result12);

  GaussianFactorGraph graph42;
  graph42.add(fx1l2); graph42.add(e21->marginal()); graph42.add(e23->marginal());
  Ordering ordering42; ordering42.push_back(l2);
  auto elim_result42 = elim_fun(graph42, ordering42);
  e42->setEliminationResult(elim_result42);

  GaussianFactorGraph graph54;
  graph54.add(fx1r2); graph54.add(e42->marginal());
  Ordering ordering54; ordering54.push_back(x1);
  auto elim_result54 = elim_fun(graph54, ordering54);
  e54->setEliminationResult(elim_result54);


  // compare the constructed tree to manually created one
  GaussianMRBayesTree::RootCliqueMap root_clique_map;
  root_clique_map[0] = c1;
  root_clique_map[1] = c5;
  GaussianMRBayesTree expected_tree(root_clique_map);
  EXPECT(expected_tree.equals(mrbt));

  // check computeDetla
  VectorValues delta;
  delta.insert(x0, (Vector(1) << 10).finished());
  delta.insert(x1, (Vector(1) << 10).finished());
  delta.insert(l2, (Vector(1) << 10).finished());
  delta.insert(l3, (Vector(1) << 10).finished());
  delta.insert(r2, (Vector(1) << 10).finished());
  delta.insert(r3, (Vector(1) << 10).finished());
  GaussianMRBayesTree::SharedClique root_clique = mrbt.roots().at(0);
  mrbt.computeDelta(root_clique, root_clique, graph, VariableIndex(graph), delta);
  VectorValues expected_delta;
  expected_delta.insert(x0, (Vector(1) << 0).finished());
  expected_delta.insert(x1, (Vector(1) << 0).finished());
  expected_delta.insert(l2, (Vector(1) << -1).finished());
  expected_delta.insert(l3, (Vector(1) << -2).finished());
  expected_delta.insert(r2, (Vector(1) << 1).finished());
  expected_delta.insert(r3, (Vector(1) << 2).finished());
  EXPECT(assert_equal(expected_delta, delta));
}


#define a0 Symbol('a', 0)
#define a1 Symbol('a', 1)
#define a2 Symbol('a', 2)
#define a3 Symbol('a', 3)
#define a4 Symbol('a', 4)
#define a5 Symbol('a', 5)
#define a6 Symbol('a', 6)
#define b0 Symbol('b', 0)
#define b1 Symbol('b', 1)
#define b2 Symbol('b', 2)
#define b3 Symbol('b', 3)
#define b4 Symbol('b', 4)
#define b5 Symbol('b', 5)
#define b6 Symbol('b', 6)

/* Test Scenario
 * graph:
 *  b0 - b1 - b2 - b3 - b4 - b5
 *               / |  /
 *  a0 - a1 - a2 - a3 - a4 - a5 
 * 
 * 
*/
TEST(MRBayesTree, ToyExample) {

  GaussianFactorGraph graph;
  SharedDiagonal model = noiseModel::Unit::Create(1);

  graph +=
    JacobianFactor(a0, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(b0, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(a0, -1.0*I_1x1, a1, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b0, -1.0*I_1x1, b1, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a1, -1.0*I_1x1, a2, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b1, -1.0*I_1x1, b2, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a2, -1.0*I_1x1, a3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b2, -1.0*I_1x1, b3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a3, -1.0*I_1x1, a4, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b3, -1.0*I_1x1, b4, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a4, -1.0*I_1x1, a5, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b4, -1.0*I_1x1, b5, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a2, -1.0*I_1x1, b3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a3, -1.0*I_1x1, b3, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(a3, -1.0*I_1x1, b4, 1.0*I_1x1, Vector1(1.0), model);

  Ordering order; 
  order += b0, b1, b2, a0, a1, a2, b5, b3, b4, a3, a4, a5;

  GaussianJunctionTree jt(GaussianEliminationTree(graph, order));

  GaussianMRBayesTree::RootKeySetMap other_root_keys_map;
  KeySet keys0, keys1;
  // keys0.insert(a5);
  keys1.insert(b5);
  // other_root_keys_map[0] = keys0;
  other_root_keys_map[1] = keys1;
  GaussianMRBayesTree mrbt(jt, graph, 0, other_root_keys_map);

  // mrbt.saveGraph("../../results/test.dot", MultiRobotKeyFormatter);
  // mrbt.print();
  // mrbt.exportTree("../visualization/tree.json", mrbt.defaultLayout());

}


TEST(MRBayesTree, errorcase) {
  SharedDiagonal model = noiseModel::Unit::Create(1);
  GaussianFactorGraph graph;
  graph +=
    JacobianFactor(a4, -1.0*I_1x1, a5, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a3, -1.0*I_1x1, a4, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b3, -1.0*I_1x1, b4, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a3, -1.0*I_1x1, b3, 1.0*I_1x1, Vector1(0.0), model),
    JacobianFactor(a3, -1.0*I_1x1, b4, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(b4, 1.0*I_1x1, Vector1(4.0), model),
    JacobianFactor(b3, 1.0*I_1x1, Vector1(3.0), model),
    JacobianFactor(a2, -1.0*I_1x1, a3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a2, -1.0*I_1x1, b3, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a2, 1.0*I_1x1, Vector1(2.0), model),
    JacobianFactor(a6, 1.0*I_1x1, Vector1(7.0), model),
    JacobianFactor(a5, -1.0*I_1x1, a6, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(a2, -1.0*I_1x1, a6, 1.0*I_1x1, Vector1(4.0), model);
    
  Ordering order; 
  order += a4, b4, b3, a3, a2, a5, a6;

  GaussianMRBayesTree::RootKeySetMap other_root_keys_map;
  KeySet keys_a, keys_b;
  // keys_a.insert(a6);
  keys_b.insert(b4);
  // other_root_keys_map[0] = keys_a;
  other_root_keys_map[1] = keys_b;

  GaussianMRBayesTree mrbt(graph, order, 0, other_root_keys_map);

}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}