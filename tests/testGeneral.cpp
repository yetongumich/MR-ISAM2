#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>


using namespace std;
using namespace gtsam;

/* Test Scenario
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
TEST(EliminationTree, eliminate) {
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
    JacobianFactor(x0, 1.0*I_1x1, x1, 1.0*I_1x1, Vector1(1.0), model),
    JacobianFactor(x1, 1.0*I_1x1, l2, 1.0*I_1x1, Vector1(3.0), model),
    JacobianFactor(l2, 1.0*I_1x1, l3, 1.0*I_1x1, Vector1(5.0), model),
    JacobianFactor(x1, 1.0*I_1x1, r2, 1.0*I_1x1, Vector1(3.0), model),
    JacobianFactor(r2, 1.0*I_1x1, r3, 1.0*I_1x1, Vector1(5.0), model);
  
  Ordering order; 
  order += x0, r3, r2;

  VariableIndex index = VariableIndex(graph);

  auto eliminationTree = GaussianEliminationTree(graph, index, order);

  eliminationTree.print();

  std::cout << eliminationTree.roots().size() << "\n";

  // auto result = eliminationTree.eliminate(EliminateQR);
  // auto bayes_net = result.first;
  // auto remaining_graph = result.second;
  // bayes_net->print();
  // remaining_graph->print();
  // GaussianJunctionTree jt(GaussianEliminationTree(graph, order));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}