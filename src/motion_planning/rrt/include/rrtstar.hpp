#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include <vector>
#include <numeric>
#include <limits>

using namespace std;

template
<
    // a Tree class should have 'near(State,scalar)' function that return a list of
    // index of nodes that is near with the state where the scalar is nearest neighbor radius
    // should overloads operator() that takes TreeIndex as argument and return the State
    // should have 'insert' function that takes (State x, TreeIdx p) to insert a new State
    // 'x' with 'p' as parent of 'x' an return TreeIndex
    // the Tree should also have 'reset' function that clears the tree
    typename Tree,
    // a Cost class should have overloaded operator () and takes (State,State) as
    // argument and return a scalar cost
    typename Cost,
    // a Sampler class should have overloaded operator() and return a state
    typename Sampler,
    // a Neighbor radius should have overloaded operator() that takes int i as argument
    // where i is the iteration
    typename NeighborRadius,
    // a CollisionChecker should have overloaded operator() that takes Edge as
    // argument and return bool
    typename CollisionChecker,
    // a GoalChecker should have overloaded operator () and take State as argument
    // and return bool
    typename GoalChecker,
    // connect states, should return edge by calling operator() and State, State as
    // argument
    typename Connector,
    // a State class should have 'setCost' and 'cost' member function that set and
    // return the cost of the state respectively
    typename State = typename Tree::State,
    typename Edge = typename Connector::Edge,
    // represent tree index
    typename TreeIndex = typename Tree::Index,
    // represent cost
    typename Scalar = typename Cost::Scalar
    >
class RRTStar
{
public:
  RRTStar(Tree &tree, Cost &cost, Sampler &sampler, CollisionChecker &checker, NeighborRadius &radius, GoalChecker &goal, Connector &connect) :
    tree(tree), cost(cost), sampler(sampler), collision(checker), radius(radius), goal(goal), connect(connect)
  {}
  void setStart(const State &start) {
    i = 0;
    // xstart = start;
    if(goal_idx) {
      delete goal_idx;
      goal_idx = nullptr;
    }
    tree.reset();
    tree.insert(start, -1);
  }
  void setGoal(const State &goal) {
    // i = 0;
    // xgoal = goal;
  }
  bool grow(State *xg = nullptr) {
    // sample new state
    auto xr = sampler();

    // find collision-free neigbor with lowest cost and make it as parent
    // first, decide the neighbor radius of the current iteration
    // auto r = radius(i++);
    auto r = radius(i+1);
    Scalar *lowest_cost = nullptr;
    TreeIndex *parent = nullptr;
    // get the list of nearest state from the tree
    auto nearest = tree.nearest(xr,r);
    // iterate through the nearest list and find the collision-free trajectory
    // (or path) from xr to the parent state with lowest cost
    Edge edge;
    for(const auto& n : nearest) {
      auto e = connect(tree(n), xr);
      if(!collision(e)) {
        auto c = cost(tree(n),xr) + tree(n).cost();
        if(lowest_cost) {
          if(*lowest_cost > c) {
            edge = e;
            *lowest_cost = c;
            if(parent) *parent = n;
            else parent = new TreeIndex(n);
          }
        }
        else {
          edge = e;
          lowest_cost = new Scalar(c);
          if(parent) *parent = n;
          else parent = new TreeIndex(n);
        }
      }
    }

    TreeIndex xr_idx;
    if(parent) {
      // only insert the new node if it has parent
      // that also means that it is collision free
      auto xr_cost = cost(tree(*parent),xr) + tree(*parent).cost();
      xr.setCost(xr_cost);
      xr_idx = tree.insert(xr, *parent, edge);
      // given neighboring states, check if the states could be reconnected to the
      // new state with lower cost (rewire)
      for(const auto &n : nearest) {
        if(n != *parent) {
          auto c = cost(xr,tree(n));
          auto nc = tree(n).cost();
          // the cost is lower by reconnecting tree(n) to xr
          if(nc > xr_cost+c) {
            // but only insert if collision-free
            auto e = connect(xr,tree(n));
            if(!collision(e)) {
              tree(n).setCost(xr_cost+c);
              tree.setParent(n,xr_idx);
              tree.setEdge(n, e);
            } // collision
          } // cost
        }
      } // nearest
      i = i+1;
    } // parent

    // check if the new state is the goal,
    // if you want an exact goal state, feed the xg
    auto is_goal = false;
    if(xg && parent) {
      auto c = cost(xr, *xg);
      auto xr_cost = tree(xr_idx).cost();
      if(goal_idx) { // goal has been previously found, modify
        auto cc = tree(*goal_idx).cost();
        if(cc > xr_cost+c) {
          auto e = connect(xr, *xg);
          if(!collision(e)) {
            tree(*goal_idx).setCost(xr_cost+c);
            tree.setParent(*goal_idx,xr_idx);
            tree.setEdge(*goal_idx,e);
          }
        }
      }
      else { // newly found goal, insert
        auto e = connect(xr, *xg);
        if(!collision(e)) {
          xg->setCost(xr_cost+c);
          auto g_idx = tree.insert(*xg, xr_idx, e);
          goal_idx = new TreeIndex(g_idx);
        }
      }
    }
    else {
      if(goal(xr)) {
        if(goal_idx) *goal_idx = xr_idx;
        else goal_idx = new TreeIndex(xr_idx);
      }
    }

    return (goal_idx ? true : false);
  }

  bool insertGoal(State &s, int it=-1) {
    auto iter = it;
    if(it<0) iter = i;
    auto r = radius(iter+1);
    Scalar *lowest_cost = nullptr;
    TreeIndex *parent = nullptr;

    auto nearest = tree.nearest(s,r);
    Edge edge;

    if(goal_idx)
      if(tree(*goal_idx) == s)
        lowest_cost = new Scalar(tree(*goal_idx).cost());

    for(const auto& n : nearest) {
      auto e = connect(tree(n), s);
      if(!collision(e)) {
        auto c = cost(tree(n),s) + tree(n).cost();
        if(lowest_cost) {
          if(*lowest_cost > c) {
            edge = e;
            *lowest_cost = c;
            if(parent) *parent = n;
            else parent = new TreeIndex(n);
          }
        }
        else {
          edge = e;
          lowest_cost = new Scalar(c);
          if(parent) *parent = n;
          else parent = new TreeIndex(n);
        }
      }
    }

    if(parent) {
      if(it<0) i = i+1;
      auto xr_cost = cost(tree(*parent),s) + tree(*parent).cost();
      s.setCost(xr_cost);
      auto xr_idx = tree.insert(s, *parent, edge);
      if(goal_idx) {
        *goal_idx = xr_idx;
      }
      else {
        goal_idx = new TreeIndex(xr_idx);
      }
      for(const auto &n : nearest) {
        if(n != *parent) {
          auto c = cost(s,tree(n));
          auto nc = tree(n).cost();
          if(nc > xr_cost+c) {
            auto e = connect(s,tree(n));
            if(!collision(e)) {
              tree(n).setCost(xr_cost+c);
              tree.setParent(n,xr_idx);
              tree.setEdge(n, e);
            }
          }
        }
      }
    }

    if(parent) return true;
    else return false;
  }

  TreeIndex goalIndex() const { return (goal_idx ? *goal_idx : -1); }

  void setIteration(int it) { i = it; }

private:
  int i = 0;
  Tree &tree;
  Cost &cost;
  Sampler &sampler;
  GoalChecker &goal;
  Connector &connect;
  CollisionChecker &collision;
  NeighborRadius &radius;
  TreeIndex *goal_idx = nullptr;
  // State xstart;
  // State xgoal;
  // std::vector<State> solutions;
};

#endif // RRTSTAR_HPP
