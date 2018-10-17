#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include <vector>
#include <numeric>
#include <limits>

using namespace std;

    // a Tree class should have 'near(State,scalar)' function that return a list of
    // index of nodes that is near with the state where the scalar is nearest neighbor radius
    // should overloads operator() that takes TreeIndex as argument and return the State
    // should have 'insert' function that takes (State x, TreeIdx p) to insert a new State
    // 'x' with 'p' as parent of 'x' an return TreeIndex
    // the Tree should also have 'reset' function that clears the tree
    // a Cost class should have overloaded operator () and takes (State,State) as
    // argument and return a scalar cost
    // a Sampler class should have overloaded operator() and return a state
    // a Neighbor radius should have overloaded operator() that takes int i as argument
    // where i is the iteration
    // a CollisionChecker should have overloaded operator() that takes Edge as
    // argument and return bool
    // a GoalChecker should have overloaded operator () and take State as argument
    // and return bool
    // connect states, should return edge by calling operator() and State, State as
    // argument
    // a State class should have 'setCost' and 'cost' member function that set and
    // return the cost of the state respectively
    // TreeIndex represent tree index
    // Scalar represent cost
template
<
    typename Tree, typename Cost, typename Sampler, typename NeighborRadius,
    typename CollisionChecker, typename GoalChecker, typename Connector,
    typename State = typename Tree::State,
    typename Edge = typename Connector::Edge,
    typename TreeIndex = typename Tree::Index,
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

    _grow(xr, i, xg);

    return (goal_idx ? true : false);
  }

  bool insertGoal(State &s, int it=-1) {
    int iter = it;
    if(it<0) iter = i;
    auto idx = _grow(s, iter);
    if(it<0) i = iter;
    if(idx > 0) {
      if(goal_idx) *goal_idx = idx;
      else goal_idx = new TreeIndex(idx);
    }
    return idx > 0;
  }

  TreeIndex goalIndex() const { return (goal_idx ? *goal_idx : -1); }

  void setIteration(int it) { i = it; }

  TreeIndex _grow(State &xr, int &it, State *xg=nullptr)
  {
    // find collision-free neigbor with lowest cost and make it as parent
    // first, decide the neighbor radius of the current iteration
    // auto r = radius(i++);
    auto r = radius(it+1);
    Scalar *lowest_cost = nullptr;
    TreeIndex *parent = nullptr;
    // get the list of nearest state from the tree
    auto nearest = tree.nearest(xr,r);
    // iterate through the nearest list and find the collision-free trajectory
    // (or path) from xr to the parent state with lowest cost
    Edge edge;

    if(goal_idx)
      if(tree(*goal_idx) == xr)
        lowest_cost = new Scalar(tree(*goal_idx).cost());

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

    TreeIndex xr_idx = -1;
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
            auto s1 = tree(n);
            auto s0 = tree(xr_idx);
            auto e = connect(s0,s1);
            if(!collision(e)) {
              tree(n).setCost(xr_cost+c);
              tree.setParent(n,xr_idx);
              tree.setEdge(n, e);
            } // collision
          } // cost
        }
      } // nearest
      it = it+1;
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
          auto s1 = *xg;
          auto s0 = tree(xr_idx);
          auto e = connect(s0, s1);
          if(!collision(e)) {
            tree(*goal_idx).setCost(xr_cost+c);
            tree.setParent(*goal_idx,xr_idx);
            tree.setEdge(*goal_idx,e);
          }
        }
      }
      else { // newly found goal, insert
        auto s1 = *xg;
        auto s0 = tree(xr_idx);
        auto e = connect(s0, s1);
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
    return xr_idx;
  }

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

template
<
    typename Tree, typename Sampler, typename NeighborRadius,
    typename BatchCollisionChecker, typename GoalChecker,
    typename State = typename Tree::State,
    typename Edge = typename BatchCollisionChecker::Edge,
    typename TreeIndex = typename Tree::Index,
    typename Scalar = typename BatchCollisionChecker::Scalar
    >
class RRTStarBatch
{
public:
  RRTStarBatch(Tree &tree, Sampler &sampler, BatchCollisionChecker &checker, NeighborRadius &radius, GoalChecker &goal)
    : tree(tree), sampler(sampler), collision(checker), radius(radius), goal(goal)
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
    _grow(xr, i, xg);
    return (goal_idx ? true : false);
  }
  bool insertGoal(State &s, int it=-1)
  {
    int iter = it;
    if(it<0) iter = i;
    int li = iter;
    auto idx = _grow(s,iter);
    if(it<0) i = iter;
    if(idx > 0) {
      if(goal_idx) *goal_idx = idx;
      else goal_idx = new TreeIndex(idx);
    }
    return (li != iter);
  }

  TreeIndex goalIndex() const { return (goal_idx ? *goal_idx : -1); }

  void setIteration(int it) { i = it; }

private:
  TreeIndex _grow(State &xr, int &it, State *xg = nullptr) {
    auto r = radius(it+1);
    Scalar *lowest_cost = nullptr;
    TreeIndex *parent = nullptr;
    auto nearest_idx = tree.nearest(xr,r);
    auto states = tree.states(nearest_idx);

    if(goal_idx)
      if(tree(*goal_idx) == xr)
        lowest_cost = new Scalar(tree(*goal_idx).cost());

    vector<Edge> edges;
    vector<bool> collide;
    vector<Scalar> costs;

    // take nearest (indexes of state) and xr (state) as input args
    // & edges, collide, & costs as output args
    // collision(nearest_idx, xr, edges, collide, costs);
    collision(states, xr, edges, collide, costs);

    Edge edge;
    auto n = edges.size();
    for(size_t i=0; i<n; i++) {
      if(!collide[i]) {
        if(!lowest_cost) {
          lowest_cost = new Scalar(costs[i]);
          parent = new TreeIndex(nearest_idx[i]);
          edge = edges[i];
        }
        else {
          if(costs[i] < *lowest_cost) {
            *lowest_cost = costs[i];
            if(parent) *parent = nearest_idx[i];
            else parent = new TreeIndex(nearest_idx[i]);
            edge = edges[i];
          }
        }
      }
    }

    TreeIndex xr_idx = -1;
    if(parent) {
      auto xr_cost = *lowest_cost + tree(*parent).cost();
      xr.setCost(xr_cost);
      xr_idx = tree.insert(xr, *parent, edge);
      edges.clear(); collide.clear(); costs.clear();
      // assuming (and therefore is should) states is vector
      if(xg) states.push_back(*xg);
      // take xr (state) and states as input args
      // & edges, collide, & costs as output args
      collision(xr, states, edges, collide, costs);
      auto n = nearest_idx.size();
      for(size_t i=0; i<n; i++) {
        auto idx = nearest_idx[i];
        if(idx != *parent) {
          auto c = costs[i];
          auto nc = tree(idx).cost();
          if((nc > xr_cost + c) && (!collide[i])) {
            tree(idx).setCost(xr_cost+c);
            tree.setParent(idx, xr_idx);
            tree.setEdge(idx, edges[i]);
          }
        }
      }
      it = it+1;
      if(xg) {
        if(!collide.back()) {
          // at this point you could connect newly added state to xg (if given)
          auto c = costs.back();
          if(goal_idx) {
            auto cc = tree(*goal_idx).cost();
            // we have found goal previously,
            // check if this new connection has lower cost
            if(cc > xr_cost + c) {
              // the new connection has lower cost, reconnect
              // no need to insert becouse it has already inserted
              // just change parent, cost, & trajectory
              tree(*goal_idx).setCost(xr_cost+c);
              tree.setParent(*goal_idx,xr_idx);
              tree.setEdge(*goal_idx,edges.back());
            }
          }
          else {
            // for newly added goal, just insert it
            xg->setCost(xr_cost+c);
            auto g_idx = tree.insert(*xg, xr_idx, edges.back());
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
    }
    return xr_idx;
  }

private:
  int i = 0;
  Tree &tree;
  Sampler &sampler;
  GoalChecker &goal;
  BatchCollisionChecker &collision;
  NeighborRadius &radius;
  TreeIndex *goal_idx = nullptr;
};

#endif // RRTSTAR_HPP
