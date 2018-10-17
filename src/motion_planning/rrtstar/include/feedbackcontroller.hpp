#ifndef FEEDBACKCONTROLLER_HPP
#define FEEDBACKCONTROLLER_HPP

#include <vector>
#include <tuple>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

#ifdef TEST
#include "logger.hpp"
#endif
#ifdef FEEDBACKCONTROLLER_DEBUG_TIME
#include "timer.hpp"
#endif

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>

template<typename Type, int N, int M>
bool operator > (Eigen::Matrix<Type,N,M> m, Type v){
  return m.norm() > v;
}

template<typename Type, int N, int M>
bool operator > (Eigen::Matrix<Type,N,M> m, Eigen::Matrix<Type,N,M> n){
  return m.norm() > n.norm();
}

template
<
typename TimeType,
typename SystemType,
typename ControlType,
typename StateType = typename SystemType::State,
typename InputType = typename ControlType::InputType
>
class FeedbackController {
public:
  typedef std::vector<StateType> States;
  typedef std::vector<InputType> Inputs;
public:
  FeedbackController(const SystemType &system, const ControlType &control)
  : system(system), control(control) {}

#define DT (0.0005)
  std::vector<tuple<StateType,InputType>> solve(const StateType &init, const StateType &target, const StateType &eps, const TimeType &dt = TimeType(DT)) {
    std::vector<tuple<StateType,InputType>> res;
    auto x = init;
    StateType dx = target-x;
    TimeType t = 0.0;
#ifdef TEST
    Logger logger;
    std::stringstream ss;
#ifdef FEEDBACKCONTROLLER_DEBUG_TIME
    Timer<double> timer;
#endif
#endif
    while(dx > eps) {
#ifdef TEST
#ifdef FEEDBACKCONTROLLER_DEBUG_TIME
      timer.start();
#endif
#endif
      t += dt;
      auto c = control.solve(-dx);
      auto s = system.solve(dt, x, c);
      res.push_back(std::make_tuple(s,c));
      x = s;
      dx = target-x;
#ifdef TEST
#ifdef FEEDBACKCONTROLLER_DEBUG_TIME
      auto compute_time = timer.stop();
      logger.log("compute time : ", compute_time.count());
//      ss << "compute time : " << compute_time.count() << std::endl;
#endif
      logger.log("t : ", t);
      logger.log("target : \n", target);
      logger.log("c : \n", c);
      logger.log("x : \n", x);
      logger.log("dx : \n", dx);
      logger.log("dt : ", dt);
//      ss << "t : " << t << std::endl;
//      ss << "target :\n" << target << std::endl;
//      ss << "c :\n" << c << std::endl;
//      ss << "x :\n" << x << std::endl;
//      ss << "dx :\n" << dx << std::endl;
//      ss << "dt : " << dt << std::endl;
#endif
    }

#ifdef TEST
#ifdef FEEDBACKCONTROLLER_DEBUG_TIME
    logger.log("compute time:\n");
    logger.log("average : ", timer.avg);
    logger.log("best : ", timer.best);
    logger.log("worst : ", timer.worst);
    logger.log("operations : ", timer.count);
    logger.log("total time : ", timer.total);
//      ss << "compute time : "
//         << "average = " << timer.avg << "; "
//         << "best = " << timer.best << "; "
//         << "worst = " << timer.worst << "; "
//         << "operations = " << timer.count << "; "
//         << "total = " << timer.total << std::endl;
#endif // TIME

#ifdef FEEDBACKCONTROLLER_DEBUG_PRINT
    std::cout << "[FeedbackController]" << std::endl;
    logger.print();
//    std::cout << ss.str() << std::endl;
    std::cout << "[FeedbackController]" << std::endl;
#endif // DEBUG_PRINT

#ifdef FEEDBACKCONTROLLER_DEBUG_LOG
    std::string filename;
#ifdef FEEDBACKCONTROLLER_DEBUG_LOG_FILENAME
    filename = std::string(FEEDBACKCONTROLLER_DEBUG_LOG_FILENAME);
#else
    filename = std::string(std::getenv("HOME")) + "/feedback_controller_log.txt";
#endif // DEBUG_LOG_FILENAME
    logger.save(filename);
//    std::ofstream writer(filename);
//    writer << ss.str();
//    writer.close();
#endif // DEBUG_LOG
#endif // TEST

    return res;
  }
public:
  const SystemType &system;
  const ControlType &control;
};

#endif // FEEDBACKCONTROLLER_HPP
