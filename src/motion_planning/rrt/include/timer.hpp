#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

template
<
typename numeric,
typename ratio = std::milli,
typename clock = std::chrono::high_resolution_clock
>
struct Timer {
  typedef std::chrono::time_point<clock> TP;
  typedef std::chrono::duration<numeric,ratio> D;

  Timer() {}

  void reset()
  {
    best = numeric(1e10);
    worst = numeric(-1e10);
    total = numeric(0.0);
    count = 0;
  }

  void start()
  {
    t0 = clock::now();
  }

  D stop()
  {
    t1 = clock::now();
    auto t = dt();
    total += t.count();
    count++;
    if(t.count() < best)
      best = t.count();
    else if(t.count() > worst)
      worst = t.count();
    avg = total/count;
    return t;
  }

  inline
  D dt()
  {
    return t1-t0;
  }

  TP t0;
  TP t1;
  numeric best = numeric(1e10);
  numeric worst = numeric(-1e10);
  numeric avg = numeric(0.0);
  numeric total = numeric(0.0);
  int count = 0;
};
#endif // TIMER_HPP
