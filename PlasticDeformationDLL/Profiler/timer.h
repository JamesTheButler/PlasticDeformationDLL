#ifndef __TIMER_H
#define __TIMER_H

#include <chrono>
namespace realm
{

inline double GetRealTime()
{
	auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> time_span = t2.time_since_epoch();

	return time_span.count() * 1000.0;
}

inline double GetRealTimeInMS() 
{
	auto t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> time_span = t2.time_since_epoch();

	return time_span.count();
}
} // namespace realm
#endif
