#pragma once
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
using namespace tbb;

struct TBBTest {
//public:
	mutable int a=0;

	TBBTest() {	}

	void operator() (const blocked_range<size_t>& range) const {
		a = 1;
		for (size_t i = range.begin(); i <= range.end(); i++) {
			a++;
		}
	}

	int getA() const {
		return a;
	}
};