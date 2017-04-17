#include <iostream>
#include <vector>
#include "Graph.h"
#include "PriorityQueue.hpp"
#include "VertexInfo.h"

using namespace std;

int main() {

	vector < VertexInfo > v;

	for (int i; i < 10; i++) {
		v.push_back(VertexInfo(i));
		v[i].setCost(10 - i);
	}

	PriorityQueue < VertexInfo > p(v);
	p.printQueue();

	return 0;
}
