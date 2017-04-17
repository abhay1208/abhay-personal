#include <iostream>
#include <vector>
#include "Graph.h"
#include "PriorityQueue.hpp"
using namespace std;

int main() {

	vector<double> temp;
	for (int i = 0; i < 10; i++) {
		temp.push_back(15 - i);
	}

	PriorityQueue<double> queue(temp);
	queue.printQueue();
}
