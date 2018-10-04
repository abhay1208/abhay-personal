#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>
#include "Graph.h"
using namespace std;

int main() {
	srand (time(NULL));

	Graph g(3, 10, 1000);
	g.findShortestPath(0,18);
	return 0;
}
