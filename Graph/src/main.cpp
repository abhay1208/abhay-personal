#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>
#include "Graph.h"
using namespace std;

int main() {
	srand (time(NULL));

	Graph g("sampleData.txt");
	cout << "Constructed graph from the given data file:" << endl;
	g.printGraph();
	cout << "\n \n";
	g.findMST();
	g.findShortestPath(0,18);
	g.getShortestPathCost();
	//g.getShortestPathCost();
	cout << "Total nodes are " << g.getNumNodes() << endl;
	return 0;
}
