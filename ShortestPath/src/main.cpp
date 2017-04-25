#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include "ShortestPath.h"
#include <numeric>
using namespace std;

int main() {
	srand (time(NULL));

	/*
	 * Run Montey Carlo simulation for density 20
	 */

cout	<< "Running simulation for a 50 nodes graph of density 20" << endl;
	vector <int> pathCost;
	for (int i = 1;i<50;i++) {
		Graph g(20, 10, 50);
		ShortestPath sp(g);
		vector<int> path = sp.findShortestPath(0, i);
		//Get path cost if there exists a path from Node 0 to Node i
		if (!path.empty()) {
			pathCost.push_back(sp.getPathCost());
		}
	}
	double sum = std::accumulate(pathCost.begin(),pathCost.end(), 0);
	cout << "Average path cost for all shortest paths with a graph of density 20 is " << sum/pathCost.size() << endl;

	/*
	 * Run Montey Carlo simulation for density 40
	 */

	cout << "Running simulation for a 50 nodes graph of density 40" << endl;

	pathCost.clear();
	for (int i = 1;i<50;i++) {
		Graph g(40, 10, 50);
		ShortestPath sp(g);
		vector<int> path = sp.findShortestPath(0, i);
		//Get path cost if there exists a path from Node 0 to Node i
		if (!path.empty()) {
			pathCost.push_back(sp.getPathCost());
		}
	}
	sum = std::accumulate(pathCost.begin(),pathCost.end(), 0);
	cout << "Average path cost for all shortest paths with a graph of density 40 is " << sum/pathCost.size() << endl;
	return 0;

}
