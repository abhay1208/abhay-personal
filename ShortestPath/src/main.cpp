#include <iostream>
#include <vector>
#include <time.h>
#include "ShortestPath.h"
using namespace std;

int main() {
	int myArray[9][9] =
			{ { 0, 4, 0, 0, 0, 0, 0, 8, 0 }, { 4, 0, 8, 0, 0, 0, 0, 11, 0 }, {
					0, 8, 0, 7, 0, 4, 0, 0, 2 }, { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
					{ 0, 0, 0, 9, 0, 10, 0, 0, 0 }, { 0, 0, 4, 14, 10, 0, 2, 0,
							0 }, { 0, 0, 0, 0, 0, 2, 0, 1, 6 }, { 8, 11, 0, 0,
							0, 0, 1, 0, 7 }, { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };

	vector < vector<int> > v;
	for (int i = 0; i < 9; i++) {
		vector<int> columnVector(9, 0);
		v.push_back(columnVector);
	}

	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++) {
			v[i][j] = myArray[i][j];
		}
	}

	Graph g(v);
	g.printGraph();
	cout << "total nodes: ";
	cout << g.getNumNodes() << endl;
	ShortestPath path(g, 0, 4);
	path.findShortestPath();
	return 0;

}
