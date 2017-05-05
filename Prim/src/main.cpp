#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>

#include "PrimMST.h"
using namespace std;

int main() {

	Graph g("sampleData.txt");
	cout << "Constructed graph from the given data file:" << endl;
	g.printGraph();
	PrimMST p(g);
	cout << "\n \n";
	p.findMST();
	return 0;

}

