#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>
#include "Hex.h"
using namespace std;

int main() {
	Graph* g;
	Hex hex(3);
	g = &hex;
	hex.createBoard();
	g->findShortestPath(0, 8);
	return 0;
}
