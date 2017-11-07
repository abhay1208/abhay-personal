#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>
#include "Hex.h"
using namespace std;

int main() {
	Graph* g;
	Hex hex(10);
	g = &hex;
	hex.createBoard();
	g->findShortestPath(0, 8);
	hex.printBoard();
	return 0;
}
