/*
 * ShortestPath.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#include "Graph.h"
#include <vector>

#include "PriorityQueue.hpp"
using namespace std;

#ifndef SRC_SHORTESTPATH_H_
#define SRC_SHORTESTPATH_H_

class ShortestPath {
public:
	ShortestPath(const Graph & g, int sNode, int dNode);
	void findShortestPath();
	virtual ~ShortestPath();
private:
	Graph m_g;
	PriorityQueue<int> m_queue;
	int m_sNode;
	int m_dNode;
	vector<int> m_visitedNodes;
	vector<int> m_unvisitedNodes;

};

#endif /* SRC_SHORTESTPATH_H_ */
