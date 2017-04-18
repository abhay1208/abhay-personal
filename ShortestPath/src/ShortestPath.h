/*
 * ShortestPath.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#include "Graph.h"
#include "NodeInfo.h"
#include "PriorityQueue.hpp"
#include <vector>

using namespace std;

#ifndef SRC_SHORTESTPATH_H_
#define SRC_SHORTESTPATH_H_

class ShortestPath {
public:
	ShortestPath(const Graph & g, int sNode, int dNode);
	void findShortestPath();
	virtual ~ShortestPath();
	void printPath();
private:
	Graph m_g;
	PriorityQueue<NodeInfo> m_queue;
	int m_sNode;
	int m_dNode;
	vector<NodeInfo> m_visitedNodes;

	bool isInVisitedNodes(int node);
	NodeInfo getNextNode();
	vector<NodeInfo> getUnvisitedNbrs(int node);
	void updateQueue(NodeInfo n);

}
;

#endif /* SRC_SHORTESTPATH_H_ */
