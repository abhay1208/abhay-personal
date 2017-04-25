/*
 * ShortestPath.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#ifndef SRC_SHORTESTPATH_H_
#define SRC_SHORTESTPATH_H_

#include "Graph.h"
#include "NodeInfo.h"
#include "PriorityQueue.hpp"
#include <vector>

using namespace std;
class ShortestPath {
public:
	ShortestPath(const Graph & g);
	vector<int> findShortestPath(int sNode, int dNode);
	virtual ~ShortestPath();
	int getPathCost();
private:
	Graph m_g;
	PriorityQueue<NodeInfo> m_pq;
	int m_sNode;
	int m_dNode;
	vector<NodeInfo> m_visitedNodes;

	bool isInVisitedNodes(int node);
	NodeInfo getNextNode();
	vector<NodeInfo> getUnvisitedNbrs(int node);
	void updateQueue(NodeInfo n);
	vector<int> getShortestPath();
	void printPath(vector<int> path);

}
;

#endif /* SRC_SHORTESTPATH_H_ */
