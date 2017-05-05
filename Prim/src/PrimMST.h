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
class PrimMST {
public:
	PrimMST(const Graph & g);
	int findMST();
	virtual ~PrimMST();
	int getMSTCost();

private:
	Graph m_g;
	PriorityQueue<NodeInfo> m_pq;
	vector<NodeInfo> m_visitedNodes;

	bool isInVisitedNodes(int node);
	NodeInfo getNextNode();
	vector<NodeInfo> getUnvisitedNbrs(int node);
	void updateQueue(NodeInfo n);
	void printMST();

}
;

#endif /* SRC_SHORTESTPATH_H_ */
