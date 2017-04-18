/*
 * ShortestPath.cpp
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */

#include "ShortestPath.h"
#include <vector>
#include <algorithm>
using namespace std;

ShortestPath::ShortestPath(const Graph & g, int sNode, int dNode) :
		m_g(g), m_sNode(sNode), m_dNode(dNode) {
	int totalNodes = m_g.getNumNodes();

	for (int i = 0; i < totalNodes; i++) {
		m_queue.push(NodeInfo(i));
	}
}

void ShortestPath::findShortestPath() {
	bool pathFound = false;
	// Do this for the first source node.
	NodeInfo currentNode(m_sNode);
	currentNode.setCost(0);
	m_queue.deleteElement(currentNode);
	m_visitedNodes.push_back(currentNode);
	m_queue.printQueue();
	while (!pathFound) {

		vector < NodeInfo > unvN = getUnvisitedNbrs(currentNode.getNode());

		// Get it's neighbors and their cost

		for (int i = 0; i < unvN.size(); i++) {
			int cost = m_g.getEdgeValue(currentNode.getNode(),
					unvN[i].getNode());
			unvN[i].setPrevNode(currentNode.getNode());
			unvN[i].setCost(currentNode.getCost() + cost);
			updateQueue (unvN[i]);
		}

		if (isInVisitedNodes (m_dNode)) {
			pathFound = true;
			printPath();
		} else {
			currentNode = getNextNode();
		}
	}

}

void ShortestPath::updateQueue(NodeInfo n) {
	int idx = m_queue.getElementIndex(n);
	NodeInfo qn = m_queue.getElement(idx);
	if (qn.getCost() > n.getCost()) {
		m_queue.changePriority(n);
	}
}

NodeInfo ShortestPath::getNextNode() {
	NodeInfo n = m_queue.pop();
	m_visitedNodes.push_back(n);
	return n;
}

vector<NodeInfo> ShortestPath::getUnvisitedNbrs(int node) {
	vector<int> n = m_g.getNeighbours(node);
	vector < NodeInfo > un;
	for (int i = 0; i < n.size(); i++) {
		if (!isInVisitedNodes(n[i])) {
			un.push_back(NodeInfo(n[i]));
		}
	}
	return un;
}

bool ShortestPath::isInVisitedNodes(int node) {
	for (int j = 0; j < m_visitedNodes.size(); j++) {
		if (m_visitedNodes[j].getNode() == node) {
			return true;
		}
	}
	return false;
}

void ShortestPath::printPath() {
	int index;
	for (int j = 0; j < m_visitedNodes.size(); j++) {
		if (m_visitedNodes[j].getNode() == m_dNode) {
			index = j;
			break;
		}
	}

	NodeInfo n = m_visitedNodes[index];
	cout << "Shortest distance from Node  " << m_sNode << " to Node" << m_dNode
			<< " is " << n.getCost() << endl;
}

ShortestPath::~ShortestPath() {
// TODO Auto-generated destructor stub
}

