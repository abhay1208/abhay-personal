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

ShortestPath::ShortestPath(const Graph & g) :
		m_g(g) {
	int totalNodes = m_g.getNumNodes();

	for (int i = 0; i < totalNodes; i++) {
		m_pq.push(NodeInfo(i));
	}
}

vector<int> ShortestPath::findShortestPath(int sNode, int dNode) {
	m_sNode = sNode;
	m_dNode = dNode;
	bool pathFound = false;
	// Do this for the first source node.
	NodeInfo currentNode(m_sNode);
	currentNode.setCost(0);
	m_pq.deleteElement(currentNode);
	m_visitedNodes.push_back(currentNode);
	while (!pathFound) {

		vector < NodeInfo > unvN = getUnvisitedNbrs(currentNode.getNode());

		// Get it's neighbors and their cost

		for (int i = 0; i < unvN.size(); i++) {
			int cost = m_g.getEdgeValue(currentNode.getNode(),
					unvN[i].getNode());
			unvN[i].setParent(currentNode.getNode());
			unvN[i].setCost(currentNode.getCost() + cost);
			updateQueue (unvN[i]);
		}

		if (isInVisitedNodes (m_dNode)) {
			pathFound = true;
		} else {
			currentNode = getNextNode();
		}
	}
	return getShortestPath();;
}

void ShortestPath::updateQueue(NodeInfo n) {
	int idx = m_pq.getElementIndex(n);
	NodeInfo qn = m_pq.getElement(idx);
	if (qn.getCost() > n.getCost()) {
		m_pq.changeElement(qn, n);
	}
}

NodeInfo ShortestPath::getNextNode() {
	NodeInfo n = m_pq.pop();
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

vector<int> ShortestPath::getShortestPath() {

	// Trace back the path.
	int currentNode = m_dNode;
	vector<int> path;
	while (currentNode != m_sNode) {
		for (int j = 0; j < m_visitedNodes.size(); j++) {
			if (m_visitedNodes[j].getNode() == currentNode) {
				path.push_back(m_visitedNodes[j].getNode());
				if (m_visitedNodes[j].getNode()
						== m_visitedNodes[j].getParent()) {
					cout << "There is no path between Node " << m_sNode
							<< " to " << m_dNode << endl;
					return {}; // return an empty vector
				} else {
					currentNode = m_visitedNodes[j].getParent();
				}
			}
		}
	}
	path.push_back(m_sNode);
	std::reverse(path.begin(), path.end());
	printPath(path); // Call this function to print path From Node s to Node d
	return path;
}

void ShortestPath::printPath(vector<int> path) {
	cout << "Shortest path cost from Node " << m_sNode << " to Node " << m_dNode
			<< " is " << getPathCost() << endl;
	for (int i = 0; i < path.size(); i++) {
		cout << path[i];
		if (i != path.size() - 1) {
			cout << "->";
		}
	}
	cout << endl;
}

int ShortestPath::getPathCost() {
// Get the total distance for this path.
	int index;
	for (int j = 0; j < m_visitedNodes.size(); j++) {
		if (m_visitedNodes[j].getNode() == m_dNode) {
			index = j;
			break;
		}
	}

	NodeInfo n = m_visitedNodes[index];
	return n.getCost();
}

ShortestPath::~ShortestPath() {
// TODO Auto-generated destructor stub
}

