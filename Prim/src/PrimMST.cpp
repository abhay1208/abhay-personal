/*
 * PrimMST.cpp
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */

#include "PrimMST.h"
#include <vector>
#include <algorithm>
using namespace std;

PrimMST::PrimMST(const Graph & g) :
		m_g(g) {
	int totalNodes = m_g.getNumNodes();

	for (int i = 0; i < totalNodes; i++) {
		m_pq.push(NodeInfo(i));
	}
}

int PrimMST::findMST() {
	bool MSTFound = false;
	// Starting with node 0
	NodeInfo currentNode(0);
	currentNode.setCost(0);
	m_pq.deleteElement(currentNode);
	m_visitedNodes.push_back(currentNode);
	while (!MSTFound) {

		vector < NodeInfo > unvN = getUnvisitedNbrs(currentNode.getNode());

		// Get it's neighbors and their cost

		for (int i = 0; i < unvN.size(); i++) {
			int cost = m_g.getEdgeValue(currentNode.getNode(),
					unvN[i].getNode());
			unvN[i].setParent(currentNode.getNode());
			unvN[i].setCost(cost);
			updateQueue (unvN[i]);
		}

		if (m_pq.getTotalElements() == 0) {
			MSTFound = true;
		} else {
			currentNode = getNextNode();
		}
	}

	return getMSTCost();
}

void PrimMST::updateQueue(NodeInfo n) {
	int idx = m_pq.getElementIndex(n);
	NodeInfo qn = m_pq.getElement(idx);
	if (qn.getCost() > n.getCost()) {
		m_pq.changeElement(qn, n);
	}
}

NodeInfo PrimMST::getNextNode() {
	NodeInfo n = m_pq.pop();
	m_visitedNodes.push_back(n);
	return n;
}

vector<NodeInfo> PrimMST::getUnvisitedNbrs(int node) {
	vector<int> n = m_g.getNeighbours(node);
	vector < NodeInfo > un;
	for (int i = 0; i < n.size(); i++) {
		if (!isInVisitedNodes(n[i])) {
			un.push_back(NodeInfo(n[i]));
		}
	}
	return un;
}

bool PrimMST::isInVisitedNodes(int node) {
	for (int j = 0; j < m_visitedNodes.size(); j++) {
		if (m_visitedNodes[j].getNode() == node) {
			return true;
		}
	}
	return false;
}

void PrimMST::printMST() {
	cout << "Edge \t \t" << "Cost" << endl;
	for (int i = 1; i < m_visitedNodes.size(); i++) {
		cout << m_visitedNodes[i].getParent() << "->"
				<< m_visitedNodes[i].getNode() << "\t \t";
		cout << m_visitedNodes[i].getCost() << '\n';
	}
}

int PrimMST::getMSTCost() {
	int cost = 0;
	for (int i = 1; i < m_visitedNodes.size(); i++) {
		cost += m_visitedNodes[i].getCost();
	}
	cout << "Minimum spanning tree cost for the given data file graph is "
			<< cost << endl;
	printMST();
	return cost;
}

PrimMST::~PrimMST() {
// TODO Auto-generated destructor stub
}

