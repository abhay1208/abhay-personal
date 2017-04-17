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
	int totalNodes = m_g.getNumVertices();
	for (unsigned int i = 0; i < totalNodes; i++) {
		m_unvisitedNodes.push_back(i);
	}
}

void ShortestPath::findShortestPath() {
	int currentNode = m_sNode;

	// Add to visited nodes
	m_visitedNodes.push_back(currentNode);

	// Remove the element from unvisited nodes;
	vector<int>::iterator idx = find(m_unvisitedNodes.begin(),
			m_unvisitedNodes.end(), currentNode);
	if (idx != m_unvisitedNodes.end()) {
		m_unvisitedNodes.erase(idx);
	}

	// Get it's neighbours and their cost
	vector<int> adjacentNodes = m_g.getNeighbours(currentNode);

}

ShortestPath::~ShortestPath() {
	// TODO Auto-generated destructor stub
}

