/*
 * Graph.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: Abhay
 */

#include "Graph.h"
using namespace std;

Graph::Graph(double density, int maxDistance, int Nodes) :
		m_density(density), m_maxDistance(maxDistance), m_Nodes(Nodes) {
	createGraph();
}

void Graph::createGraph() {

	/*
	 * Initializing the graph 2-D vector to all zeros first
	 */

	for (int i = 0; i < m_Nodes; i++) {
		vector<int> columnVector(m_Nodes, 0);
		m_data.push_back(columnVector);
	}

	/*
	 * Creating a graph keeping the following three constraints in mind
	 * 1. Diagonal elements are zeros - no loops
	 * 2. Graph 2-d vector should be symmetric as it's undirectional
	 * 3. Edges and cost will be decided using density and max distance
	 *    given to the constructor
	 */

	for (int i = 0; i < m_Nodes; i++) {
		for (int j = 0; j < m_Nodes; j++) {
			if (prob() && (i != j)) {
				m_data[i][j] = generateDistance();
			} else {
				m_data[i][j] = 0;
			}
			m_data[j][i] = m_data[i][j];
		}
	}
}

vector<int> Graph::getNeighbours(int vertex) {
	// Make sure the given vertex doesn't exceed maximum nodes
	vector<int> neighbours;
	if (vertex >= m_Nodes) {
		cout << "The input argument vertex exceeds max number of nodes" << endl;
		return neighbours; // return an empty vector;
	}

	vector<int> row = m_data.at(vertex);
	for (unsigned int i = 0; i < row.size(); i++) {
		if (row[i] != 0) {
			neighbours.push_back(i);
		}
	}
	return neighbours;

}

int Graph::getNumVertices() {
	return m_Nodes;
}

int Graph::getNumEdges() {
	int numEdges = 0;
	vector<vector<int> >::iterator rit;
	vector<int>::iterator cit;

	for (rit = m_data.begin(); rit != m_data.end(); rit++) {
		vector<int> column = *rit;
		for (cit = column.begin(); cit != column.end(); cit++) {
			if (*cit != 0) {
				numEdges++;
			}
		}

	}

	return numEdges;
}

bool Graph::isAdjacent(int node1, int node2) {
	if (m_data[node1][node2] != 0) {
		return true;
	}
	// else
	return false;
}

void Graph::addEdge(int node1, int node2) {
	if (node1 == node2) {
		cout << "Loops not allowed" << endl;
		return;
	}

//else
	m_data[node1][node2] = m_data[node1][node2] = Graph::generateDistance();
}

void Graph::deleteEdge(int node1, int node2) {
	m_data[node1][node2] = m_data[node2][node1] = 0;
}

int Graph::getEdgeValue(int node1, int node2) {
	return m_data[node1][node2];
}

void Graph::setEdgeValue(int node1, int node2, int value) {
	if ((node1 == node2) && value == 0) {
		cout << "Loops not allowed" << endl;
		return;
	}
	m_data[node1][node2] = m_data[node1][node2] = value;
}

/*
 * Prints the graph 2-d vector. Useful in debugging.
 */
void Graph::printGraph() {
	vector<vector<int> >::iterator rit;
	vector<int>::iterator cit;

	for (rit = m_data.begin(); rit != m_data.end(); rit++) {
		vector<int> column = *rit;
		for (cit = column.begin(); cit != column.end(); cit++) {
			cout << *cit << "\t";
		}
		cout << endl;
	}

}

/*
 * Returns true if a random number generated is smaller than the density.
 */
bool Graph::prob() {
	if (rand() % 100 + 1 <= m_density) {
		return true;
	}
// else
	return false;
}

/*
 * Generates a random distance for the given maximum distance.
 */
int Graph::generateDistance() {
	return (rand() % m_maxDistance + 1);
}

Graph::~Graph() {
// TODO Auto-generated destructor stub
}

