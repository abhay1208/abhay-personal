/*
 * Graph.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: Abhay
 */
#include <fstream>
#include <iterator>
#include <algorithm>
#include "Graph.h"

using namespace std;

Graph::Graph(double density, int maxDistance, int Nodes) :
								m_density(density), m_maxDistance(maxDistance), m_Nodes(Nodes) {
	createGraph();
}

Graph::Graph(const vector<vector<int> > & g) {
	m_data = g;
	vector<int> column = g[0];
	m_Nodes = column.size();
	m_dNode = 0;
	m_sNode = 0;
}

Graph::Graph(std::string filename) {
	ifstream data_file(filename.c_str());
	istream_iterator<int> start(data_file), end;
	m_Nodes = *start;
	std::vector<std::vector<int> > v(m_Nodes, std::vector<int> (m_Nodes)); // Defaults to zero initial value

	while (start != end) {
		v[*++start][*++start] = *++start;
	}
	m_data = v;
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

int Graph::getNumNodes() {
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

/*
 * Calculates shortest path between two nodes using Dijkstra's algorithm
 */

vector<int> Graph::findShortestPath(int sNode, int dNode) {
	// m_pq.clear(); // TO DO
	m_visitedNodes.clear();
	int totalNodes = getNumNodes();
	for (int i = 0; i < totalNodes; i++) {
		m_pq.push(NodeInfo(i));
	}
	cout << "size of m_pq is in Dijk " << m_pq.getTotalElements() << endl;
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

		for (auto &it:unvN) {
			int cost = getEdgeValue(currentNode.getNode(),
					it.getNode());
			it.setParent(currentNode.getNode());
			it.setCost(currentNode.getCost() + cost);
			updateQueue (it);
		}

		if (isInVisitedNodes (m_dNode)) {
			pathFound = true;
		} else {
			currentNode = getNextNode();
		}
	}
	return getShortestPath();
}

/*
 * Calculates Minimum spanning tree for the graph using Prim's algorithm
 */
int Graph::findMST() {
	// Visited nodes need to be reset
	m_visitedNodes.clear();

	int totalNodes = getNumNodes();

	for (int i = 0; i < totalNodes; i++) {
		m_pq.push(NodeInfo(i));
	}
	cout << "size of m_pq is in MST " << m_pq.getTotalElements() << endl;

	bool MSTFound = false;
	// Starting with node 0
	NodeInfo currentNode(0);
	currentNode.setCost(0);
	m_pq.deleteElement(currentNode);
	m_visitedNodes.push_back(currentNode);
	while (!MSTFound) {
		vector < NodeInfo > unvN = getUnvisitedNbrs(currentNode.getNode());

		// Get it's neighbors and their cost

		for (auto &it:unvN) {
			int cost = getEdgeValue(currentNode.getNode(),
					it.getNode());
			it.setParent(currentNode.getNode());
			it.setCost(cost);
			updateQueue(it);
		}

		if (m_pq.getTotalElements() == 0) {
			MSTFound = true;
		} else {
			currentNode = getNextNode();
		}
	}

	return getMSTCost();
}


/*
 * Updates priority queue if the cost of a node changes
 */
void Graph::updateQueue(NodeInfo n) {
	int idx = m_pq.getElementIndex(n);
	NodeInfo qn = m_pq.getElement(idx);
	if (qn.getCost() > n.getCost()) {
		m_pq.changeElement(qn, n);
	}
}

/*
 * Gets the next node from the queue and updates visited nodes
 */

NodeInfo Graph::getNextNode() {
	NodeInfo n = m_pq.pop();
	m_visitedNodes.push_back(n);
	return n;
}

/*
 * Gets unvisited neighbor of a node
 */

vector<NodeInfo> Graph::getUnvisitedNbrs(int node) {
	vector<int> n = getNeighbours(node);
	vector < NodeInfo > un;
	for (auto it:n) {
		if (!isInVisitedNodes(it)) {
			un.push_back(NodeInfo(it));
		}
	}
	return un;
}

/*
 * Check if a node is in visited nodes
 */
bool Graph::isInVisitedNodes(int node) {
	for (auto it:m_visitedNodes) {
		if (it.getNode() == node) {
			return true;
		}
	}
	return false;
}


vector<int> Graph::getShortestPath() {

	// Trace back the path.
	int currentNode = m_dNode;
	vector<int> path;
	while (currentNode != m_sNode) {
		for (auto it:m_visitedNodes) {
			if (it.getNode() == currentNode) {
				path.push_back(it.getNode());
				if (it.getNode()
						== it.getParent()) {
					cout << "There is no path between Node " << m_sNode
							<< " to " << m_dNode << endl;
					return {}; // return an empty vector
				} else {
					currentNode = it.getParent();
				}
			}
		}
	}
	path.push_back(m_sNode);
	std::reverse(path.begin(), path.end());
	printPath(path); // Call this function to print path From Node s to Node d
	return path;
}

void Graph::printPath(vector<int> path) {
	cout << "Shortest path cost from Node " << m_sNode << " to Node " << m_dNode
			<< " is " << getShortestPathCost() << endl;
	for (unsigned int i = 0; i < path.size(); i++) {
		cout << path[i];
		if (i != path.size() - 1) {
			cout << "->";
		}
	}
	cout << endl;
}

int Graph::getShortestPathCost() {
	// Get the total distance for this path.
	int index;
	for (unsigned int j = 0; j < m_visitedNodes.size(); j++) {
		if (m_visitedNodes[j].getNode() == m_dNode) {
			index = j;
			break;
		}
	}

	NodeInfo n = m_visitedNodes[index];
	return n.getCost();
}

void Graph::printMST() {
	cout << "Edge \t \t" << "Cost" << endl;
	for (unsigned int i = 1; i < m_visitedNodes.size(); i++) {
		cout << m_visitedNodes[i].getParent() << "->"
				<< m_visitedNodes[i].getNode() << "\t \t";
		cout << m_visitedNodes[i].getCost() << '\n';
	}
}

int Graph::getMSTCost() {
	int cost = 0;
	for (unsigned int i = 1; i < m_visitedNodes.size(); i++) {
		cost += m_visitedNodes[i].getCost();
	}
	cout << "Minimum spanning tree cost for the given data file graph is "
			<< cost << endl;
	printMST();
	return cost;
}



Graph::~Graph() {

}

