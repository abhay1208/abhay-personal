/*
 * Graph.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Abhay
 */

#ifndef GRAPH_H_
#define GRAPH_H_
#endif

/*
 Assignment 1 - Graph implemetaion
 Author - Abhay Gupta
 Date - 04/07/2017
 */

#include <iostream>
#include <vector>
#include <time.h>
#include <cstdlib>

using namespace std;

class Graph {
public:

	Graph(double density, int maxDistance, int Nodes = 5);
	void printGraph();
	vector<int> getNeighbours(int v);
	int getNumVertices();
	int getNumEdges();
	bool isAdjacent(int node1, int node2);
	void addEdge(int node1, int node2);
	void deleteEdge(int node1, int node2);
	int getEdgeValue(int node1, int node2);
	void setEdgeValue(int node1, int node2, int value);

	~Graph();

private:
	double m_density;
	int m_maxDistance;
	int m_Nodes;
	vector<vector<int> > m_data;

	void createGraph();
	bool prob();
	int generateDistance();
};
