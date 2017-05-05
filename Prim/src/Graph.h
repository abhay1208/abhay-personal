/*
 * Graph.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Abhay
 */

#ifndef GRAPH_H_
#define GRAPH_H_

/*
 Assignment 2 - Graph implemetaion
 Author - Abhay Gupta
 Date - 04/07/2017
 */

#include <iostream>
#include <vector>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <iterator>

class Graph {
	/*
	 * Graph reprensation using an array.
	 */
public:

	Graph(double density, int maxDistance, int Nodes);
	Graph(const std::vector<std::vector<int> > & g);
	Graph(std::string filename);
	void printGraph();
	std::vector<int> getNeighbours(int v);
	int getNumNodes();
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
	std::vector<std::vector<int> > m_data;

	void createGraph();
	bool prob();
	int generateDistance();
};
#endif
