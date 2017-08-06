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
#include "NodeInfo.h"
#include "PriorityQueue.hpp"

class Graph {
	/*
	 * Graph representation using an array.
	 */
public:

	Graph(double density, int maxDistance, int Nodes);
	Graph(const std::vector<std::vector<int> > & g);
	Graph(std::string filename);
	void printGraph();

	/* These public methods are to query/change the graph created using the construtor */
	std::vector<int> getNeighbours(int v);
	int getNumNodes();
	int getNumEdges();
	bool isAdjacent(int node1, int node2);
	virtual void addEdge(int node1, int node2);
	void deleteEdge(int node1, int node2);
	int getEdgeValue(int node1, int node2);
	void setEdgeValue(int node1, int node2, int value);

	/* Method to implement Dijkstra's shortest path algorithm */
	std::vector<int> findShortestPath(int sNode, int dNode);
	int getShortestPathCost();

	/* Method to implement Prim Minimum Spanning Tree algorithm */
	int findMST();

	/* Destructor */
	~Graph();

protected:
	int m_Nodes;
	std::vector<std::vector<int> > m_data;
	double m_density;
	int m_maxDistance;

	/* These private members are useful for shortest path calculation */
	PriorityQueue<NodeInfo> m_pq;
	int m_sNode;
	int m_dNode;
	std::vector<NodeInfo> m_visitedNodes;

	/* These private methods are useful to create a graph of a given density */
	void createGraph();
	bool prob();
	int generateDistance();

	/* These private methods are useful for shortest path calculation */
	bool isInVisitedNodes(int node);
	NodeInfo getNextNode();
	std::vector<NodeInfo> getUnvisitedNbrs(int node);
	void updateQueue(NodeInfo n);
	std::vector<int> getShortestPath();
	void printPath(std::vector<int> path);

	/* Print Minimum Spanning Tree */
	int getMSTCost();
	void printMST();


};
#endif
