/*
 * Hex.h
 *
 *  Created on: Jul 30, 2017
 *      Author: Abhay
 */

#ifndef HEX_H_
#define HEX_H_

#include "Graph.h"

class Hex: public Graph {
public:
	Hex(int boardSize);
	void createBoard();
	void printBoard();
	virtual ~Hex();

private:
	int m_boardSize;
	void addEdge(int node1, int node2);
	int cell2node(int i, int j) const;
	std::vector<int> node2cell(int n) const;


};

#endif /* HEX_H_ */
