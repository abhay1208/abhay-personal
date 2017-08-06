/*
 * Hex.cpp
 *
 *  Created on: Jul 30, 2017
 *      Author: Abhay
 */
#include <vector>
#include "Hex.h"

Hex::Hex(int boardSize): m_boardSize(boardSize), Graph(0,0, boardSize*boardSize) {
}


void Hex::createBoard(){
	/*
	 * Go through each  cell and create edges depending on where the cell is
	 */
	std::vector<int> cell;
	int row, column;

	for(auto i = 0; i < m_Nodes; i++){
		cell = node2cell(i);
		row = cell[0];
		column = cell[1];

		//Inside cells
		if(row != (m_boardSize - 1) && row != 0 &&
				column != (m_boardSize - 1) && column != 0){
			addEdge(cell2node(row,column), cell2node(row,column - 1));
			addEdge(cell2node(row,column), cell2node(row,column + 1));
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row + 1 , column));
			addEdge(cell2node(row,column), cell2node(row + 1, column - 1));
			addEdge(cell2node(row,column), cell2node(row - 1, column + 1));
			continue;
		}


		/*
		 * Corner cells have 2 or 3 connections depending on their location
		 */

		// top left
		if(row == 0 && column == 0){
			addEdge(cell2node(row,column), cell2node(row + 1, column));
			addEdge(cell2node(row,column), cell2node(row, column + 1));
			continue;
		}

		// bottom right
		if(row == (m_boardSize - 1) && column == (m_boardSize - 1)){
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row, column - 1));
			continue;
		}

		// top right
		if(row == 0 && column == (m_boardSize - 1)){
			addEdge(cell2node(row,column), cell2node(row , column - 1));
			addEdge(cell2node(row,column), cell2node(row + 1, column));
			addEdge(cell2node(row,column), cell2node(row + 1, column - 1));
			continue;
		}

		// bottom left
		if(row == (m_boardSize - 1) && column == 0){
			addEdge(cell2node(row,column), cell2node(row , column + 1));
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row - 1, column + 1));
			continue;
		}

		/*
		 * Edge cells have 4 connections
		 */

		// top edge
		if(row == 0){
			addEdge(cell2node(row,column), cell2node(row, column - 1));
			addEdge(cell2node(row,column), cell2node(row, column + 1));
			addEdge(cell2node(row,column), cell2node(row + 1, column - 1));
			addEdge(cell2node(row,column), cell2node(row + 1, column));
			continue;
		}

		// bottom edge
		if(row == m_boardSize - 1){
			addEdge(cell2node(row,column), cell2node(row, column - 1));
			addEdge(cell2node(row,column), cell2node(row, column + 1));
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row - 1, column + 1));
			continue;
		}

		// left edge
		if(column == 0){
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row + 1, column));
			addEdge(cell2node(row,column), cell2node(row - 1, column + 1));
			addEdge(cell2node(row,column), cell2node(row, column + 1));
			continue;
		}

		// right edge
		if(column == m_boardSize - 1){
			addEdge(cell2node(row,column), cell2node(row - 1, column));
			addEdge(cell2node(row,column), cell2node(row + 1, column));
			addEdge(cell2node(row,column), cell2node(row, column - 1));
			addEdge(cell2node(row,column), cell2node(row + 1, column - 1));
			continue;
		}


	}


}

void Hex::addEdge(int node1, int node2) {
	if (node1 == node2) {
		std::cout << "Loops not allowed" << std::endl;
		return;
	}

	//else
	m_data[node1][node2] = m_data[node1][node2] = 1;
}


int Hex::cell2node(int i, int j) const {
	int n = m_boardSize * i + j;
	return n;
}

std::vector<int> Hex::node2cell(int n) const{
	std::vector<int> v;
	if(n>m_Nodes){
		return v; //empty vector
	}
	//else
	int j = n % m_boardSize; //column
	int i = (n-j)/m_boardSize;
	v.push_back(i);
	v.push_back(j);
	return v;
}

Hex::~Hex() {
	// TODO Auto-generated destructor stub
}

