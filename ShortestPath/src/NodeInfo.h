/*
 * VertexInfo.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#include <iostream>
#ifndef SRC_NodeINFO_H_
#define SRC_NodeINFO_H_
using namespace std;
class NodeInfo {
public:
	NodeInfo() {
	}
	;

	NodeInfo(int node);
	NodeInfo(int node, int prevNode);

	// Getters

	int getNode() const {
		return m_node;
	}
	int getPrevNode() const {
		return m_prevNode;
	}
	int getCost() const {
		return m_cost;
	}

	// Setters
	void setPrevNode(int prevNode) {
		m_prevNode = prevNode;
	}
	void setCost(int cost) {
		m_cost = cost;
	}

	// Operator Overloading
	bool operator>(const NodeInfo& other);
	bool operator<(const NodeInfo& other);
	bool operator ==(const NodeInfo& other);

	friend std::ostream& operator <<(std::ostream& out, const NodeInfo& v) {
		out << v.getNode();
		return out;
	}

	virtual ~NodeInfo();
private:
	int m_node;
	int m_prevNode;
	int m_cost;
}
;

#endif /* SRC_NodeINFO_H_ */
