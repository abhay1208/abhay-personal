/*
 * VertexInfo.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#include <iostream>
#ifndef SRC_VERTEXINFO_H_
#define SRC_VERTEXINFO_H_
using namespace std;
class VertexInfo {
public:
	VertexInfo() {
	}
	;

	VertexInfo(int node);

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
	bool operator>(const VertexInfo& other);
	bool operator<(const VertexInfo& other);
	bool operator ==(const VertexInfo& other);

	friend std::ostream& operator <<(std::ostream& out, const VertexInfo& v) {
		out << v.getNode();
		return out;
	}

	virtual ~VertexInfo();
private:
	int m_node;
	int m_prevNode;
	int m_cost;
}
;

#endif /* SRC_VERTEXINFO_H_ */
