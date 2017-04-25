/*
 * VertexInfo.h
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */
#ifndef SRC_NodeINFO_H_
#define SRC_NodeINFO_H_

#include <iostream>
class NodeInfo {
	/*
	 * This class is meant to store two pieces of information for each node.
	 * Node's parent and the cost associated with the node. For Dijkstra's
	 * algorithm, node's parent is updated with it's parent at every iteration
	 * And the cost is updated as the distance from source node to this node.
	 */
public:
	NodeInfo() = default;
	NodeInfo(int node);
	NodeInfo(int node, int parent);

	// Getters

	int getNode() const {
		return m_node;
	}
	int getParent() const {
		return m_parent;
	}
	int getCost() const {
		return m_cost;
	}

	// Setters
	void setParent(int parent) {
		m_parent = parent;
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
	int m_parent;
	int m_cost;
}
;

#endif /* SRC_NodeINFO_H_ */
