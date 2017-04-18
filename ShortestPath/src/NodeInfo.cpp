/*
 * NodeInfo.cpp
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */

#include "NodeInfo.h"
#include <limits.h>
#include <iostream>
using namespace std;

NodeInfo::NodeInfo(int node) :
		m_node(node), m_prevNode(node) {
	m_cost = INT_MAX;
}

NodeInfo::NodeInfo(int node, int prevNode) :
		m_node(node), m_prevNode(prevNode) {
	m_cost = INT_MAX;
}

bool NodeInfo::operator>(const NodeInfo& other) {
	if (m_cost > other.m_cost) {
		return true;
	}
	// else
	return false;
}

bool NodeInfo::operator<(const NodeInfo& other) {
	if (m_cost < other.m_cost) {
		return true;
	}
	// else
	return false;
}

bool NodeInfo::operator ==(const NodeInfo& other) {
	if (m_node == other.m_node) {
		return true;
	}
	// else
	return false;
}

NodeInfo::~NodeInfo() {
	// TODO Auto-generated destructor stub
}

