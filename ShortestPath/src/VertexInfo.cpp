/*
 * VertexInfo.cpp
 *
 *  Created on: Apr 17, 2017
 *      Author: Abhay
 */

#include "VertexInfo.h"
#include <limits.h>
#include <iostream>
using namespace std;

VertexInfo::VertexInfo(int node) :
		m_node(node), m_prevNode(node) {
	m_cost = INT_MAX;
}

bool VertexInfo::operator>(const VertexInfo& other) {
	if (m_cost > other.m_cost) {
		return true;
	}
	// else
	return false;
}

bool VertexInfo::operator<(const VertexInfo& other) {
	if (m_cost < other.m_cost) {
		return true;
	}
	// else
	return false;
}

bool VertexInfo::operator ==(const VertexInfo& other) {
	if (m_cost == other.m_cost) {
		return true;
	}
	// else
	return false;
}

VertexInfo::~VertexInfo() {
	// TODO Auto-generated destructor stub
}

