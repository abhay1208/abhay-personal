/*
 * Template class for min heap implementation
 * Author - Abhay Gupta
 */

#include <iostream>
#include <vector>
using namespace std;

template<class T>
class PriorityQueue {
public:
	PriorityQueue() {
	}
	;
	PriorityQueue(const vector<T>& data);
	PriorityQueue(const T data[], int length);

	T pop();
	void push(T vertex);
	void changePriority(T vertex);
	void deleteVertex(T vertex);
	int getTotalNodes();
	vector<int> getVertexIndex(T vertex);
	void printQueue();

private:
	vector<T> m_queue;
	vector<int> getChildren(int node);
	vector<int> getParent(int node);
	void heapify();
	void bubbleup(int node);
	void sinkDown(int node = 0);
	void swapNodes(int node1, int node2);

};

template<typename T>
PriorityQueue<T>::PriorityQueue(const vector<T>& data) {
	for (int i = 0; i < data.size(); i++) {
		m_queue.push_back(data[i]);
	}

	heapify();

}

template<typename T>
PriorityQueue<T>::PriorityQueue(const T data[], int length) {
	for (int i = 0; i < length; i++) {
		m_queue.push_back(data[i]);
	}

	heapify();

}

template<typename T>
void PriorityQueue<T>::heapify() {
	for (int i = m_queue.size() - 1; i >= 0; --i) {
		sinkDown(i);
	}
}

template<typename T>
void PriorityQueue<T>::push(T vertex) {
	m_queue.push_back(vertex);

	if (m_queue.empty()) {
		return; //only node
	}
	//else bubble up the last node
	bubbleup(getTotalNodes() - 1);
}

template<typename T>
T PriorityQueue<T>::pop() {
	// Get the root element
	T vertex = m_queue[0];

	// Add the last element to the root and resize
	m_queue[0] = m_queue[m_queue.size() - 1];
	m_queue.resize(m_queue.size() - 1);

	// Sink down the root element to get the min heap again.
	sinkDown();

	// return the root element
	return vertex;

}
template<typename T>
void PriorityQueue<T>::changePriority(T vertex) {

	vector<int> node = getVertexIndex(vertex);

	if (node.empty()) {
		return; // This vertex is not there in the priority queue.
	}
	// else update the cost of this vertex and either bubbleup or sinkDown.
	int currentNode = node[0];
	vector<int> parent = getParent(currentNode);

	if (parent.empty()) {
		sinkDown(currentNode);
		return;
	}

	if (parent[0] > currentNode) {
		bubbleup(currentNode);
	} else {
		sinkDown(currentNode);
	}

}

template<typename T>
void PriorityQueue<T>::deleteVertex(T vertex) {
	vector<int> node = getVertexIndex(vertex);
	if (node.empty()) {
		return; //this vertex does not exist in the queue.
	}

	// Fill up the node with the last node in the queue and either bubble up or sink down.

	int currentNode = node[0];
	m_queue[currentNode] = m_queue[m_queue.size() - 1];
	m_queue.resize(m_queue.size() - 1);

	vector<int> parent = getParent(currentNode);

	if (parent.empty()) {
		sinkDown(currentNode);
		return;
	}

	if (m_queue[parent[0]] > m_queue[currentNode]) {
		bubbleup(currentNode);
	} else {
		sinkDown(currentNode);
	}

}

template<typename T>
vector<int> PriorityQueue<T>::getParent(int node) {
	vector<int> parent;
	if (node == 0) {
		return parent;
	}
	//else
	int temp = static_cast<int>((node - 1) / 2);
	parent.push_back(temp);
	return parent;
}

template<typename T>
vector<int> PriorityQueue<T>::getChildren(int node) {
	vector<int> children;

	if ((2 * node + 1) < m_queue.size()) {
		children.push_back(2 * node + 1);
	}

	if ((2 * node + 2) < m_queue.size()) {
		children.push_back(2 * node + 2);
	}

	return children;

}

template<typename T>
int PriorityQueue<T>::getTotalNodes() {
	return m_queue.size();
}

template<typename T>
vector<int> PriorityQueue<T>::getVertexIndex(T vertex) {
	vector<int> index;
	for (int i = 0; i < m_queue.size(); i++) {
		if (m_queue[i] == vertex) {
			index.push_back(i);
			break;
		}
	}
	return index;
}

template<typename T>
void PriorityQueue<T>::bubbleup(int node) {
	bool isMinHeap = false;
	int currentNode = node; // Start at the last node as that's the new addition.

	while (!isMinHeap) {
		vector<int> parent = getParent(currentNode);
		if (parent.empty()) {
			isMinHeap = true;
			return; // It's the root node.
		}

		if (m_queue[parent[0]] > m_queue[currentNode]) {
			swapNodes(currentNode, parent[0]);
			currentNode = parent[0];
		} else {
			isMinHeap = true;
		}

	}
}

template<typename T>
void PriorityQueue<T>::sinkDown(int node) {
	int currentNode = node;
	bool isMinHeap = false;
	while (!isMinHeap) {
		vector<int> children = getChildren(currentNode);
		int NumChildren = children.size();
		switch (NumChildren) {
		case 0:
			isMinHeap = 1; // Only the root node is left.
			break;

		case 1:
			if (m_queue[currentNode] > m_queue[children[0]]) {
				swapNodes(currentNode, children[0]);
				currentNode = children[0];
			} else {
				isMinHeap = true;
			}
			break;

		case 2:
			// Get the smaller children first.
			int smallerChildren =
					(m_queue[children[0]] < m_queue[children[1]]) ?
							children[0] : children[1];
			if (m_queue[currentNode] > m_queue[smallerChildren]) {
				swapNodes(currentNode, smallerChildren);
				currentNode = smallerChildren;
			} else {
				isMinHeap = true;
			}
			break;
		}
	}
}

template<typename T>
void PriorityQueue<T>::swapNodes(int node1, int node2) {

	T temp = m_queue[node1];
	m_queue[node1] = m_queue[node2];
	m_queue[node2] = temp;
}

template<typename T>
void PriorityQueue<T>::printQueue() {
	for (typename vector<T>::iterator it = m_queue.begin(); it != m_queue.end();
			it++) {
		cout << *it << endl;
	}
}
