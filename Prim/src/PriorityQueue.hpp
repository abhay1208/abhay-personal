#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <iostream>
#include <vector>

template<class T>
class PriorityQueue {
	/*
	 * Generic Min heap implementation.
	 */
public:
	PriorityQueue() = default;
	PriorityQueue(const std::vector<T>& data);
	PriorityQueue(const T data[], int length);

	T pop();
	void push(T element);
	void changeElement(T element, const T& newElement);
	void deleteElement(T element);
	int getTotalElements();
	int getElementIndex(T element);
	T getElement(int index);
	void printQueue();

private:
	std::vector<T> m_queue;
	std::vector<unsigned int> getChildrenIndex(int index);
	int getParentIndex(int index);
	bool isRoot(int index);
	void heapify();
	void bubbleup(int index);
	void sinkDown(int index = 0);
	void swapElements(int index1, int index2);

}
;

template<typename T>
PriorityQueue<T>::PriorityQueue(const std::vector<T>& data) :
		m_queue(data) {
	heapify();
}

template<typename T>
PriorityQueue<T>::PriorityQueue(const T data[], int length) {
	for (unsigned int i = 0; i < length; i++) {
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
void PriorityQueue<T>::push(T element) {
	m_queue.push_back(element);

	if (m_queue.empty()) {
		return; //only node
	}
	//else bubble up the last node
	bubbleup(getTotalElements() - 1);
}

template<typename T>
T PriorityQueue<T>::pop() {
	T element;
	if (m_queue.empty()) {
		std::cout << "Nothing to pop. Queue is empty" << std::endl;
		element = -1;
	}
	// Get the root element
	element = m_queue[0];

	// Add the last element to the root and resize
	m_queue[0] = m_queue[m_queue.size() - 1];
	m_queue.resize(m_queue.size() - 1);

	// Sink down the root element to get the min heap again.
	sinkDown();

	// return the root element
	return element;

}
template<typename T>
void PriorityQueue<T>::changeElement(T element, const T& newElement) {

	int index = getElementIndex(element);

	if (index == -1) {
		return; // This vertex is not there in the priority queue.
	}

	m_queue[index] = newElement;
	int parentIndex = getParentIndex(index);

	if (isRoot(index)) {
		sinkDown(index);
		return;
	}

	if (m_queue[parentIndex] > m_queue[index]) {
		bubbleup(index);
	} else {
		sinkDown(index);
	}

}

template<typename T>
void PriorityQueue<T>::deleteElement(T element) {
	int index = getElementIndex(element);
	if (index == -1) {
		return; //this vertex does not exist in the queue.
	}

	// Fill up the node with the last node in the queue and either bubble up or sink down.

	m_queue[index] = m_queue[m_queue.size() - 1];
	m_queue.resize(m_queue.size() - 1);

	int parentIndex = getParentIndex(index);

	if (isRoot(index)) {
		sinkDown(index);
		return;
	}

	if (m_queue[parentIndex] > m_queue[index]) {
		bubbleup(index);
	} else {
		sinkDown(index);
	}

}

template<typename T>
int PriorityQueue<T>::getParentIndex(int index) {
	int parentIndex;
	if (isRoot(index)) {
		parentIndex = index; // returns the same index if it's the root element
	}
	//else
	parentIndex = static_cast<int>((index - 1) / 2);
	return parentIndex;
}

template<typename T>
bool PriorityQueue<T>::isRoot(int index) {
	return index == 0;
}

template<typename T>
std::vector<unsigned int> PriorityQueue<T>::getChildrenIndex(int index) {
	std::vector<unsigned int> childrenIndex;

	if ((2 * index + 1) < m_queue.size()) {
		childrenIndex.push_back(2 * index + 1);
	}

	if ((2 * index + 2) < m_queue.size()) {
		childrenIndex.push_back(2 * index + 2);
	}

	return childrenIndex;

}

template<typename T>
int PriorityQueue<T>::getTotalElements() {
	return m_queue.size();
}

template<typename T>
int PriorityQueue<T>::getElementIndex(T element) {
	// returns -1 if element not found
	int index = -1;
	for (int i = 0; i < m_queue.size(); i++) {
		if (m_queue[i] == element) {
			index = i;
			break;
		}
	}

	// else
	return index;
}

template<typename T>
void PriorityQueue<T>::bubbleup(int index) {
	bool isMinHeap = false;
	int currentIndex = index;

	while (!isMinHeap) {
		if (isRoot(currentIndex)) {
			isMinHeap = true;
			return; // It's the root node.
		}

		// else
		int parentIndex = getParentIndex(currentIndex);

		if (m_queue[parentIndex] > m_queue[currentIndex]) {
			swapElements(currentIndex, parentIndex);
			currentIndex = parentIndex;
		} else {
			isMinHeap = true;
		}

	}
}

template<typename T>
void PriorityQueue<T>::sinkDown(int index) {
	int currentIndex = index;
	bool isMinHeap = false;
	while (!isMinHeap) {
		std::vector<unsigned int> childrenIndex = getChildrenIndex(
				currentIndex);
		int NumChildren = childrenIndex.size();
		switch (NumChildren) {
		case 0:
			isMinHeap = true; // It's a leaf. No need to sink down any further.
			break;

		case 1: // 1 children

			if (m_queue[currentIndex] > m_queue[childrenIndex[0]]) {
				swapElements(currentIndex, childrenIndex[0]);
				currentIndex = childrenIndex[0];
			} else {
				isMinHeap = true;
			}
			break;

		case 2:
			// Get the smaller children first.
			int smallerChildrenIndex =
					(m_queue[childrenIndex[0]] < m_queue[childrenIndex[1]]) ?
							childrenIndex[0] : childrenIndex[1];
			if (m_queue[currentIndex] > m_queue[smallerChildrenIndex]) {
				swapElements(currentIndex, smallerChildrenIndex);
				currentIndex = smallerChildrenIndex;
			} else {
				isMinHeap = true;
			}
			break;
		}
	}
}

template<typename T>
void PriorityQueue<T>::swapElements(int index1, int index2) {

	T temp = m_queue[index1];
	m_queue[index1] = m_queue[index2];
	m_queue[index2] = temp;
}

template<typename T>
T PriorityQueue<T>::getElement(int index) {
	return m_queue[index];
}

template<typename T>
void PriorityQueue<T>::printQueue() {
	for (typename std::vector<T>::iterator it = m_queue.begin();
			it != m_queue.end(); it++) {
		std::cout << *it << std::endl;
	}
}
#endif //
