#include <vector>
#include <iostream>
#include <unordered_map>
#include <queue>
#include "Node.h"

class Tree
{
public:
/********* Public Members **********/
std::shared_ptr<Node> mp_root;

/********* Public Methods **********/
Tree(const std::vector<int>& v);
void printLevelOrder();
~Tree();





private:
/********* Private Members **********/
int m_height;

/********* Private Methods **********/
void constructTree(const std::vector<int>& v);

};


