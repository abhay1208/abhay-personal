#include "Node.h"

Node::Node(int _val, Node* _left, Node* _right) : 
m_val(_val), 
mp_left(_left), 
mp_right(_right)
{
}

Node::~Node()
{
    std::cout << "Node destroyer called!\n" ;
}