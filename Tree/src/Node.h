#include <iostream>

class Node
{

public:
int m_val;
Node* mp_left;
Node* mp_right;

Node(int _val, Node* _left = NULL, Node* _right = NULL);
~Node();

};

