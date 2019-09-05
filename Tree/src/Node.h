#include <iostream>
#include <memory>

class Node
{

public:
int m_val;
std::shared_ptr<Node> mp_left;
std::shared_ptr<Node> mp_right;

Node(int _val);
~Node();

};

