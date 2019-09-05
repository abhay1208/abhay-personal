#include <memory>
#include "Tree.h"
typedef std::shared_ptr<Node> SPtr;

Tree::Tree(const std::vector<int>& v)
{
    constructTree(v);
}

void Tree::constructTree(const std::vector<int>& v)
{
    std::unordered_map<int, std::vector<int> > umap;
    // Create a hashmap with values as keys and a vector of indices of where those keys are present in the vector
    for(int i = 0; i<v.size(); ++i)
    {
        auto it = umap.find(v[i]);
        if (it != umap.end()) //Key already exists, add the new index to the vector of indices
        {
            auto oldIdx = it->second;
            oldIdx.push_back(i);
            umap[v[i]] = oldIdx;
        } else 
        {
            std::vector<int> temp(1, i);
            umap.insert(make_pair(v[i], temp));
        }
    }

    // Using the hashmap now to construct the tree.
    // Root is reprsented by -1

    int rootValue = umap[-1][0];
    mp_root = std::make_shared<Node> (rootValue);

    std::queue<SPtr> q;
    q.push(mp_root);

    while (!q.empty())
    {
        SPtr currentNode = q.front();
        q.pop();
        std::vector<int> children = umap[currentNode->m_val];

        if (children.size() >= 1)
        {
            currentNode->mp_left = std::make_shared<Node> (children[0]);
            q.push(currentNode->mp_left);
        }

        if(children.size() == 2)
        {
            currentNode->mp_right = std::make_shared<Node> (children[1]);
            q.push(currentNode->mp_right);
        }

    }
}

void Tree::printLevelOrder()

{
    std::queue<SPtr> q;
    if(mp_root == nullptr)
    {
        std::cout << "Tree is empty\n";
        return;
    }
    q.push(mp_root);

    while(!q.empty())
    {
        SPtr currentNode = q.front();
        q.pop();
        if(currentNode->mp_left != nullptr)
        {
            q.push(currentNode->mp_left);
        }

        if(currentNode->mp_right != nullptr)
        {
            q.push(currentNode->mp_right);
        }

        std::cout << currentNode->m_val << " ";
    }

    std::cout << std::endl;
}

Tree::~Tree()
{
}