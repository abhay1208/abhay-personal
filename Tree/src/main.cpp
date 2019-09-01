#include <iostream>
#include "Tree.h"
#include <vector>
using namespace std;
int main()
{
    int arr[] = {1, 5, 5, 2, 2, -1, 3};
    vector<int> input(arr, arr + sizeof(arr) / sizeof(arr[0]));
    Tree bt(input);
    bt.printLevelOrder();



}