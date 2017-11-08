//============================================================================
// Name        : Algorithms.cpp
// Author      : agupta
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;
double multiply (double a, double b);
int main() {
	double a = 12345.0;
	double b = 23.0;
	double temp = multiply(a, b);
	cout << "Product is: " << temp << endl;
    cout << "error is: " << a*b - temp << endl;

	return 0;
}
