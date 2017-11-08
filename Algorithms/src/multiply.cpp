#include <iostream>
#include <cmath>
#include <math.h>

using namespace std;

double multiply (double a, double b);
double getNumDigits(double num);

double getNumDigits(double num){
	double numDigits;
	numDigits = log10(num);
	return static_cast<double>(floor(numDigits) + 1);
}

double multiply(double x,  double y){
	/*
	 * Computes multiplication of two integers recursively - close to Kastrubha algorithm.
	 */
	cout << "a = " << x << " b = " << y << endl;;
	// base case
	if ((x < 10) || (y < 10)){
		return x*y;
	}
	// else
	double m;
	double n = getNumDigits(x);
	if (fmod(n, 2) == 0)
	{
		m = n;
	} else
	{
		m = n+1;
	}

	cout << "m = " << m << endl;
	double temp = pow(10, m/2);
	double a = floor(x/temp);
	double b = x - a * temp;
	double c = floor(y/temp);
	double d = y - c * temp;
	return(pow(10, m) * multiply(a ,c) + temp * (multiply(a, d) + multiply(b, c)) + multiply(b, d));

}
