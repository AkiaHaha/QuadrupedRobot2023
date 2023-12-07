#ifndef OPERATOR_H
#define OPERATOR_H
#include <iostream>
#include <vector>
#include <aris.hpp>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
using namespace aris::dynamic;
using namespace aris::plan;

/// <summary>
/// classify if 2 number is Quotient Odd
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool inline isQuotientOdd(int a, int b);

/// <summary>
/// splitMatrix28 to 2 mttrix of body pose and leg point
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <param name="c"></param>
void inline splitMatrix28(double a[28], double b[16], double c[12]);

/// <summary>
/// print an array in matrix type  of row * col automatically
/// </summary>
/// <param name="m"></param>
/// <param name="n"></param>
/// <param name="a"></param>
void inline show(int8_t m, int8_t n, double* a);
#endif