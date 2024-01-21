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
/// const values
/// </summary>
constexpr double pi = aris::PI;
constexpr double kDefaultMajorLength = 0.1;
constexpr double kDefaultMinorLength = 0.01;
constexpr double kDefaultHeight = 0.1;
constexpr int kTcurvePeriodCount = 900;
constexpr int kFactorThousnad = 0.001;
constexpr int kErr3 = 0.001;
constexpr int kErr4 = 0.0001;
constexpr int kErr5 = 0.00001;
constexpr char kBars10[] = "----------";
constexpr char kBars20[] = "--------------------";
constexpr char kBars40[] = "----------------------------------------";
constexpr char kBars50[] = "--------------------------------------------------";

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
void show(int8_t m, int8_t n, double* a);
#endif