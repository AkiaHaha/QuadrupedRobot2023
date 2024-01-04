#include <aris.hpp>
#include "tools/operator.h"
using namespace aris::control;
using namespace aris::dynamic;
using namespace aris::plan;

bool inline isQuotientOdd(int a, int b) {
  int c = a / b;
  return c % 2 != 0;
}
void inline splitMatrix28(double a[28], double b[16], double c[12]) {
  for (int8_t i = 0; i < 16; i++) {
    b[i] = a[i];
  }
  for (int8_t i = 0; i < 12; i++) {
    c[i] = a[i + 16];
  }
}
void show(int8_t m, int8_t n, double* a) {
	std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(6);
	std::cout << std::endl;
	for (int8_t i = 0; i < m; i++) {
		for (int8_t j = 0; j < n; j++) {
			std::cout << a[n * i + j]  << "   ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}