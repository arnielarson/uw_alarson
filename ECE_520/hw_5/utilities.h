#ifndef UTILITIES
#define UTILITIES

#include <iostream>
#include <vector>
#include <map>
#include "typed_array.h"

using namespace std;

void sort_by_magnitude(vector<double> & v);

TypedArray<TypedArray<double>> read_matrix_csv(const string path);

void write_matrix_csv(const TypedArray<TypedArray<double>> &matrix, const string path);

map<string, int> occurrence_map(const string path);

#endif