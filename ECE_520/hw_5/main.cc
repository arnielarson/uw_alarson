#include <iostream>
#include <utilities.h>
#include <typed_array.h>
#include <math.h>
#include <map>
#include "gtest/gtest.h"

using namespace std;


GTEST_API_ int main(int argc, char **argv) {
  cout << "Running Test suite" << endl;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


/*
int main(void) {

    // Looking at sort method
    cout << "hello" << endl;
    vector<double> v1 = {1.0, -7.0, 2.3, 0.2, 4.5, 2.1, -5.0};
    for (const double d : v1) {
        cout << d << " ";
    }
    cout << endl;
    sort_by_magnitude(v1);
    //sort(v1.begin(), v1.end());
    for (const double d : v1) {
        cout << d << " ";
    }
    cout << endl;


    // looking at file io - read a file
    //TypedArray<TypedArray<double>> t = read_matrix_csv("in.txt");
    //t = read_matrix_csv("in2.txt");

    map<string, int> m = occurrence_map("sp2.txt");
    for ( map<string, int>::iterator e=m.begin(); e!=m.end(); ++e) {
        cout << e->first << " : " << e->second << endl;
    }
   
}
*/
