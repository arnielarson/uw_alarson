#include "utilities.h"
#include "stdio.h"
#include "gtest/gtest.h"

namespace {

    TEST(HW5,ReadCSV) {

        TypedArray<TypedArray<double>> t = read_matrix_csv("in.txt");
        //cout << "m[0][0]: " << t.get(0).get(0) << endl;
        //cout << "m[1][1]: " << t.get(1).get(1) << endl;
        EXPECT_DOUBLE_EQ(t.get(0).get(0), 1.0);
        EXPECT_DOUBLE_EQ(t.get(0).get(1), 2.0);
        EXPECT_DOUBLE_EQ(t.get(1).get(0), 3.0);
        EXPECT_DOUBLE_EQ(t.get(1).get(1), 4.0);
    }

    TEST(HW5,ReadCSV1) {

        ASSERT_THROW(TypedArray<TypedArray<double>> t = read_matrix_csv("in1.txt"), std::invalid_argument);
        //cout << "m[0][0]: " << t.get(0).get(0) << endl;
        //cout << "m[1][1]: " << t.get(1).get(1) << endl;
       
    }


    // invalid data throws an error
    TEST(HW5,ReadCSV2) {

        ASSERT_THROW(TypedArray<TypedArray<double>> t = read_matrix_csv("in2.txt"), std::invalid_argument);
    }

    TEST(HW5,WriteCSV) {

        TypedArray<TypedArray<double>> t = read_matrix_csv("in.txt");
        string outfile = "out.txt";
        write_matrix_csv(t, outfile);
        TypedArray<TypedArray<double>> out = read_matrix_csv(outfile);
        EXPECT_DOUBLE_EQ(out.get(0).get(0), 1.0);
        EXPECT_DOUBLE_EQ(out.get(0).get(1), 2.0);
        EXPECT_DOUBLE_EQ(out.get(1).get(0), 3.0);
        EXPECT_DOUBLE_EQ(out.get(1).get(1), 4.0);    
        if (remove(outfile.c_str())==0) {
            cout << "file "<< outfile << "removed.\n" << endl;
        }    
    }

    TEST(HW5, WordOccurence) {
        map<string, int> m = occurrence_map("sp2.txt");
        ASSERT_EQ(m["xx"], 1);
        ASSERT_EQ(m["this"], 4);
        ASSERT_EQ(m["This"], 0);
        ASSERT_EQ(m["XX"], 0);
        ASSERT_EQ(m["And"], 0);
        ASSERT_EQ(m["and"], 1);
        ASSERT_EQ(m["123"], 1);   
    }
}