#include "complex.h"
#include <math.h>
#include "gtest/gtest.h"

namespace {

    TEST(Complex, Basics) {
        Complex a = (Complex) { 1, 1 },
                b = (Complex) { 1, -1 };
        EXPECT_EQ(add(a,b).real,2);       
        EXPECT_EQ(add(a,b).im,0);
        EXPECT_EQ(negate(a).real,-1);
        EXPECT_EQ(negate(a).im,-1);
        EXPECT_EQ(multiply(a,b).real,2);
        EXPECT_EQ(multiply(a,b).im,0);
        EXPECT_EQ(magnitude(a),(double)sqrt(2));
    }

}
