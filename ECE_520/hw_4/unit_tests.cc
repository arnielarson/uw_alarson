#include <math.h>
#include <float.h> /* defines DBL_EPSILON */
#include <assert.h>
#include "typed_array.h"
#include "complex.h"
#include "gtest/gtest.h"

namespace {

    /* EE520 HW4 - Tests 
       [*]  push
       [*]  push_front
       [*]  pop
       [*]  pop_front
    */
    TEST(TypedArray, Push) {
        TypedArray<double> b;
        b.push(1.0);
        b.push(2.0);
        //printf("In Test: Push, 1: %f, 2: %f\n", b.get(0), b.get(1));
        EXPECT_DOUBLE_EQ(b.get(0), 1.0);
        EXPECT_DOUBLE_EQ(b.get(1), 2.0);
    }
    TEST(TypedArray, PushFront) {
        TypedArray<double> b;
        b.push_front(1.0);
        b.push_front(2.0);
        b.push_front(3.0);
        //printf("In Test: PushFront, 1: %f, 2: %f, 3: %f\n", b.get(0), b.get(1), b.get(2));
        EXPECT_DOUBLE_EQ(b.get(0), 3.0);
        EXPECT_DOUBLE_EQ(b.get(1), 2.0);
        EXPECT_DOUBLE_EQ(b.get(2), 1.0);
    }
    TEST(TypedArray, Pop) {
        TypedArray<double> b;
        b.push(1.0);
        b.push(2.0);
        double x = b.pop();
        EXPECT_EQ(b.size(), 1);
        EXPECT_DOUBLE_EQ(x, 2.0);
    }
    TEST(TypedArray, PopFront) {
        TypedArray<double> b;
        b.push(1.0);
        b.push(2.0);
        double x = b.pop_front();
        EXPECT_EQ(b.size(), 1);
        EXPECT_DOUBLE_EQ(x, 1.0);
    }
     /* EE520 HW4 - Tests 
       [*]  concat
       [*]  reverse
       [*]  +
       [ *]  complex ...
    */
   TEST(TypedArray, Concat) {
        TypedArray<double> a;
        a.push(1.0);
        a.push(2.0);
        TypedArray<double> b;
        b.push(3.0);
        b.push(4.0);
        TypedArray<double> c = a.concat(b);
        EXPECT_EQ(c.size(), 4);
        EXPECT_DOUBLE_EQ(c.get(0), 1.0);
        EXPECT_DOUBLE_EQ(c.get(3), 4.0);

        TypedArray<double> d = a.concat(a).concat(a);
        EXPECT_EQ(d.size(), 6);
    }
    

    TEST(TypedArray, Reverse) {
        TypedArray<double> a;
        a.push(1.0);
        a.reverse();
        EXPECT_DOUBLE_EQ(a.get(0), 1.0);
        a.push(2.0);        
        a.reverse();
        EXPECT_DOUBLE_EQ(a.get(0), 2.0);
        EXPECT_DOUBLE_EQ(a.get(1), 1.0);
        a.reverse();
        EXPECT_DOUBLE_EQ(a.get(0), 1.0);
        EXPECT_DOUBLE_EQ(a.get(1), 2.0);

    }
    
   TEST(TypedArray, PlusOverload) {
        TypedArray<double> a;
        a.push(1.0);
        a.push(2.0);
        TypedArray<double> b;
        b.push(3.0);
        b.push(4.0);
        TypedArray<double> c = a+b;
        EXPECT_EQ(c.size(), 4);
        EXPECT_DOUBLE_EQ(c.get(0), 1.0);
        EXPECT_DOUBLE_EQ(c.get(3), 4.0);
        TypedArray<double> d = a+b+c;
        EXPECT_DOUBLE_EQ(d.get(0), 1.0);
        EXPECT_EQ(d.size(), 8);
    }

    // Tests - conjugate, operators..
    TEST(Complex, Complex) {
        Complex a = Complex(1.0,1.0);
        ASSERT_EQ(a==a, true);
        Complex b = a * a; // (1 + i)*(1 + i) = 0 + 2i
        Complex c = a + a; // (1 + i)+(1 + i) = 2 + 2i
        EXPECT_DOUBLE_EQ(b.real(), 0.0);
        EXPECT_DOUBLE_EQ(b.imaginary(), 2.0);
        EXPECT_DOUBLE_EQ(c.real(), 2.0);
        EXPECT_DOUBLE_EQ(c.imaginary(), 2.0);
        
        Complex d = a.conjugate() * a;
        EXPECT_DOUBLE_EQ(d.magnitude(), 2.0);
        EXPECT_DOUBLE_EQ(d.real(), 2.0);

        printf("Value of b=a*a:  b.re: %f, b.im: %f\n", b.real(), b.imaginary());
        printf("Value of c=a+a:  c.re: %f, c.im: %f\n", c.real(), c.imaginary());


    }
    /* End EE520 HW4 tests */

    TEST(TypedArray, Construction) {
        TypedArray<double> b;
        b.set(0, 1.0);
        b.set(1, 2.0);
        EXPECT_DOUBLE_EQ(b.get(0), 1.0);
        EXPECT_DOUBLE_EQ(b.get(1), 2.0);
    }

   

    TEST(TypedArray, Matrix) {

        TypedArray<TypedArray<double>> m;

        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                m.get(i).set(j,3*i+j);
            }
        }

        std::cout << m << "\n"; 

        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                EXPECT_DOUBLE_EQ(m.get(i).get(j),3*i+j);
            }
        }

    }


}