#include <math.h>
#include <float.h> /* defines DBL_EPSILON */
#include "dynamic_array.h"
#include "gtest/gtest.h"

#define X 1.2345

namespace {

    TEST(DynamicArray, CreateAndDestroy) {
        DynamicArray * a = DynamicArray_new();
        DynamicArray_destroy(a);
    }

    TEST(DynamicArray, DeathTests) {
        DynamicArray * a = DynamicArray_new();
        ASSERT_DEATH(DynamicArray_pop(a), ".*Assertion.*");
        ASSERT_DEATH(DynamicArray_pop_front(a), ".*Assertion.*");
        DynamicArray_destroy(a);
        ASSERT_DEATH(DynamicArray_size(a), ".*Assertion.*");
    }    

    TEST(DynamicArray, SmallIndex) {
        DynamicArray * da = DynamicArray_new();
        ASSERT_EQ(DynamicArray_size(da),0);
        DynamicArray_set(da, 0, -X);        
        DynamicArray_set(da, 3, X);
        ASSERT_EQ(DynamicArray_size(da),4);
        ASSERT_EQ(DynamicArray_get(da,0), -X);
        ASSERT_EQ(DynamicArray_get(da,3), X);
        DynamicArray_destroy(da);
    }

    TEST(DynamicArray, BigIndex) {
        DynamicArray * da = DynamicArray_new();
        DynamicArray_set(da, 8, X);
        ASSERT_EQ(DynamicArray_get(da,8), X);
        DynamicArray_destroy(da);              
    }

    TEST(DynamicArray, ReallyBig) {
        DynamicArray * da = DynamicArray_new();
        DynamicArray_set(da, 400, X);
        DynamicArray_set(da, 200, X/2);        
        ASSERT_EQ(DynamicArray_get(da,200), X/2);
        ASSERT_EQ(DynamicArray_get(da,400), X);
        DynamicArray_destroy(da);              
    }  

    TEST(DynamicArray, Push) {
        DynamicArray * da = DynamicArray_new();
        double x = 0;
        while ( x < 10 ) {
            DynamicArray_push(da, x);  
            x += 0.25;
        }
        ASSERT_EQ(DynamicArray_size(da),40);
        printf("Push test Intermediate Result: %s\n", 
               DynamicArray_to_string(da));
        while ( DynamicArray_size(da) > 0 ) {
            DynamicArray_pop(da);
        }
        ASSERT_EQ(DynamicArray_size(da),0);
        DynamicArray_destroy(da);          
    }

    TEST(DynamicArray, PushFront) {
        DynamicArray * da = DynamicArray_new();
        double x = 0;
        while ( x < 10 ) {
            DynamicArray_push_front(da, x);  
            x += 0.25;
        }
        ASSERT_EQ(DynamicArray_size(da),40);
        while ( DynamicArray_size(da) > 0 ) {
            DynamicArray_pop_front(da);
        }
        ASSERT_EQ(DynamicArray_size(da),0);
        DynamicArray_destroy(da);          
    } 

    TEST(DynamnicArray,ToString) {
        DynamicArray * da = DynamicArray_new();
        double x = 1.0;
        while ( x <= 5 ) {
            DynamicArray_push(da, x);  
            x += 1.0;
        }
        char * str = DynamicArray_to_string(da);
        printf("ToString Example: %s\n", str);
        ASSERT_STREQ(
            str,
            "[1.00000,2.00000,3.00000,4.00000,5.00000]"
        );
        DynamicArray_destroy(da);
        free(str);
    }

    TEST(DynamicArray, Pop) {
        DynamicArray * da = DynamicArray_new();
        double x;
        DynamicArray_push(da, X);  
        DynamicArray_push(da, X);  
        x = DynamicArray_pop(da);  
        ASSERT_EQ(DynamicArray_size(da),1);
        ASSERT_EQ(x,X);
        ASSERT_EQ(DynamicArray_get(da,1), 0.0);
        DynamicArray_destroy(da);          
    }

    TEST(DynamicArray, Map) {
        DynamicArray * t = DynamicArray_new(),
                     * y;
        double s = 0.0;
        for ( int i=0; i<628; i++ ) {
            DynamicArray_set(t, i, s);
            s = s + 0.1;
        }
        y = DynamicArray_map(t,sin);
        for (int i=0; i<DynamicArray_size(t); i++) {
            ASSERT_NEAR(
                DynamicArray_get(y,i),sin(0.1*i), 0.0001
            );
        }
        DynamicArray_destroy(t);    
        DynamicArray_destroy(y);                    
    }       

    /*
        EE520 - Student exercise section, Part 1

        Tests to Implement:
        [*] Min
        [*] Max
        [*] Mean
        [*] Sum
        [*] Median
    */
   
    TEST(DynamicArray, Min) {

        DynamicArray * da = DynamicArray_new();
        DynamicArray_push(da, X); 
        ASSERT_EQ(DynamicArray_min(da), X);
        DynamicArray_destroy(da); 

        DynamicArray * da2 = DynamicArray_new();
        for(int i = 0; i>-10; i--) {
            DynamicArray_push(da2, (double) i); 
        }        
        ASSERT_EQ(DynamicArray_min(da2), -9.0);
        DynamicArray_destroy(da2); 
    }  
    TEST(DynamicArray, Max) {

        DynamicArray * da = DynamicArray_new();
        DynamicArray_push(da, X); 
        ASSERT_EQ(DynamicArray_max(da), X);
        DynamicArray_destroy(da); 

        DynamicArray * da2 = DynamicArray_new();
        for(int i = 0; i<10; i++) {
            DynamicArray_push(da2, (double) i); 
        }        
        ASSERT_EQ(DynamicArray_max(da2), 9.0);
        DynamicArray_destroy(da2); 
    } 
    TEST(DynamicArray, Sum) {

        DynamicArray * da = DynamicArray_new();
        ASSERT_EQ(DynamicArray_sum(da), 0.0);
        DynamicArray_push(da, 1);
        ASSERT_EQ(DynamicArray_sum(da), 1.0);
        DynamicArray_push(da, 10.0);
        ASSERT_EQ(DynamicArray_sum(da), 11.0); 
        DynamicArray_destroy(da); 

    }
    TEST(DynamicArray, Mean) {

        DynamicArray * da = DynamicArray_new();
        DynamicArray_push(da, X); 
        ASSERT_EQ(DynamicArray_mean(da), X);
        DynamicArray_destroy(da); 

        DynamicArray * da2 = DynamicArray_new();
        for(int i = 1; i<11; i++) {
            DynamicArray_push(da2, (double) i); 
        }        
        ASSERT_EQ(DynamicArray_mean(da2), 5.5);
        DynamicArray_destroy(da2); 
    } 
    TEST(DynamicArray, Median) {


        DynamicArray * da = DynamicArray_new();
      
        DynamicArray_push(da, 1);
        ASSERT_EQ(DynamicArray_median(da), 1.0); 
        
        DynamicArray_push(da, 2);
        DynamicArray_push(da, 4);
        ASSERT_EQ(DynamicArray_median(da), 2.0); 
        DynamicArray_push(da, 3);
        ASSERT_EQ(DynamicArray_median(da), 2.5); 
        DynamicArray_destroy(da); 

    }
    /*
        EE520 Tests for parts 2:

        [*] Last
        [*] First
        [*] Copy
        [*] Range
        [*] Concat
        [ ] Take
    */
    TEST(DynamicArray, FIRST) {

        DynamicArray * da2 = DynamicArray_new();
        for(int i = 1; i<11; i++) {
            DynamicArray_push(da2, (double) i); 
        }        
        ASSERT_EQ(DynamicArray_first(da2), 1.0);
        DynamicArray_destroy(da2); 
    } 
    TEST(DynamicArray, LAST) {

        DynamicArray * da2 = DynamicArray_new();
        for(int i = 1; i<11; i++) {
            DynamicArray_push(da2, (double) i); 
        }        
        ASSERT_EQ(DynamicArray_last(da2), 10.0);
        DynamicArray_destroy(da2); 
    } 
    TEST(DynamicArray, COPY) {

        DynamicArray * da = DynamicArray_new();
        for(int i = 1; i<6; i++) {
            DynamicArray_push(da, (double) i); 
        }        
        DynamicArray * da2 = DynamicArray_copy(da);
        ASSERT_EQ(DynamicArray_last(da2), 5.0);
        ASSERT_EQ(DynamicArray_first(da2), 1.0);
        DynamicArray_pop(da);
        DynamicArray_pop_front(da);
        ASSERT_EQ(DynamicArray_last(da2), 5.0);
        ASSERT_EQ(DynamicArray_first(da2), 1.0);
        DynamicArray_destroy(da); 
        DynamicArray_destroy(da2); 
    } 
    TEST(DynamicArray, RANGE) {

        DynamicArray * da = DynamicArray_range(0, 1, 0.1);
        //printf("%s\n", DynamicArray_to_string(da));
        /* Note - initially this did not pass using ASSERT_EQ */
        ASSERT_NEAR(DynamicArray_get(da, 0), 0.0, 0.0001);
        ASSERT_NEAR(DynamicArray_get(da, 1), 0.1, 0.0001);
        ASSERT_NEAR(DynamicArray_last(da), 1.0, 0.0001);
        
        DynamicArray_destroy(da); 
    } 
    TEST(DynamicArray, CONCAT) {

        DynamicArray * da1 = DynamicArray_range(0, 1, 0.5);
        DynamicArray * da2 = DynamicArray_range(7, 8, 0.5);
        DynamicArray * da3 = DynamicArray_concat(da1, da2);
        //printf("DA1: %s\n", DynamicArray_to_string(da1));
        //printf("DA2: %s\n", DynamicArray_to_string(da2));
        //printf("DA3: %s\n", DynamicArray_to_string(da3));
        
        ASSERT_NEAR(DynamicArray_get(da3, 0), 0.0, 0.0001);
        ASSERT_NEAR(DynamicArray_get(da3, 2), 1.0, 0.0001);
        ASSERT_NEAR(DynamicArray_get(da3, 3), 7.0, 0.0001);
        ASSERT_NEAR(DynamicArray_get(da3, 5), 8.0, 0.0001);

        DynamicArray_destroy(da1); 
        DynamicArray_destroy(da2); 
        DynamicArray_destroy(da3); 
    } 
    TEST(DynamicArray, TAKE) {

        DynamicArray * da = DynamicArray_range(0, 5, 1.0);
        printf("Take_DA_2: %s\n", DynamicArray_to_string(DynamicArray_take(da, 2)));
        printf("Take_DA_1: %s\n", DynamicArray_to_string(DynamicArray_take(da, 1)));
        printf("Take_DA_-2: %s\n", DynamicArray_to_string(DynamicArray_take(da, -2)));
        printf("Take_DA_8: %s\n", DynamicArray_to_string(DynamicArray_take(da, 8)));
        printf("Take_DA_-8: %s\n", DynamicArray_to_string(DynamicArray_take(da, -8)));
        DynamicArray_destroy(da);
    }
}