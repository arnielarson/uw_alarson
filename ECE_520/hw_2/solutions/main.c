#include <stdio.h>
#include "solutions.h"
#include "gtest/gtest.h"


/*
int main() {
     
    printf("Calling running_total(2), Running Total: %d\n", running_total(2));
    printf("Calling running_total(2), Running Total: %d\n", running_total(2));    
    printf("Calling running_total(2), Running Total: %d\n", running_total(2));    
  
    char c[] = { 'a', 'b', 'c', 'd', '1', '2', '3', '4'};
    printf("Before reverse: %s\n", c);
    char* r = reverse(c, 8);
    printf("Returned array: %s\n", r);
    //free(r);


    reverse_in_place(c, 4);
    char d[] = { 'a', 'b', 'c', 'd', 'e', 'x', 'y'};
    printf("Calling reverse_in_place with %s\n",d);
    reverse_in_place(d, 7);
    printf("After calling reverse_in_place with %s\n",d);

    printf("Num Occurences test array: ");
    int test[] = {1,2,3,4,5,6,1,2,3,1,2,1,1,9};
    for (int i = 0; i<14; i++) {
      printf("%d, ", *(test+i));
    }
    printf("\n");

    printf("Num occurences of 1: %d\n",num_occurences(test, 14, 1));
    printf("Num occurences of 2: %d\n",num_occurences(test, 14, 2));
    printf("Num occurences of 3: %d\n",num_occurences(test, 14, 3));
    printf("Num occurences of -1: %d\n",num_occurences(test, 14, -1));
}
*/
// to execute google tests, use this 

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
