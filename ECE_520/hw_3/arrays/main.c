#include <stdio.h>
#include "gtest/gtest.h"
#include "dynamic_array.h"


GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


/* 
// Some basic exploration of the DynamicArray class
int main(void) {
  DynamicArray * da = DynamicArray_new();
  DynamicArray_set(da,0, 3);
  DynamicArray_push_front(da, 7);    
  
  DynamicArray_set(da, 3, 3);  
  printf("Array size %d\n",DynamicArray_size(da));
  printf("ToString: %s\n",DynamicArray_to_string(da));
  printf("Array origin: %d, end: %d, capacity: %d\n", da->origin, da->end, da->capacity);
  printf("Address of buffer: %x\n", da->buffer);
  
  for(int x = 0; x<10; x++) {
    DynamicArray_push_front(da, 7*x);
    printf("Array origin: %d, end: %d, capacity: %d\n", da->origin, da->end, da->capacity);
    printf("Address of buffer: %x\n", da->buffer);
  }
  DynamicArray_destroy(da);
}

*/


/*

int main(void) {
  DynamicArray * da = DynamicArray_new();
  DynamicArray_push(da, 1);
  double m = DynamicArray_median(da);
  printf("Initial size of median is: %f\n", m);
  DynamicArray_push(da, 3);
  DynamicArray_push(da, 2);
  m = DynamicArray_median(da);
  printf("Size of median is: %f\n", m);
  DynamicArray_push(da, 5);
  m = DynamicArray_median(da);
  printf("Size of median now is: %f\n", m);

}
*/