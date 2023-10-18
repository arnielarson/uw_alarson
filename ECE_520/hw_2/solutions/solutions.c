#include <stdio.h> 
#include <stdlib.h>
#include "solutions.h"

// use a global varaibel to maintain total
int rt=0;

int running_total(int t) {
    rt+=t;
    return rt;
}


// initial impl with a void return 
// a is a pointer to a char array, l is the expected length
char* reverse(char* a, int l) {
  int i;
  char* b = (char * )calloc(l, sizeof(char));
  int j=0;
  for (i = l-1; i>=0; i--) {
      // each of these are the same
      //*(b+j++)=*(a+i);
      //*(b+j++)=a[i];
      b[j++]=a[i];
  }
  return b;
}

// Same as reverse(char* a, int l) but with int
int* reverse(int* a, int l) {
  int i;
  int* b = (int * )calloc(l, sizeof(int));
  int j=0;
  for (i = l-1; i>=0; i--) {
      // each of these are the same
      //*(b+j++)=*(a+i);
      //*(b+j++)=a[i];
      b[j++]=a[i];
  }
  return b;
}

// Take in an array, and length, and reverse in place
// should be able to swap in place for all indices less than half
void reverse_in_place(char* a, int l) {

  float half = (float)l/2.0;

  int end = l-1;
  for(int start=0; start<half; start++) {
      // swap
      char t1 = *(a+start);
      a[start] = *(a+end);
      a[end--] = t1;
  }
}

// same as reverse_in_place(int* a, int l) but with int*
void reverse_in_place(int* a, int l) {

  float half = (float)l/2.0;

  int end = l-1;
  for(int start=0; start<half; start++) {
      // swap
      int t1 = *(a+start);
      a[start] = *(a+end);
      a[end--] = t1;
  }
}

int num_occurences(int* a, int l, int value) {
  int occurences = 0;
  for (int i = 0; i<l; i++) {
    if (value == *(a+i)) {
      occurences++;
    }
  }
  return occurences;
}
