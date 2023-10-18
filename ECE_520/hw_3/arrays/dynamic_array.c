#include "dynamic_array.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* private functions *********************************************************/

/* Position in the buffer of the array element at position index */
static int index_to_offset ( const DynamicArray * da, int index ) {
    return index + da->origin;
}

/* Position of the element at buffer position 'offset' */
static int offset_to_index ( const DynamicArray * da, int offset ) {
    return offset - da->origin;
}

/* Non-zero if and only if offset lies ouside the buffer */
static int out_of_buffer ( DynamicArray * da, int offset ) {
    return offset < 0 || offset >= da->capacity;
}

/* Makes a new buffer that is twice the size of the old buffer,
   copies the old information into the new buffer, and deletes
   the old buffer */
static void extend_buffer ( DynamicArray * da ) {

    double * temp = (double *) calloc ( 2 * da->capacity, sizeof(double) );
    int new_origin = da->capacity - (da->end - da->origin)/2,
           new_end = new_origin + (da->end - da->origin);

    for ( int i=0; i<DynamicArray_size(da); i++ ) {
        temp[new_origin+i] = DynamicArray_get(da,i);
    }

    free(da->buffer);
    da->buffer = temp;

    da->capacity = 2 * da->capacity;
    da->origin = new_origin;
    da->end = new_end;

    return;

}

/* public functions **********************************************************/

/* EE520 Exercises */

double DynamicArray_min ( const DynamicArray * da ) {
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    // get offset, iterate to end, keep track of minimum
    // can't remember if there is a clever way to pop the first value..
    double min = DynamicArray_get(da, 0);

    for ( int i=1; i < DynamicArray_size(da); i++ ) {
      double e = DynamicArray_get(da,i);
      if ( e < min ) {
        min = e;
      }
    }
    return min;
}
double DynamicArray_max ( const DynamicArray * da ) {
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    
    double max = DynamicArray_get(da, 0);

    for ( int i=0; i < DynamicArray_size(da); i++ ) {
      double e = DynamicArray_get(da,i);
      if ( e > max ) {
        max = e;
      }
    }
    return max;
}
double DynamicArray_mean ( const DynamicArray * da ){
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    
    double sum =  DynamicArray_sum ( da );
    int size = DynamicArray_size(da);
    return sum/size;
}
double DynamicArray_median ( const DynamicArray * da ){
    // expecting to implement a sort method??
    // Updated 2/7/2022
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    
    // copy and sort da
    DynamicArray * a = DynamicArray_copy(da);
    int size = DynamicArray_size(a);
    int flips = 1;
    while( flips > 0) {
        flips = 0;
        for( int i = 0; i < size -1; i++) {
            double e1 = DynamicArray_get(a, i);
            double e2 = DynamicArray_get(a, i+1);
            if (e2<e1) {
                DynamicArray_set(a, i, e2);
                DynamicArray_set(a, i+1, e1);
                flips++;
            }
        }        
    }
    // get median
    double median = 0.0;
    if (size % 2 == 0) {
        int m = size/2;
        median = (DynamicArray_get(a, size/2) + DynamicArray_get(a, size/2-1))/2.0;
    } else {
        median = DynamicArray_get(a, size/2);
    }
    DynamicArray_destroy(a);

    return median;
}
double DynamicArray_sum ( const DynamicArray * da ){
    assert(da->buffer != NULL);
    int size = DynamicArray_size(da);
    double sum = 0;
    for ( int i=0; i < size; i++ ) {
      sum += DynamicArray_get(da,i);
    }
    return sum;
}

double DynamicArray_last ( const DynamicArray * da ) {
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    return DynamicArray_get(da, DynamicArray_size(da)-1);
}

double DynamicArray_first ( const DynamicArray * da ) {
    assert(da->buffer != NULL);
    assert(DynamicArray_size(da)!=0);
    return DynamicArray_get(da, 0);
}

DynamicArray * DynamicArray_copy( const DynamicArray * da ) {
    assert(da->buffer != NULL);

    // So - the array to copy could be arbitrarily different,
    // might as well make use of the existing public methods to 
    // simply iterate over da and set new array a
    DynamicArray * a = DynamicArray_new();
    int size = DynamicArray_size(da);
    for(int i = 0; i < size; i++) {
        // call to set is: (array*, index, value)
        DynamicArray_set(a, i, DynamicArray_get(da, i));
    }
    return a;
}

/* Create a new array that goes from start to end by increment */
DynamicArray * DynamicArray_range(double start, double end, double increment) {
    assert(end >= start);
    DynamicArray * da = DynamicArray_new();
    double value = start;
    while ( value <= end ) {
        DynamicArray_push(da, value);
        value += increment;
    }

    return da;
}


/* Create a new array that is the concatenation of two arrays.. */
DynamicArray * DynamicArray_concat( const DynamicArray * da1, const DynamicArray * da2 ) {
    assert(da1->buffer != NULL);
    assert(da2->buffer != NULL);

    // Again, might as well make use of the existing public methods to 
    // simply iterate over da1 and da2 and set new array a
    DynamicArray * da = DynamicArray_new();
    int size1 = DynamicArray_size(da1);
    int size2 = DynamicArray_size(da2);
    int index = 0;
    for(int i = 0; i < size1; i++) {
        // call to set is: (array*, index, value)
        DynamicArray_set(da, index++, DynamicArray_get(da1, i));
    }
    for(int j = 0; j < size2; j++) {
        // call to set is: (array*, index, value)
        DynamicArray_set(da, index++, DynamicArray_get(da2, j));
    }
    return da;
}

/* Create a new array that takes from da 
        if take > 0, then takes from the front
        if take < 0 the takes from the last
*/
DynamicArray * DynamicArray_take(const DynamicArray * da, int take) {
    assert(da->buffer != NULL);
    DynamicArray * a = DynamicArray_new();
    int size = DynamicArray_size(da);
    if (take > 0 ) {

        for (int i=0; i<take; i++) {
            if (i > size - 1) { 
                DynamicArray_set(a, i, 0.0);
            } else {
                DynamicArray_set(a, i, DynamicArray_get(da, i));
            }
        }
    } else {
        // easiest to probably just use the push method
        for (int i = 0; i < -take; i++) {
            if (size-1-i < 0) { 
                DynamicArray_push_front(a, 0.0);
            } else {
                DynamicArray_push_front(a, DynamicArray_get(da, size-1-i));
            }
        }
    }

    return a;
}

/* End EE520 Exercise */

DynamicArray * DynamicArray_new(void) {
    DynamicArray * da = (DynamicArray *) malloc(sizeof(DynamicArray));
    da->capacity = DYNAMIC_ARRAY_INITIAL_CAPACITY;    
    da->buffer = (double *) calloc ( da->capacity, sizeof(double) ); 
    da->origin = da->capacity / 2;
    da->end = da->origin;
    return da;
}

void DynamicArray_destroy(DynamicArray * da) {
    free(da->buffer);
    da->buffer = NULL;
    return;
}

int DynamicArray_size(const DynamicArray * da) {
    assert(da->buffer != NULL);
    return da->end - da->origin;
}

char * DynamicArray_to_string(const DynamicArray * da) {
    assert(da->buffer != NULL);
    char * str = (char *) calloc (20,DynamicArray_size(da)),
         temp[20];
    int j = 1;
    str[0] = '[';
    for ( int i=0; i < DynamicArray_size(da); i++ ) {
        if ( DynamicArray_get(da,i) == 0 ) {
            snprintf ( temp, 20, "0" );
        } else {
            snprintf ( temp, 20, "%.5lf", DynamicArray_get(da,i) ); 
        }
        if ( i < DynamicArray_size(da) - 1 ) {
            sprintf( str + j, "%s,", temp);
            j += strlen(temp) + 1;
        } else {
            sprintf( str + j, "%s", temp);
            j += strlen(temp);
        }

    }
    str[j] = ']';
    return str;
}

void DynamicArray_print_debug_info(const DynamicArray * da) {

    char * s = DynamicArray_to_string(da);
    printf ( "  %s\n", s);
    printf ( "  capacity: %d\n  origin: %d\n  end: %d\n  size: %d\n\n",
      da->capacity, 
      da->origin, 
      da->end,
      DynamicArray_size(da));

    free(s);

}

void DynamicArray_set(DynamicArray * da, int index, double value) {
    assert(da->buffer != NULL);
    assert ( index >= 0 );
    while ( out_of_buffer(da, index_to_offset(da, index) ) ) {
        extend_buffer(da);
    }
    da->buffer[index_to_offset(da, index)] = value;
    if ( index >= DynamicArray_size(da) ) {
        da->end = index_to_offset(da,index+1);
    }

}

double DynamicArray_get(const DynamicArray * da, int index) {
    assert(da->buffer != NULL);
    assert ( index >= 0 );
    if (  index >= DynamicArray_size(da) ) {
        return 0;
    } else {
        return da->buffer[index_to_offset(da,index)];
    }
}

void DynamicArray_push(DynamicArray * da, double value ) {
    DynamicArray_set(da, DynamicArray_size(da), value );
}

void DynamicArray_push_front(DynamicArray * da, double value) {
    assert(da->buffer != NULL);
    while ( da->origin == 0 ) {
        extend_buffer(da);
    }
    da->origin--;
    DynamicArray_set(da,0,value);
}

double DynamicArray_pop(DynamicArray * da) {
    assert(DynamicArray_size(da) > 0);
    double value = DynamicArray_get(da, DynamicArray_size(da)-1);
    DynamicArray_set(da, DynamicArray_size(da)-1, 0.0);
    da->end--;
    return value;
}

double DynamicArray_pop_front(DynamicArray * da) {
    assert(DynamicArray_size(da) > 0);
    double value = DynamicArray_get(da, 0);
    da->origin++;
    return value;    
}

DynamicArray * DynamicArray_map(const DynamicArray * da, double (*f) (double)) {
    assert(da->buffer != NULL);
    DynamicArray * result = DynamicArray_new();
    for ( int i=0; i<DynamicArray_size(da); i++ ) {
        DynamicArray_set(result, i, f(DynamicArray_get(da, i)));
    }
    return result;
}

DynamicArray * DynamicArray_subarray(DynamicArray * da, int a, int b) {

  assert(da->buffer != NULL);
  assert(b >= a);

  DynamicArray * result = DynamicArray_new();

  for (int i=a; i<b; i++) {
      DynamicArray_push(result,DynamicArray_get(da, i));
  }

  return result;

}