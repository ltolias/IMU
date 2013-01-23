//matrix.h
//Matrix Operations Library

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
	float *list;
	int rows;
	int cols;
}lt_array;


void transpose(lt_array *array, lt_array *result);
void scalarMult(float scalar, lt_array *array);
void scalarAdd(float scalar, lt_array *array);
void add(lt_array *array1, lt_array *array2);
void product(lt_array *array1, lt_array *array2, lt_array *result);
void inverse(lt_array * array, lt_array *result);