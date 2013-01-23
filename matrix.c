//matrix.h
//Matrix Operations Library


void transpose(lt_array *array, lt_array *result)
{
	int j, k;
	for (j = 0; j < array->rows; j ++)
	{
		for (k = 0; k < array->cols; k ++)
		{
			result->list[k * array->rows + j] = array->list[j * array->cols + k];
		}
	}
	
	result->rows = array->cols;
	result->cols = array->rows;
}

void scalarMult(float scalar, lt_array *array)
{
	int j, k;
	for (j = 0; j < array->rows; j ++)
	{
		for (k = 0; k < array->cols; k ++)
		{
			array->list[j * array->cols + k] *= scalar;
		}
	}
}

void scalarAdd(float scalar, lt_array *array)
{
	int j, k;
	for (j = 0; j < array->rows; j ++)
	{
		for (k = 0; k < array->cols; k ++)
		{
			array->list[j * array->cols + k] += scalar;
		}
	}
}

void add(lt_array *array1, lt_array *array2)
{
	int j, k;
	for(j = 0; j < array1->rows; j++)
	{
		for (k = 0; k < array1->cols; k ++)
		{
			array1->list[j * array1->cols + k] += array2->list[j * array1->cols + k];
		}
	}
}

void product(lt_array *array1, lt_array *array2, lt_array *result)
{
	int j, k, r;
	result->rows = array1->rows;
	result->cols = array2->cols;
	for(j = 0; j < array1->rows; j++)
	{
		for (k = 0; k < array2->cols; k ++)
		{
			float temp = 0;
			for (r = 0; r < array1->cols; r ++)
			{
				temp += array1->list[r + j * array1->cols] * array2->list[r * array2->cols + k];
				
			}
			result->list[j * array2->cols + k] = temp;
		}
	}
}

void inverse(lt_array * array, lt_array *result)
{
	result->rows = array->rows;
	result->cols = array->cols;
	lt_array *temp = malloc(sizeof *temp);
	temp->list = malloc(sizeof (float) * array->rows * array->cols * 2);
	temp->rows = array->rows;
	temp->cols = array->cols * 2;
	int j, k, i;
	for (j = 0; j < array->rows; j ++)
	{
		for (k = 0; k < array->cols * 2; k ++)
		{
			if (k < array->cols)
			{
				temp->list[j * temp->cols + k] = array->list[j * array->cols + k];
			}
			else if ((k - array->cols) == j) temp->list[j * temp->cols + k] = 1;
			else temp->list[j * temp->cols + k] = 0;
		}
	}

	for (j = 0; j < array->rows; j ++)
	{
		for (k = 0; k <= j; k ++)
		{
			float val = temp->list[j * temp->cols + k];
			for (i = k; i < temp->cols; i ++)
			{
				if (k < j) temp->list[j * temp->cols + i] -= temp->list[k * temp->cols + i] * val;
				else temp->list[j * temp->cols + i] /= val;
			}
		}

	}
	
	//weve made it to row echelon form, now to reduce:
	for(j = array->rows - 2; j >= 0; j --)
	{
		for(k = array->cols - 1; k > j; k--)
		{
			float val = temp->list[j * temp->cols + k];
			for(i = j + 1; i < temp->cols; i ++)
			{
				temp->list[j * temp->cols + i] -= temp->list[k * temp->cols + i] * val;
			}
		}
	}
	//made it to reduced row echelon form, now lets copy the inverse into the result
	for(j = 0; j < temp->rows; j ++)
	{
		for(k = 0; k < temp->cols; k ++)
		{
			if(k >= temp->cols/2) 
			{
				//lets check for weird things and if they happen put nice numbers
				if(temp->list[j * temp->cols + k] == -(INFINITY)) result->list[j * result->cols + k - temp->cols/2] = -1 * 2 << 16;
				else if (temp->list[j * temp->cols + k] == INFINITY) result->list[j * result->cols + k - temp->cols/2] = 2 << 16;
				else if (isnan(temp->list[j * temp->cols + k])) result->list[j * result->cols + k - temp->cols/2] = 1;
				else result->list[j * result->cols + k - temp->cols/2] = temp->list[j * temp->cols + k];
			}
				
		}
	}


}

