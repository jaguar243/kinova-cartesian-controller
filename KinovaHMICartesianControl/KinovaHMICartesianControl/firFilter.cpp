#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "CartesianControl.h"

// ------------------------------------------------------------------------------ 
/*! FIR Filter Initialization

This function initializes the FIR filter by searching for a CSV file formatted as

num_weights\n
w1\n
w2\n
.\n
.\n
.\n
wn\n

which is read into the coeffs variable.
*/
// ------------------------------------------------------------------------------
bool FIRFilter::firFloatInit()
{
	memset(insamp, 0, sizeof(insamp));
	// Load Filter coeffs from CSV file
	int len_LUT = 0;
	char delim = '\n';
	std::cout << "Loading Filter coefficients from CSV file..." << std::endl << std::endl;
	FILE *mat = fopen(filename, "r");
	if (mat) {
		std::cout << "File loaded ... reading the length of the arrays...";
		fscanf(mat, "%d%c", &len_LUT, &delim);
		std::cout << "\n len = " << len_LUT << std::endl;
		coeffs = new double[len_LUT];
		std::cout << "reading in the values...";
		double tmp = 0.0;
		for (int i = 0; i < len_LUT; i++) {
			fscanf(mat, "%lf%c", &tmp,&delim);
			coeffs[i] = tmp;
			// std::cout << "LUT, tmp" << *(LUT - 1) << "," << tmp << "\n"; getchar();
		}
		return true;
	}
	else {
		std::cout << "Cannot find Filter MAT file!" << std::endl;
		Sleep(5000);
		return false;
	}
}

// ------------------------------------------------------------------------------ 
/*! FIR Filter Computation

This function computes the output of the FIR Filter for the given content of the
input buffer in three steps:

1) Put new input at the high end of the buffer\n
2) Apply the filter to each input sample\n
3) Shift input samples back in time for next time instant\n

*/
// ------------------------------------------------------------------------------ 
double* FIRFilter::firFloat(double *input, int length)
{
	double acc;     // accumulator for MACs
	double *coeffp; // pointer to coefficients
	double *inputp; // pointer to input samples
	int n;
	int k;
	int filterLength = BUFFER_LEN;
	output = new double[length];

	// put the new samples at the high end of the buffer
	memcpy(&insamp[filterLength - 1], input,
		length * sizeof(double));

	// apply the filter to each input sample
	for (n = 0; n < length; n++) {
		// calculate output n
		coeffp = coeffs;
		inputp = &insamp[filterLength - 1 + n];
		acc = 0;
		for (k = 0; k < filterLength; k++) {
			acc += (*coeffp++) * (*inputp--);
		}
		output[n] = acc;
	}
	// shift input samples back in time for next time
	memmove(&insamp[0], &insamp[length],
		(filterLength - 1) * sizeof(double));
	memmove(&insamp[0], &insamp[length],
		(filterLength - 1) * sizeof(double));
	return output;
}