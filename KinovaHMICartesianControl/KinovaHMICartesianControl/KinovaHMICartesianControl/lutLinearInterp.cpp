#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "CartesianControl.h"

/*! 
Load Lookup table from CSV file

Inputs: 
1. (char*) filename:	Filename of CSV file
2. (char) delim	:	Delimiter for CSV file

Output:
1. (bool)			:	Status of LUT load operation
*/
bool Experiment::load_LUT1D(char *filename, char delim)
{
	LUT = new double[1]; LUT[0] = 0;
	// Load Desired Trajectory
	std::cout << "\n\nLoad LUT...";
	FILE *ddata = fopen(filename, "r");
	if (!ddata) {
		std::cout << "LUT not loaded..." << std::endl;
		Sleep(3000);
		return false;
	}
	else {
		std::cout << "File loaded ... reading the length of the arrays...";
		fscanf(ddata, "%d%c", &len_LUT, &delim);
		std::cout << "\n len = " << len_LUT << std::endl;
		LUT = new double[len_LUT];
		std::cout << "reading in the values...";
		double tmp = 0.0;
		for (int i = 0; i < len_LUT; i++) {
			fscanf(ddata, "%lf%c", &tmp, &delim);
			LUT[i] = tmp * scale_LUT;
			// std::cout << "LUT, tmp" << *(LUT - 1) << "," << tmp << "\n"; getchar();
		}
	}
	return true;
}

/*!
This function linearly interpolates the values in the Lookup table for a given input time instant.

Inputs: 
1. (double)	t		:	current time instant (s)
2. (double*)	ylut	:	Lookup Table
3. (int)		L_lut	:	Length of the Lookup table
4. (double)	Tp_lut	:	Time period of Lookup table (s)
5. (double)	fs_lut	:	Sampling frequency of Lookupt table (Hz)

Output:	
1. (double)			:	output of interpolation
*/
double Experiment::interp_lut(double t, double *y_lut, int L_lut, double Tp_lut, double fs_lut)
{
	double iq = ( std::fmod(t,Tp_lut) / (Tp_lut - (1 / fs_lut)))*((double)L_lut - 1);
	int i0 = std::floor(iq);
	int i1 = std::ceil(iq);

	if (i1 > (L_lut - 1))
		i1 = i0;

	return y_lut[i0] + (iq - (double)i0)*(y_lut[i1] - y_lut[i0]);
}