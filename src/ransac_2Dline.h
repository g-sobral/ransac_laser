#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

int ransac_2Dline(float **data, int n, int maxT, float threshold,
					float *bestModel, int *bestInliers, int verbose);

int randomSelect(float **sel, int nsel, float **data, int *ndata);

int fitModel_line(float *point, float *l, float threshold);

void estimateModel_line(float *l, float **P, int n);
