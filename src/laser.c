#include "laser.h"

void getPoints(char *line, int n, float *m) {
	char *c;
	c = strtok(line," ");
	for(int i = 0; i < 7; i++)
		if(c != NULL)
			c = strtok(NULL," ");

	for(int i = 0; i < n; i++)
	{
		if(c != NULL)
		{
			m[i]= atof(c);
			c = strtok(NULL," ");
		}
	}
}

void printMat(float *m, int l, int c) {
	for(int i = 0; i < l; i++)
	{
		for(int j = 0; j < c; j++)
			printf("%f ",m[i*c + j]);

		printf("\n");
	}
}

void makeTheta(int n, float ang, float *theta) {
	float init, end, inc;
	init = (PI-ang)/2;
	end = (PI+ang)/2;
	inc = (end-init)/(n-1);

	for(int i = 0; i < n; i++)
		theta[i] = init + (i * inc);
}

void polar2cartesian(float *polarM, float *theta, int n, float *cartM) {
	for(int i = 0; i < n; i++)
	{
		cartM[i] = polarM[i]*cos(theta[i]);    //linha 1
		cartM[n+i] = polarM[i]*sin(theta[i]);  //linha 2
	}
}

void cleanUpData(float *polarM, float *cartM, int dataWidth, int n, float *dataL,
				 float *dataR, int *nl, int *nr) {
	int i_r=0, i_l=0;
	int rangeMax = 4*dataWidth;

	for(int i = 0; i < n; i++)
	{
		if(polarM[i] != 0 && polarM[i] <= rangeMax)
		{
			if(cartM[i] >= 0 && cartM[i] <= dataWidth)
			{
				dataR[i_r] = cartM[i];
				dataR[n+i_r] = cartM[n + i];
				//Be aware: matrix cartM must be used as vector
				i_r += 1;
			}
			else if(cartM[i] <= 0 && cartM[i] >= -dataWidth)
			{
				dataL[i_l] = cartM[i];
				dataL[n+i_l] = cartM[n + i];
				//Be aware: matrix cartM must be used as vector
				i_l += 1;
			}
		}
	}

	*nl = i_l;
	*nr = i_r;

	/*
	if(t == 10)
	{
	        printf("dataL x with %d pontos\n",nl);
	        printMat1(dataLx, 1, nl);
	        printf("dataR x with %d pontos\n",nr);
	        printMat1(dataRx, 1, nr);
	        return 0;
	}
	printf("Numero de colunas MATL%d \n", *nl);
	printMat1(&dataL[0], 2, *nl);
	printf("Numero de colunas MATR%d \n", *nr);
	printMat1(&dataR[0], 2, *nr);
	*/
}

void intersectionPoint(float *model1, float *model2, float *point) {
    // linha 1: a1*x + b1*y + c1 = 0, b1 = -1
    // linha 2: a2*x + b2*y + c2 = 0, b2 = -1

    float a1 = model1[0];
    float c1 = model1[2];
    float a2 = model2[0];
    float c2 = model2[2];

    // coordenada X do ponto de interseccao
    point[0] = (c2 - c1)/(a1 - a2);

    // coordenada Y do ponto de interseccao
	point[1] = a1*point[0] + c1;

}

void bisectrixLine(float *l1, float *l2, float *l3) {

    float alfa1, a1, b1, alfa2, a2, b2, a, b, epson = 0.5;

    // linha 1 formada pelos pontos (x1,y1) e (x2,y2)
    // linha 2 formada pelos pontos (x3,y3) e (x4,y4)
    float x1 = l1[0];
    float x2 = l1[1];
    float y1 = l1[2];
    float y2 = l1[3];
    float x3 = l2[0];
    float x4 = l2[1];
    float y3 = l2[2];
    float y4 = l2[3];

    alfa1 = atan2(y2 - y1,x2 - x1);
    a1 = (y2 - y1) / (x2 - x1);
    b1 = y1 - a1 * x1;

    alfa2 = atan2(y4 - y3,x4 - x3);
    a2 = (y4 - y3) / (x4 - x3);
    b2 = y3 - a2 * x3;

    a = tan((alfa1 + alfa2) / 2);

    if(abs(a1 - a2) > epson){
        b = (a - a1) * (b1 - b2) / (a1 - a2) + b1;
    }
    else{
        b = (b1 + b2) / 2;
    }

    l3[0] = -15;
    l3[1] = 15;
    l3[2] = a * l3[0] + b;
    l3[3] = a * l3[1] + b;
}
