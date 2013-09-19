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

void polar2cartesian(float *M_pol, float *theta, int n, float *M_car) {
	for(int i = 0; i < n; i++)
	{
		M_car[i] = M_pol[i]*cos(theta[i]);    //linha 1
		M_car[n+i] = M_pol[i]*sin(theta[i]);  //linha 2
	}
}

void cleanUpData(float *M_pol, float *M_car, int dataWidth, int n, float *Data_l,
				 float *Data_r, int *nl, int *nr) {
	int i_r=0, i_l=0;
	int rangeMax = 4*dataWidth;

	for(int i = 0; i < n; i++)
	{
		if(M_pol[i] != 0 && M_pol[i] <= rangeMax)
		{
			if(M_car[i] >= 0 && M_car[i] <= dataWidth)
			{
				Data_r[i_r] = M_car[i];
				Data_r[n+i_r] = M_car[n + i];
				//Be aware: matrix M_car must be used as vector
				i_r += 1;
			}
			else if(M_car[i] <= 0 && M_car[i] >= -dataWidth)
			{
				Data_l[i_l] = M_car[i];
				Data_l[n+i_l] = M_car[n + i];
				//Be aware: matrix M_car must be used as vector
				i_l += 1;
			}
		}
	}

	*nl = i_l;
	*nr = i_r;

	/*
	if(t == 10)
	{
	        printf("Data_l x with %d pontos\n",nl);
	        printMat1(Data_lx, 1, nl);
	        printf("Data_r x with %d pontos\n",nr);
	        printMat1(Data_rx, 1, nr);
	        return 0;
	}
	printf("Numero de colunas MATL%d \n", *nl);
	printMat1(&Data_l[0], 2, *nl);
	printf("Numero de colunas MATR%d \n", *nr);
	printMat1(&Data_r[0], 2, *nr);
	*/
}

void intersectionPoint(float *l1, float *l2, float *l3) {
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

    float v0 = 0;
    float v1 = 0;

    // coordenada X do ponto de interseccao
    v0 = (((x1*y2-y1*x2)*(x3-x4)-(x3*y4-y3*x4)*(x1-x2))/\
        ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)));

    // coordenada Y do ponto de interseccao
    v1 = (((x1*y2-y1*x2)*(y3-y4)-(x3*y4-y3*x4)*(y1-y2))/\
        ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)));

    // retorna uma reta que varia X de -15 a 15, passando pelo ponto (0,0)
    // e pela interseccao
    l3[0] = -15;
    l3[1] = 15;
    l3[2] = v1/v0*l3[0];
    l3[3] = v1/v0*l3[1];
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
