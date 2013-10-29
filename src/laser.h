#define PI 3.14159265

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* getPoints - Transfere os pontos contidos em uma string para uma matriz

	Entradas
	line 	: uma string contendo diversos n valores reais separados por espaço
	n		: numero de elementos em "line"

	Saidas
	m		: matriz de numeros reais onde serão armazenados os valores
			  lidos em "line"
*/
void getPoints(char *line, int n, float *m);


/* printMat - Imprime na tela os elementos de uma matriz

	Entradas
	m	 	: matriz a ser exibida
	l		: numero de linhas da matriz
	c		: numero de colunas da matriz
*/
void printMat(float *m, int l, int c);


/* makeTheta - Cria um vetor com os angulos correspondentes a cada ponto do laser

	Entradas
	n	 	: numero de pontos em cada amostra do laser
	ang		: angulo de abertura do laser (em radianos)

	Saidas
	theta	: matriz contendo os valores dos angulos (em radianos)
*/
void makeTheta(int n, float ang, float *theta);


/* polar2cartesian - Converte um vetor de pontos em coordenada polar para cartesiana

	Entradas
	polarM 	: vetor com pontos em coordenada polar (contem apenas a distancia)
	theta 	: vetor com os angulos correspondentes a cada ponto de polarM
	n		: numero de elementos no vetor polarM

	Saidas
	cartM	: matriz contendo os pontos em cordenada cartesiana
			  linha 0 contem os valores para a coordenada x
			  e a linha 1 contem os valores para a coordenada y
*/
void polar2cartesian(float *polarM, float *theta, int n, float *cartM);


/* cleanUpData - Limpa e prepara os dados para a execução do RANSAC

	- elimina os pontos (0,0)
	- elimina pontos que tenham distancia ao ponto (0,0) > 4*dataWidth
	- elimina pontos que tenham distância a linha central > dataWidth
	- separa pontos a direita e esquerda

	Entradas
	polarM 		: pontos do laser em coordenada polar
	cartM		: pontos do laser em coordenada cartesiana
	dataWidth	: largura do corredor
	n			: numero de colunas das matrizes polarM e carM

	Saidas
	dataL		: matriz contendo os pontos a esquerda
	dataR		: matriz contendo os pontos a direita
	nl			: quantidade de pontos a esquerda
	nr			: quantidade de pontos a direita
*/
void cleanUpData(float *polarM, float *cartM, int dataWidth, int n, 
				 float *dataL, float *dataR, int *nl, int *nr);

/* intersectionPoint - calcula o ponto de interseccao de duas linhas representadas
pelo modelo a*x + b*y + c = 0

    Entradas
    model1  : vetor contendo os coeficientes da primeira linha (a1, b1, c1)
    model2  : vetor contendo os coeficientes da primeira linha (a2, b2, c2)

    Saidas
    point  : ponto de interseccao das linhas (x, y)
*/
void intersectionPoint(float *model1, float *model2, float *point);


/* bisectrixLine - calcula a bissetriz entre duas retas

    Entradas
    l1  : vetor contendo os pontos da primeira linha (x1, x2, y1, y2)
    l2  : vetor contendo os pontos da segunda linha (x3, x4, y3, y4)

    Saidas
    l3  : vetor contendo os pontos da bissetriz (x1, x2, y1, y2)
*/
void bisectrixLine(float *l1, float *l2, float *l3);


/* model2line - retorna dois pontos presentes em uma linha representada
pelo modelo a*x + b*y + c = 0

    Entradas
    model  : vetor contendo os coeficientes da linha (a, b, c)

    Saidas
    line  : vetor contendo os pontos da linha (x1, x2, y1, y2)
*/
void model2line(float *model, float *line);
