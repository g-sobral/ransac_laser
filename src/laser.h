#define PI 3.14159265

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* getPoints - Transfere os pontos contidos em uma string para uma matriz

	Entradas
	line 	: uma string contendo diversos pontos com valor numerico
	n		: numero de elementos em "line"

	Saidas
	m		: matriz de elementos float onde serão armazenados os valores
			  lidos na string
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
	M_pol 	: vetor com pontos em coordenada polar (contem apenas a distancia)
	theta 	: vetor com os angulos correspondentes a cada ponto de M_pol
	n		: numero de elementos no vetor M_pol

	Saidas
	M_car	: matriz contendo os pontos em cordenada cartesiana
			  linha 0 contem os valores para a coordenada x
			  e a linha 1 contem os valores para a coordenada y
*/
void polar2cartesian(float *M_pol, float *theta, int n, float *M_car);


/* cleanUpData - Limpa e prepara os dados para a execução do RANSAC

	- elimina os pontos (0,0)
	- elimina pontos que tenham distancia ao ponto (0,0) > 4*dataWidth
	- elimina pontos que tenham distância a linha central > dataWidth
	- separa pontos a direita e esquerda

	Entradas
	M_pol 		: pontos do laser em coordenada polar
	M_car		: pontos do laser em coordenada cartesiana
	dataWidth	: largura do corredor
	n			: numero de colunas das matrizes M_pol e M_car

	Saidas
	Data_lx		: vetor de coordenadas X dos pontos para ransac a esquerda
	Data_ly		: vetor de coordenadas Y dos pontos para ransac a esquerda
	Data_rx		: vetor de coordenadas X dos pontos para ransac a direita
	Data_ry		: vetor de coordenadas Y dos pontos para ransac a direita
	nl			: quantidade de pontos a esquerda
	nr			: quantidade de pontos a direita
*/
void cleanUpData(float *M_pol, float *M_car, int dataWidth, int n, float *Data_l,
				 float *Data_r, int *nl, int *nr);

/* intersectionPoint - calcula o ponto de interseccao de duas linhas, cada uma 
delas
                    definida por dois pontos

    Entradas
    l1  : vetor contendo os pontos da primeira linha (x1, x2, y1, y2)
    l2  : vetor contendo os pontos da segunda linha (x3, x4, y3, y4)

    Saidas
    l3  : retorna uma reta que varia X de -15 a 15, passando pelo ponto (0,0)
        e pela interseccao (x1, x2, y1, y2)
*/
void intersectionPoint(float *l1, float *l2, float *l3);


/* bisectrixLine - calcula a bissetriz entre as duas retas do RANSAC
                (como implementado no ROS)

    Entradas
    l1  : vetor contendo os pontos da primeira linha (x1, x2, y1, y2)
    l2  : vetor contendo os pontos da segunda linha (x3, x4, y3, y4)

    Saidas
    l3  : vetor contendo os pontos da bissetriz (x1, x2, y1, y2)
*/

