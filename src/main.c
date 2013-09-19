/*
		Estima a trajetoria através de um corredor utilizando o
	algoritmo RANSAC aplicado a uma nuvem de pontos obtida por
	sensores laser.

	Autor: Gabriel S. Sobral <gasan.sobral@gmail.com>, 2012

	revisao 2.0 - por Renato Martins <renatojmsdh@gmail.com>
		Implementacao das funcoes laserRansacFinal para FPGA
         * com base no codigo original + cálculo de bissetriz;
         * representa matrizes+vetores estaticamente de forma simples

	COMPILAR: $ make all

	EXECUTAR: $ ./ransacFPGA p1 p2 p3 p4 p5 p6

			p1: arquivo contendo os pontos do sensor laser
			p2: numero de pontos por leitura
			p3: angulo de leitura em graus
			p4: largura do corredor
			p5: threshold para o algoritmo RANSAC
			p6: porcentagem de inliers para o algoritmo RANSAC

	EXEMPLO: $ ./ransacFPGA [path2VeroLidarData].log 1081 270 8 1.5 0.55

*/

#include "laser.h"
#include "ransac_2Dline.h"

#define BUFFSIZE 10000

int main(int argc, char **argv)
{

//---- checa os argumentos na linha de comando ----
	if(argc!=6)
	{
		fprintf(stderr,"\n USO: $ %s p1 p2 p3 p4 p5 p6 \n\n"
				" p1: arquivo contendo os pontos do sensor laser \n"
				" p2: [int] numero de pontos por leitura \n"
				" p3: [int] angulo de leitura em graus \n"
				" p4: [int] largura do corredor \n"
				" p5: [float] threshold para o algoritmo RANSAC \n"
				" EXEMPLO: $ %s [path2VeroLidarData].log 1081 270 8 1.5 0.55 \n\n",
				argv[0], argv[0]);
		exit(1);
	}

//---- extrai os valores dos argumentos ----
	char dataPath[100];
	if(sscanf(argv[1],"%s",dataPath)!=1)
	  { fprintf(stderr,"parametro 1 invalido: %s\n",argv[1]); exit(1); }
	int nCols;
	if(sscanf(argv[2],"%d",&nCols)!=1)
	  { fprintf(stderr,"parametro 2 invalido: %s\n",argv[2]); exit(1); }
	int angDeg;
	if(sscanf(argv[3],"%d",&angDeg)!=1)
	  { fprintf(stderr,"parametro 3 invalido: %s\n",argv[3]); exit(1); }
	int dataWidth;
	if(sscanf(argv[4],"%d",&dataWidth)!=1)
	  { fprintf(stderr,"parametro 4 invalido: %s\n",argv[4]); exit(1); }
	float threshold;
	if(sscanf(argv[5],"%f",&threshold)!=1)
	  { fprintf(stderr,"parametro 5 invalido: %s\n",argv[5]); exit(1); }

//---- inicializa variaveis ----
	int nl, nr;
	float angRad = angDeg*(2*PI/360);
	char buffer[BUFFSIZE];

	float polarM[nCols];
	float theta[nCols];
	float cartM[2*nCols];
	float dataL[2*nCols];
	float dataR[2*nCols];
	float lineL[3];
	float lineR[3];
	float lineTraj[4];

	int inliersL = 0;
	int inliersR = 0;

	FILE *data;

	data = fopen(dataPath, "r");
	if(data)
	{
		printf("Lendo dados do arquivo %s\n",dataPath);
		makeTheta(nCols, angRad, theta);

		int t = 1;
		// reject first line! -- parameters from sensor
//		fgets(buffer,BUFFSIZE,data);
		while(!feof(data))
		{
			if(fgets(buffer,BUFFSIZE,data))
			{
				getPoints(buffer, nCols, polarM);
				polar2cartesian(polarM, theta, nCols, cartM);

				cleanUpData(polarM, cartM, dataWidth, nCols, dataL,
							dataR, &nl, &nr);

				ransac_2Dline(dataL, nCols, nl, (nl/2)-1, threshold, lineL,
							 &inliersL, 1);

				ransac_2Dline(dataR, nCols, nr, (nr/2)-1, threshold, lineR,
							 &inliersR, 1);

				if(lineL[0] && lineR[0])
				{
					//intersectionPoint(line_l, line_r, trajectory);
				    //bisectrixLine(line_l, line_r, trajectory);
				}
				else
					fprintf(stderr,"linha nao detectada na iteracao %d\n",t);

				t++;
			}
			else
			{
                fprintf(stderr,"Problema ao ler o arquivo na linha %d\n",t);
                exit(1);
			}
		}
		printf("Fim do arquivo.\n");
	}
	else
	{
		perror("Erro ao abrir o arquivo");
		exit(1);
	}

	return 0;
}
