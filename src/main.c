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
	int nl, nr, i;

	float angRad = angDeg*(2*PI/360);
	char buffer[BUFFSIZE];

	float polarM[nCols];
	float theta[nCols];
	float modelL[3];
	float modelR[3];
	float intersec[2];
	float lineL[4];
	float lineR[4];
	float lineTraj[4];
	
	float cartM[2*nCols];
	float dataL[2*nCols];
	float dataR[2*nCols];
	
	float **dR, **dL;

	int inliersL = 0;
	int inliersR = 0;
	
	int ret = 0;

	FILE *data;

	data = fopen(dataPath, "r");
	if(data)
	{
		// printf("Lendo dados do arquivo %s\n",dataPath);
		makeTheta(nCols, angRad, theta);

		int t = 1;
		// reject first line! -- parameters from sensor
		fgets(buffer,BUFFSIZE,data);

		while(!feof(data))
		{
			if(fgets(buffer,BUFFSIZE,data))
			{
				getPoints(buffer, nCols, polarM);
				polar2cartesian(polarM, theta, nCols, cartM);
				
				/*
				for(i = 0; i < 2*nCols; i++)
					printf("%f, ", cartM[i]);
				printf("\n");
				*/
				
				cleanUpData(polarM, cartM, dataWidth, nCols, dataL,
							dataR, &nl, &nr);
				
				dR = malloc(nr * sizeof(float *));
				if(dR == NULL) { perror("out of memory\n"); exit(0); }
				for(i = 0; i < nr; i++)
				{
					dR[i] = malloc(2 * sizeof(float));
					if(dR[i] == NULL) { perror("out of memory\n"); exit(0); }
					dR[i][0] = dataR[i];
					dR[i][1] = dataR[nCols + i];
				}
				
				dL = malloc(nl * sizeof(float *));
				if(dL == NULL) { perror("out of memory\n"); exit(0); }
				for(i = 0; i < nl; i++)
				{
					dL[i] = malloc(2 * sizeof(float));
					if(dL[i] == NULL) { perror("out of memory\n"); exit(0); }
					dL[i][0] = dataL[i];
					dL[i][1] = dataL[nCols + i];
				}
	
				ret = ransac_2Dline(dR, nr, (nr/2)-1, threshold, modelR,
							 &inliersR, 0);

				ret += ransac_2Dline(dL, nl, (nl/2)-1, threshold, modelL,
							 &inliersL, 0);
							 
				for(i = 0; i < nr; i++)
					free(dR[i]);
				free(dR);
				
				for(i = 0; i < nl; i++)
					free(dL[i]);
				free(dL);

				if(ret == 0)
				{
					intersectionPoint(modelL, modelR, intersec);
				    model2line(modelL, lineL);
				    model2line(modelR, lineR);
				    bisectrixLine(lineL, lineR, lineTraj);
				    
				    //printf("\nLinha #%d\n", t);
					//printf("RANSAC left side: %.3f*x + %.3f*y + %.3f = 0; inliers = %d\n", 								modelL[0], modelL[1], modelL[2], inliersL);
					//printf("RANSAC right side: %.3f*x + %.3f*y + %.3f = 0; inliers = %d\n", 								modelR[0], modelR[1], modelR[2], inliersR);
					//printf("Intersection point: [%.3f; %.3f]\n", intersec[0], intersec[1]);
					//printf("Bisectrix line: [(%.3f; %.3f) (%.3f; %.3f)]\n", 							lineTraj[0], lineTraj[2], lineTraj[1], lineTraj[3]);
					printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", \
							modelL[0], modelL[1], modelL[2], \
							modelR[0], modelR[1], modelR[2], \
							lineTraj[0], lineTraj[1], lineTraj[2], lineTraj[3], \
							intersec[0], intersec[1]);
				}
				else
					fprintf(stderr,"linha nao detectada na iteracao %d\n",t);

				t++;
			}
			/*else
			{
                fprintf(stderr,"Problema ao ler o arquivo na linha %d\n",t);
                exit(1);
			}*/
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
