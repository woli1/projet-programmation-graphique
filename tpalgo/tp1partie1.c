#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>

/* TP1 — Partie 1 (matrices d’adjacence, connexité, Floyd-Warshall)
   Version volontairement simple (n=8). */

#define N 8
#define INF (INT_MAX/4)

static void afficher_0_1(const char *titre, int A[N][N])
{
	printf("%s\n", titre);
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) printf("%d ", A[i][j]);
		printf("\n");
	}
}

static void afficher_dist(const char *titre, int D[N][N])
{
	printf("%s\n", titre);
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (D[i][j] >= INF/2) printf("INF ");
			else printf("%d ", D[i][j]);
		}
		printf("\n");
	}
}

/* 1) Matrice d’adjacence aléatoire, densité p */
static void generer_graphe(int M[N][N], double p)
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (i == j) {
				M[i][j] = 0;
			} else {
				double u = (double)rand() / (double)RAND_MAX;
				M[i][j] = (u < p) ? 1 : 0;
			}
		}
	}
}

/* Produit booléen (AND/OR) : C = A ⊗ B */
static void produit_booleen(int A[N][N], int B[N][N], int C[N][N])
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			C[i][j] = 0;
			for (int k = 0; k < N; k++) {
				if (A[i][k] && B[k][j]) {
					C[i][j] = 1;
					break;
				}
			}
		}
	}
}

static int egales(int A[N][N], int B[N][N])
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			if (A[i][j] != B[i][j]) return 0;
	return 1;
}

/* 2) Connexité via puissances successives
   R0 = I ∪ M ; Rt = Rt-1 ∪ (Rt-1 ⊗ M) */
static void connexite_puissances(int M[N][N], int R[N][N])
{
	int RM[N][N];
	int R2[N][N];

	/* R = I ∪ M */
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			R[i][j] = (i == j) ? 1 : M[i][j];
		}
	}

	for (int t = 1; t <= N; t++) {
		produit_booleen(R, M, RM);
		for (int i = 0; i < N; i++)
			for (int j = 0; j < N; j++)
				R2[i][j] = (R[i][j] || RM[i][j]) ? 1 : 0;
		if (egales(R, R2)) break;
		for (int i = 0; i < N; i++)
			for (int j = 0; j < N; j++)
				R[i][j] = R2[i][j];
	}
}

/* 3) Floyd-Warshall (connexité) */
static void floyd_connexite(int M[N][N], int R[N][N])
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			R[i][j] = (i == j) ? 1 : M[i][j];

	for (int k = 0; k < N; k++) {
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				if (R[i][j] == 0)
					R[i][j] = (R[i][k] && R[k][j]) ? 1 : 0;
			}
		}
	}
}

/* 4) Floyd-Warshall (plus courts chemins) + reconstruction simple */
static void floyd_plus_courts_chemins(int D[N][N], int Next[N][N])
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (D[i][j] >= INF/2) Next[i][j] = -1;
			else Next[i][j] = j;
		}
		Next[i][i] = i;
	}

	for (int k = 0; k < N; k++) {
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				if (D[i][k] >= INF/2 || D[k][j] >= INF/2) continue;
				if (D[i][k] + D[k][j] < D[i][j]) {
					D[i][j] = D[i][k] + D[k][j];
					Next[i][j] = Next[i][k];
				}
			}
		}
	}
}

static void afficher_chemin(int Next[N][N], int src, int dst)
{
	if (Next[src][dst] == -1) {
		printf("(aucun chemin)\n");
		return;
	}
	int at = src;
	printf("%d", at);
	while (at != dst) {
		at = Next[at][dst];
		if (at == -1) {
			printf(" -> ?\n");
			return;
		}
		printf(" -> %d", at);
	}
	printf("\n");
}

int main(void)
{
	double p = 0.30; /* densité */
	unsigned int seed = (unsigned int)time(NULL);
	srand(seed);

	int M[N][N];
	int Rpow[N][N];
	int Rfw[N][N];

	printf("TP1 Partie 1 (n=%d, p=%.2f, seed=%u)\n\n", N, p, seed);

	generer_graphe(M, p);
	afficher_0_1("M (matrice d’adjacence)", M);
	printf("\n");

	connexite_puissances(M, Rpow);
	afficher_0_1("Connexité (puissances) : R*", Rpow);

	/* vérif : R* ∪ (R* ⊗ M) == R* */
	int RM[N][N];
	int Rcheck[N][N];
	produit_booleen(Rpow, M, RM);
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			Rcheck[i][j] = (Rpow[i][j] || RM[i][j]) ? 1 : 0;
	printf("Stabilité (R* == R* ∪ (R* ⊗ M)) : %s\n\n", egales(Rpow, Rcheck) ? "OK" : "NON");

	floyd_connexite(M, Rfw);
	afficher_0_1("Connexité (Floyd-Warshall) : R_fw", Rfw);
	printf("Comparaison R* et R_fw : %s\n\n", egales(Rpow, Rfw) ? "IDENTIQUES" : "DIFFÉRENTES");

	/* plus courts chemins : on transforme M en matrice de poids */
	int D[N][N];
	int Next[N][N];
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (i == j) D[i][j] = 0;
			else if (M[i][j]) D[i][j] = 1 + (rand() % 9);
			else D[i][j] = INF;
		}
	}
	afficher_dist("Poids initiaux (INF = pas d’arête)", D);
	floyd_plus_courts_chemins(D, Next);
	printf("\n");
	afficher_dist("Distances minimales (Floyd-Warshall)", D);

	printf("\nExemple chemin 0 -> %d : ", N - 1);
	afficher_chemin(Next, 0, N - 1);

	return 0;
}
