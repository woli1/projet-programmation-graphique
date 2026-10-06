#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <string.h>

/* TP1 — Partie 2 :
   - Parcours en largeur ("vague")
   - Composantes connexes (graphe non orienté)
   - Plus courts chemins en nombre d'arêtes (BFS)
   - Compteur d'opérations (recherche des voisins)
   - Dijkstra (poids positifs) */

#define MAXN 64
#define INF (INT_MAX/4)

static void afficher_0_1(int n, int A[MAXN][MAXN], const char *titre)
{
	printf("%s\n", titre);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) printf("%d ", A[i][j]);
		printf("\n");
	}
}

static void afficher_vect(int n, int v[MAXN], const char *titre)
{
	printf("%s\n", titre);
	for (int i = 0; i < n; i++) printf("%d ", v[i]);
	printf("\n");
}

static void afficher_dist(int n, int d[MAXN], const char *titre)
{
	printf("%s\n", titre);
	for (int i = 0; i < n; i++) {
		if (d[i] >= INF/2) printf("INF ");
		else printf("%d ", d[i]);
	}
	printf("\n");
}

/* Floyd-Warshall (poids) pour comparer les résultats (exemples du TP1). */
static void floyd_warshall(int n, int D[MAXN][MAXN])
{
	for (int k = 0; k < n; k++) {
		for (int i = 0; i < n; i++) {
			if (D[i][k] >= INF/2) continue;
			for (int j = 0; j < n; j++) {
				if (D[k][j] >= INF/2) continue;
				int cand = D[i][k] + D[k][j];
				if (cand < D[i][j]) D[i][j] = cand;
			}
		}
	}
}

static void init_dist_unweighted(int n, int M[MAXN][MAXN], int D[MAXN][MAXN])
{
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i == j) D[i][j] = 0;
			else if (M[i][j]) D[i][j] = 1;
			else D[i][j] = INF;
		}
	}
}

/* Lecture d'un graphe (annexe) depuis un flux.
   Format supporté (non orienté par défaut):
     n m [S]
     u v            (arête non pondérée, poids implicite 1)
     u v w          (arête pondérée)

   Indices de sommets : 0..n-1.
   Sorties:
     - M : matrice 0/1
     - W : matrice de poids (INF si pas d'arête, 0 sur la diagonale)
     - *src : sommet de départ (S si fourni, sinon 0)
     - *hasWeights : 1 si au moins une arête avec poids explicite
*/
static int lire_graphe(FILE *in, int *n_out, int M[MAXN][MAXN], int W[MAXN][MAXN], int *src, int *hasWeights)
{
	char line[256];
	int n = 0, m = 0;
	int s = 0;
	int got = 0;

	*hasWeights = 0;
	*src = 0;

	/* Lire l'en-tête (ignorer lignes vides/commentaires) */
	while (fgets(line, sizeof line, in)) {
		if (line[0] == '#' || line[0] == '\n') continue;
		int tmpS = 0;
		got = sscanf(line, "%d %d %d", &n, &m, &tmpS);
		if (got >= 2) {
			if (got == 3) s = tmpS;
			break;
		}
	}
	if (got < 2) return 0;
	if (n < 1) return 0;
	if (n > MAXN) {
		fprintf(stderr, "n=%d > MAXN=%d\n", n, MAXN);
		return 0;
	}
	if (m < 0) return 0;
	if (s < 0 || s >= n) s = 0;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			M[i][j] = 0;
			W[i][j] = (i == j) ? 0 : INF;
		}
	}

	int edgesRead = 0;
	while (edgesRead < m && fgets(line, sizeof line, in)) {
		if (line[0] == '#' || line[0] == '\n') continue;
		int u, v, w;
		int c = sscanf(line, "%d %d %d", &u, &v, &w);
		if (c < 2) continue;
		if (c == 2) w = 1;
		else *hasWeights = 1;
		if (u < 0 || u >= n || v < 0 || v >= n) {
			fprintf(stderr, "Arête ignorée (sommet hors bornes): %d %d\n", u, v);
			continue;
		}
		if (u == v) {
			/* pas de boucle utile ici */
			edgesRead++;
			continue;
		}
		if (w < -INF/2 || w > INF/2) {
			fprintf(stderr, "Poids trop grand, arête ignorée: %d %d %d\n", u, v, w);
			edgesRead++;
			continue;
		}
		M[u][v] = 1;
		M[v][u] = 1;
		/* si plusieurs arêtes, on garde le plus petit poids */
		if (w < W[u][v]) {
			W[u][v] = w;
			W[v][u] = w;
		}
		edgesRead++;
	}

	*n_out = n;
	*src = s;
	return 1;
}

/* Graphe non orienté aléatoire de densité p (sans boucles). */
static void graphe_aleatoire_non_oriente(int n, double p, int M[MAXN][MAXN])
{
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			M[i][j] = 0;

	for (int i = 0; i < n; i++) {
		for (int j = i + 1; j < n; j++) {
			double u = (double)rand() / (double)RAND_MAX;
			int a = (u < p) ? 1 : 0;
			M[i][j] = a;
			M[j][i] = a;
		}
	}
}

/* File simple pour BFS */
typedef struct {
	int data[MAXN];
	int head;
	int tail;
} Queue;

static void q_init(Queue *q) { q->head = q->tail = 0; }
static int q_empty(Queue *q) { return q->head == q->tail; }
static void q_push(Queue *q, int x) { q->data[q->tail++] = x; }
static int q_pop(Queue *q) { return q->data[q->head++]; }

/* 1) Procédure de la vague (BFS) pour connexité + composantes connexes.
   comp[i] = numéro de composante (1..k). Retourne k.
   ops_voisins compte le nombre de tests "est-ce voisin ?" (boucle j=0..n-1). */
static int composantes_connexes_vague(int n, int M[MAXN][MAXN], int comp[MAXN], long long *ops_voisins)
{
	for (int i = 0; i < n; i++) comp[i] = 0;
	if (ops_voisins) *ops_voisins = 0;

	int k = 0;
	Queue q;

	for (int s = 0; s < n; s++) {
		if (comp[s] != 0) continue;
		k++;
		comp[s] = k;
		q_init(&q);
		q_push(&q, s);

		while (!q_empty(&q)) {
			int u = q_pop(&q);
			for (int v = 0; v < n; v++) {
				if (ops_voisins) (*ops_voisins)++;
				if (M[u][v] && comp[v] == 0) {
					comp[v] = k;
					q_push(&q, v);
				}
			}
		}
	}
	return k;
}

/* 2) Adapter la vague pour calculer les plus courts chemins (non pondéré):
   dist[v] = nombre d'arêtes depuis src (INF si inaccessible)
   parent[v] pour reconstruire un chemin. */
static void bfs_plus_courts_chemins(int n, int M[MAXN][MAXN], int src,
				   int dist[MAXN], int parent[MAXN], long long *ops_voisins)
{
	for (int i = 0; i < n; i++) {
		dist[i] = INF;
		parent[i] = -1;
	}
	if (ops_voisins) *ops_voisins = 0;

	Queue q;
	q_init(&q);
	dist[src] = 0;
	parent[src] = src;
	q_push(&q, src);

	while (!q_empty(&q)) {
		int u = q_pop(&q);
		for (int v = 0; v < n; v++) {
			if (ops_voisins) (*ops_voisins)++;
			if (M[u][v] && dist[v] >= INF/2) {
				dist[v] = dist[u] + 1;
				parent[v] = u;
				q_push(&q, v);
			}
		}
	}
}

static void afficher_chemin_parent(int parent[MAXN], int src, int dst)
{
	if (parent[dst] == -1) {
		printf("(aucun chemin)\n");
		return;
	}
	int pile[MAXN];
	int top = 0;
	int x = dst;
	while (x != src) {
		pile[top++] = x;
		x = parent[x];
		if (x == -1 || top > MAXN - 1) {
			printf("(erreur reconstruction)\n");
			return;
		}
	}
	printf("%d", src);
	for (int i = top - 1; i >= 0; i--) printf(" -> %d", pile[i]);
	printf("\n");
}

/* 5) Dijkstra (poids positifs) sur matrice de poids W (INF = pas d'arête).
   dist[] et parent[]; ops_voisins compte les relaxations testées (boucle v=0..n-1). */
static void dijkstra(int n, int W[MAXN][MAXN], int src,
		     int dist[MAXN], int parent[MAXN], long long *ops_voisins)
{
	int used[MAXN];
	for (int i = 0; i < n; i++) {
		dist[i] = INF;
		parent[i] = -1;
		used[i] = 0;
	}
	if (ops_voisins) *ops_voisins = 0;

	dist[src] = 0;
	parent[src] = src;

	for (int iter = 0; iter < n; iter++) {
		int u = -1;
		int best = INF;
		for (int i = 0; i < n; i++) {
			if (!used[i] && dist[i] < best) {
				best = dist[i];
				u = i;
			}
		}
		if (u == -1) break;
		used[u] = 1;

		for (int v = 0; v < n; v++) {
			if (ops_voisins) (*ops_voisins)++;
			if (used[v]) continue;
			if (W[u][v] >= INF/2) continue;
			if (dist[u] + W[u][v] < dist[v]) {
				dist[v] = dist[u] + W[u][v];
				parent[v] = u;
			}
		}
	}
}

/* Utilitaire : construire W à partir d'un graphe 0/1 (poids aléatoires positifs) */
static void poids_positifs_depuis_adj(int n, int M[MAXN][MAXN], int W[MAXN][MAXN], int wmin, int wmax)
{
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i == j) W[i][j] = 0;
			else if (M[i][j]) W[i][j] = wmin + (rand() % (wmax - wmin + 1));
			else W[i][j] = INF;
		}
	}
}

/* 7) Poids négatifs — rappel (dans le compte rendu) :
   - Dijkstra ne marche pas (un sommet "figé" peut être amélioré par un poids négatif)
   - Alternatives : Bellman-Ford (gère poids négatifs, détecte cycles négatifs)
   - Floyd-Warshall (APSP) marche aussi en présence de poids négatifs (si pas de cycles négatifs)
   - Difficulté : cycles de poids total négatif => pas de plus court chemin (distance = -inf)
   - Remède : détecter cycles négatifs (Bellman-Ford, ou diagonal négative après Floyd) */

int main(int argc, char **argv)
{
	int n = 8;
	double p = 0.30;
	unsigned int seed = (unsigned int)time(NULL);
	int src = 0;
	int fromInput = 0;

	/* Modes:
	   - Random (défaut): ./tp1partie2 [n] [p] [seed]
	   - Annexe (stdin):  ./tp1partie2 --stdin
	   - Annexe (fichier):./tp1partie2 --file chemin.txt
	*/
	if (argc >= 2 && strcmp(argv[1], "--stdin") == 0) {
		fromInput = 1;
	} else if (argc >= 3 && strcmp(argv[1], "--file") == 0) {
		fromInput = 2;
	} else {
		if (argc >= 2) n = atoi(argv[1]);
		if (argc >= 3) p = atof(argv[2]);
		if (argc >= 4) seed = (unsigned int)strtoul(argv[3], NULL, 10);
		if (n < 1) n = 1;
		if (n > MAXN) n = MAXN;
		if (p < 0.0) p = 0.0;
		if (p > 1.0) p = 1.0;
	}

	int M[MAXN][MAXN];
	int W[MAXN][MAXN];
	int hasWeights = 0;

	if (fromInput) {
		FILE *in = stdin;
		if (fromInput == 2) {
			in = fopen(argv[2], "r");
			if (!in) {
				perror("fopen");
				return 1;
			}
		}
		if (!lire_graphe(in, &n, M, W, &src, &hasWeights)) {
			fprintf(stderr, "Format invalide. Attendu: n m [S] puis m lignes u v [w].\n");
			if (fromInput == 2) fclose(in);
			return 1;
		}
		if (fromInput == 2) fclose(in);

		printf("Sujet 2 — Graphe annexe (n=%d, src=%d)\n\n", n, src);
		afficher_0_1(n, M, "Graphe non orienté (matrice d’adjacence)");
		printf("\n");
	} else {
		srand(seed);
		src = 0;
		printf("Sujet 2 — Graphe aléatoire (n=%d, p=%.2f, seed=%u)\n\n", n, p, seed);
		graphe_aleatoire_non_oriente(n, p, M);
		poids_positifs_depuis_adj(n, M, W, 1, 9);
		hasWeights = 1;
		afficher_0_1(n, M, "Graphe non orienté (matrice d’adjacence)");
		printf("\n");
	}

	/* 1) Vague/BFS: composantes connexes */
	int comp[MAXN];
	long long ops1 = 0;
	int k = composantes_connexes_vague(n, M, comp, &ops1);
	printf("1) Nombre de composantes connexes = %d\n", k);
	afficher_vect(n, comp, "   Etiquettes comp[i] (1..k)");
	printf("   Compteur (tests voisins) = %lld\n\n", ops1);

	/* 2) Vague/BFS: plus courts chemins (non pondéré) depuis src */
	int dist[MAXN], parent[MAXN];
	long long ops2 = 0;
	bfs_plus_courts_chemins(n, M, src, dist, parent, &ops2);
	printf("2) Distances (nb d’arêtes) depuis %d\n", src);
	afficher_dist(n, dist, "   dist[]");
	printf("   Exemple chemin %d -> %d : ", src, n - 1);
	afficher_chemin_parent(parent, src, n - 1);
	printf("   Compteur (tests voisins) = %lld\n\n", ops2);

	/* 3) Comparaison avec Floyd-Warshall (exemples du TP1) */
	int D_unw[MAXN][MAXN];
	init_dist_unweighted(n, M, D_unw);
	floyd_warshall(n, D_unw);
	int okBFS = 1;
	for (int v = 0; v < n; v++) {
		int dv = (dist[v] >= INF/2) ? INF : dist[v];
		int fv = D_unw[src][v];
		if ((dv >= INF/2) != (fv >= INF/2) || (dv < INF/2 && dv != fv)) {
			okBFS = 0;
			break;
		}
	}
	printf("3) Vérif Floyd (non pondéré) vs BFS : %s\n\n", okBFS ? "OK" : "DIFFÉRENCE");

	/* 4-5) Dijkstra (poids positifs) + justification d'exactitude: voir compte rendu
	   Hypothèse clé: tous les poids sont >= 0. */
	if (hasWeights) {
		int distD[MAXN], parentD[MAXN];
		long long opsD = 0;
		dijkstra(n, W, src, distD, parentD, &opsD);
		printf("5) Dijkstra (poids) depuis %d — dist[]\n", src);
		for (int i = 0; i < n; i++) {
			if (distD[i] >= INF/2) printf("INF ");
			else printf("%d ", distD[i]);
		}
		printf("\n   Exemple chemin %d -> %d : ", src, n - 1);
		afficher_chemin_parent(parentD, src, n - 1);
		printf("   Compteur (tests voisins) = %lld\n", opsD);

		int D_w[MAXN][MAXN];
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				D_w[i][j] = W[i][j];
		floyd_warshall(n, D_w);
		int okDij = 1;
		for (int v = 0; v < n; v++) {
			int dv = distD[v];
			int fv = D_w[src][v];
			if ((dv >= INF/2) != (fv >= INF/2) || (dv < INF/2 && dv != fv)) {
				okDij = 0;
				break;
			}
		}
		printf("   Vérif Floyd (pondéré) vs Dijkstra : %s\n\n", okDij ? "OK" : "DIFFÉRENCE");
	} else {
		printf("5) Dijkstra non exécuté: aucun poids fourni (graphe non pondéré).\n\n");
	}

	/* 7) Poids négatifs (réponse attendue au TP, résumé):
	   - Dijkstra échoue avec des poids négatifs.
	   - Proposer Bellman-Ford (poids négatifs, détection cycles négatifs).
	   - Difficulté: cycle négatif => pas de plus court chemin (distance = -inf).
	   - Remède: détecter/traiter les cycles négatifs (Bellman-Ford / Floyd). */

	return 0;
}

