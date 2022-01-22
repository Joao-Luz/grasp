#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "grasp.h"

float argmin(float* elements, int n, bool* index) {
	float min = INFINITY;
	for (int i = 0; i < n; i++) {
		if (elements[i] < min && index[i]) min = elements[i];
	}
	return min;
}

float argmax(float* elements, int n, bool* index) {
	float max = -INFINITY;
	for (int i = 0; i < n; i++) {
		if (elements[i] > max && index[i]) max = elements[i];
	}
	return max;
}

static void construct(grasp* g, void* elements, const int n_elements, int* solution, int* n_solution) {
	// construction
	// initialize empty solution
	*n_solution = 0;
	// compute incremental costs
	float costs[n_elements];
	g->compute_costs(g, elements, n_elements, solution, *n_solution, costs);

	
	// initialize candidate list
	bool candidates[n_elements];
	for (int j = 0; j < n_elements; j++) candidates[j] = true;
	int n_candidates = n_elements;
	g->update_candidates(g, elements, n_elements, solution, *n_solution, candidates);

	// construct
	while (n_candidates != 0) {
		float c_min = argmin(costs, n_elements, candidates);
		float c_max = argmax(costs, n_elements, candidates);

		// build restricted candidate list
		int rcl[n_elements];
		int n_rcl = 0;
		float base_cost = c_min + g->alpha*(c_max - c_min);
		for (int k = 0; k < n_elements; k++) {
			if (candidates[k] == false) continue;
			if (g->max) {
				if (costs[k] >= base_cost) rcl[n_rcl++] = k;
			} else {
				if (costs[k] <= base_cost) rcl[n_rcl++] = k;
			}
		}
		// select random element from rcl
		int element = rcl[rand() % n_rcl];
		solution[(*n_solution)++] = element;

		// update candidates and incremental costs
		g->update_candidates(g, elements, n_elements, solution, *n_solution, candidates);
		g->compute_costs(g, elements, n_elements, solution, *n_solution, costs);
		n_candidates = 0;
		for (int k = 0; k < n_elements; k++) if (candidates[k] == true) n_candidates += 1; 
	}
}

void grasp_run(grasp* g, void* elements, const int n_elements, int* best_solution, int* n_best_solution) {
	
	int solution[n_elements];
	int n_solution = 0;
	for (int i = 0; i < g->iterations; i++){
		construct(g, elements, n_elements, solution, &n_solution);

		g->post_construction(g, elements, n_elements, solution, n_solution);
		g->local_search(g, elements, n_elements, solution, &n_solution);
		bool first = i == 0 ? true : false;
		g->compare_solutions(g, elements, solution, n_solution, best_solution, n_best_solution, first);
	}	
}
