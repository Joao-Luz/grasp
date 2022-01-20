#include <stdio.h>
#include <time.h>
#include "grasp.h"

typedef struct ks_item {
	float value;
	float weight;
} ks_item;

typedef struct ks_args {
	float limit;
} ks_args;

void ks_cost(grasp* g, void* v_items, int n_items, float* costs) {
	ks_item* items = (ks_item*) v_items;

	for (int i = 0; i < n_items; i++) {
		costs[i] = items[i].value/items[i].weight;
	}
}

void ks_candidates(grasp* g, void* v_items, int n_items, int* solution, int n_solution, bool* candidates) {
	ks_item* items = (ks_item*) v_items;
	float weight = 0;
	ks_args* args = (ks_args*) g->problem;

	for (int i = 0; i < n_solution; i++) {
		weight += items[solution[i]].weight;
		candidates[solution[i]] = false;
	}
	for (int i = 0; i < n_items; i++)
		if (items[i].weight > args->limit - weight) candidates[i] = false;
}

void ks_compare(grasp* g, void* v_items, int* sol, int n_sol, int* best, int* n_best) {
	ks_item* items = (ks_item*) v_items;

	float val_sol = 0;
	for (int i = 0; i < n_sol; i++) val_sol += items[sol[i]].value;

	float val_best = 0;
	for (int i = 0; i < *n_best; i++) val_best += items[best[i]].value;

	if (val_sol > val_best) {
		for (int i = 0; i < n_sol; i++) best[i] = sol[i];
		*n_best = n_sol;
	}
}

void ks_local_search(grasp* g, void* v_items, int n_items, int* solution, int* n_solution) {
	ks_item* items = (ks_item*) v_items;

	int val = 0;
	for (int i = 0; i < *n_solution; i++) val += items[solution[i]].value;

	for (int i = 0; i < *n_solution; i++) {
		for (int j = 0; j < n_items; j++) {
			int is_in = 0; for(int k = 0; k < *n_solution; k++) is_in += solution[k] == j;
			if (!is_in) {
				int new_val = val - items[solution[i]].value + items[j].value;
				if (new_val > val) solution[i] = j;
			}
		}
	}

}

int main() {
	srand(time(NULL));
	
	ks_args args = {.limit = 20};
	grasp g = {.iterations = 20,
			   .alpha = 0.6,
			   .max = true,
			   .compute_costs = ks_cost,
			   .compare_solutions = ks_compare,
			   .update_candidates = ks_candidates,
			   .local_search = ks_local_search,
			   .problem = &args
			   };
			   
	ks_item items[10];

	for (int i = 0; i < 10; i++) {
		items[i] = (ks_item) {.value = rand() % 15, .weight = rand() % 10};
	}

	int solution[10];
	int n_solution;
	grasp_run(&g, (void*)items, 10, solution, &n_solution);

	float val = 0;
	for (int i = 0; i < n_solution; i++) {
		val += items[solution[i]].value;
	}

	printf("Valor final: %.2f\n", val);
}
