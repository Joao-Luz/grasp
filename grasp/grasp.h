#pragma once
#include <stdlib.h>

typedef enum { false, true } bool;
typedef struct grasp grasp;

typedef void (*grasp_cost) (grasp* g, void* elements, int n_elements, int* solution, int n_solution, float* costs);
typedef void (*grasp_candidates) (grasp* g, void* elements, int n_elements, int* solution, int n_solution, bool* candidates);
typedef void (*grasp_compare) (grasp* g, void* elements, int* solution, int n_solution, int* best_solution, int* n_best_solution, bool first_solution);
typedef void (*grasp_search) (grasp* g, void* elements, int n_elements, int* solution, int* n_solution);
typedef void (*grasp_post_construction) (grasp* g, void* elements, int n_elements, int* solution, int n_solution);

// This is the struct for and instance of GRASP
// iterarions - number of iterations
// alpha - the parameter alpha
// max - boolean to indicate if the problem should find the maximum (true) or mininum (false) cost
// data - any data that might be usefull to solve the problem
// compute_costs - function to evaluate the incremental cost of each element in the candidate list
// update_candidates - function to update the candidate list at each iteration of construction
// compare_solutions - the function to compare two solutions 
// local_search - the function that performs local search in the solution space around a specific solution
struct grasp {
	int iterations;
	float alpha;
	bool max;
	void* data;
	grasp_cost compute_costs;
	grasp_candidates update_candidates;
	grasp_compare compare_solutions;
	grasp_search local_search;
	grasp_post_construction post_construction;
};

// Run the grasp algorithm for a set of elements
// g - the GRASP instance
// elements - the elements of the problem
// n_elements - the number of elements
// best_solution - an array of integers where the indices of the items in the best solution will be stored
// n_best_solution - the number of items selected in the best solution
void grasp_run(grasp* g, void* elements, const int n_elements, int* best_solution, int* n_best_solution);
