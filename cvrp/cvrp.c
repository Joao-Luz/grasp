#include "cvrp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "grasp.h"

float cvrp_distance(cvrp_node a, cvrp_node b) {
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

void print_route(cvrp_route route) {
    for(int i = 0; i < route.length; i++) printf("%d ", route.stops[i]);
    printf("\n");
}

// transform the array of indices into different routes
cvrp_route* indices_to_routes(int array[], int indices[], int n_routes) {
    cvrp_route* routes = malloc(n_routes*sizeof(cvrp_route));

    for(int i = 0; i < n_routes; i++) {
        cvrp_route route;
        
        int start = i == 0 ? 0 : indices[i-1];
        int length = indices[i] - start;

        route.length = length;
        route.stops = malloc(length*sizeof(int));
        
        for(int j = start; j < indices[i]; j++) {
            route.stops[j-start] = array[j];
        }

        routes[i] = route;
    }

    return routes;
}

void routes_to_indices(cvrp_route* routes, int n_routes, int* array, int* indices) {
    int index = 0;
    for(int i = 0; i < n_routes; i++) {
        cvrp_route route = routes[i];
        for(int j = 0; j < route.length; j++) {
            array[index++] = route.stops[j];
        }
        free(route.stops);
        indices[i] = index;
    }
    free(routes);
}

float cvrp_total_cost(cvrp_route* routes, int n_routes, cvrp_node* nodes, cvrp_node depot) {
    float total_cost = 0;
    for(int i = 0; i < n_routes; i++) {
        cvrp_route route = routes[i];
        cvrp_node first_node = nodes[route.stops[0]];
        cvrp_node last_node = nodes[route.stops[route.length-1]];
        total_cost += cvrp_distance(depot, first_node) + cvrp_distance(last_node, depot);

        for(int j = 1; j < route.length; j++) {
            total_cost += cvrp_distance(nodes[route.stops[j-1]], nodes[route.stops[j]]);
        }
    }

    return total_cost;
}

bool cvrp_feasable(cvrp_route* routes, int n_routes, cvrp_node* nodes, int cap) {
    for(int i = 0; i < n_routes; i++) {
        cvrp_route route = routes[i];
        int total_demand = 0;
        for(int j = 0; j < route.length; j++) {
            total_demand += nodes[route.stops[j]].demand;
            if(total_demand > cap) return false;
        }
    }
    return true;
}

void _cvrp_costs(grasp* g, void* v_nodes, int n_nodes, int* solution, int n_solution, float* costs) {
	cvrp_node* nodes = (cvrp_node*) v_nodes;
    cvrp_data* data = (cvrp_data*) g->data;

    cvrp_node node_i;
    if(n_solution > 0) node_i = nodes[solution[n_solution-1]];
    else if(n_solution == 0) node_i = data->depot;

	for (int j = 0; j < n_nodes; j++) {
        if(n_solution > 0 && solution[n_solution-1] == j) {
            costs[j] = 0;
        } else {
            cvrp_node node_j = nodes[j];
            
            float distance = node_i.x == node_j.x && node_i.y == node_j.y ? 1e-3 : cvrp_distance(node_i, node_j);
            costs[j] = pow(abs(data->cap - node_j.demand + node_i.demand), 1)/pow(distance, 2);
        }
    }
}

void _cvrp_compare(grasp* g, void* v_nodes, int* sol, int n_sol, int* best, int* n_best, bool first_solution) {
	cvrp_node* nodes = (cvrp_node*) v_nodes;
	cvrp_data* data = (cvrp_data*) g->data;

    if(first_solution) {
        for(int i = 0; i < n_sol; i++) best[i] = sol[i];
		*n_best = n_sol;
        for(int i = 0; i < data->n_vehicles; i++) {
            data->_best_routes_indices[i] = data->_routes_indices[i];
        }
        return;
    }

    cvrp_route* current_routes = indices_to_routes(sol, data->_routes_indices, data->n_vehicles);
    float current_cost = cvrp_total_cost(current_routes, data->n_vehicles, nodes, data->depot);
    routes_to_indices(current_routes, data->n_vehicles, sol, data->_routes_indices);

    cvrp_route* best_routes = indices_to_routes(best, data->_best_routes_indices, data->n_vehicles);
    float best_cost = cvrp_total_cost(best_routes, data->n_vehicles, nodes, data->depot);
    routes_to_indices(best_routes, data->n_vehicles, best, data->_best_routes_indices);

    if(current_cost < best_cost) {
        for(int i = 0; i < n_sol; i++) best[i] = sol[i];
		*n_best = n_sol;
        for(int i = 0; i < data->n_vehicles; i++) data->_best_routes_indices[i] = data->_routes_indices[i];
    }

    for(int i = 0; i < data->n_vehicles; i++) data->_routes_indices[i] = data->_best_routes_indices[i];
}

void _cvrp_candidates(grasp* g, void* v_nodes, int n_nodes, int* solution, int n_solution, bool* candidates) {
	cvrp_node* nodes = (cvrp_node*) v_nodes;
	cvrp_data* data = (cvrp_data*) g->data;

	for (int i = 0; i < n_solution; i++) {
		candidates[solution[i]] = false;
	}
}

void _cvrp_split(grasp* g, void* v_nodes, int n_nodes, int* solution, int n_solution) {
    cvrp_node* nodes = (cvrp_node*) v_nodes;
	cvrp_data* data = (cvrp_data*) g->data;

    int n_vehicles = data->n_vehicles;
    int cap = data->cap;
    cvrp_node depot = data->depot;

    bool visited[n_nodes]; for(int i = 0; i < n_nodes; i++) visited[i] = false;
    int split_solution[n_solution];
    int route_index = 0;

    for(int i = 0; i < n_vehicles; i++) {
        int limit = cap;
        for(int j = 0; j < n_solution; j++) {
            if(visited[j]) continue;
            cvrp_node current_node = nodes[solution[j]];
            if(current_node.demand <= limit) {
                visited[j] = true;
                limit -= current_node.demand;
                split_solution[route_index++] = solution[j];
            }
        }
        data->_routes_indices[i] = route_index;
    }
    for(int i = 0; i < n_solution; i++) solution[i] = split_solution[i];
}

void swap_nodes(cvrp_route route, int i, int j) {
    int aux = route.stops[i];
    route.stops[i] = route.stops[j];
    route.stops[j] = aux;
}

void invert_nodes(cvrp_route route, int i, int j) {
    for(; i < j; i++, j--)
        swap_nodes(route, i, j);
}

void splice(cvrp_route* route_a, cvrp_route* route_b, int a, int b) {

    int tail_len_a = route_a->length-(a+1);
    int tail_a[tail_len_a]; for(int i = a+1; i < route_a->length; i++) tail_a[i-(a+1)] = route_a->stops[i];

    int tail_len_b = route_b->length-(b+1);
    int tail_b[tail_len_b]; for(int i = b+1; i < route_b->length; i++) tail_b[i-(b+1)] = route_b->stops[i];
    
    int new_length_a = route_a->length - tail_len_a + tail_len_b;
    int new_length_b = route_b->length - tail_len_b + tail_len_a;

    route_a->stops = realloc(route_a->stops, new_length_a*sizeof(int));
    for(int i = a+1; i < new_length_a; i++) route_a->stops[i] = tail_b[i-(a+1)];


    route_b->stops = realloc(route_b->stops, new_length_b*sizeof(int));
    for(int i = b+1; i < new_length_b; i++) route_b->stops[i] = tail_a[i-(b+1)];

    route_a->length = new_length_a;
    route_b->length = new_length_b;
}

void _cvrp_local_search(grasp* g, void* v_nodes, int n_nodes, int* solution, int* n_solution) {
    cvrp_node* nodes = (cvrp_node*) v_nodes;
	cvrp_data* data = (cvrp_data*) g->data;

    cvrp_node depot = data->depot;
    int n_vehicles = data->n_vehicles;

    // cvrp_route* routes_swap = indices_to_routes(solution, data->_routes_indices, data->n_vehicles);
    // float original_cost = cvrp_total_cost(routes_swap, data->n_vehicles, nodes, depot);


    // cvrp_route* routes_inversion = indices_to_routes(solution, data->_routes_indices, data->n_vehicles);
    // original_cost = cvrp_total_cost(routes_inversion, data->n_vehicles, nodes, depot);

    // // inversion
    // for(int i = 0; i < n_vehicles; i++) {
    //     cvrp_route route = routes_inversion[i];
    //     if(route.length == 1) continue;
        
    //     int j = rand()%route.length;
    //     int k = rand()%route.length;
    //     while(j==k) k = rand()%route.length;

    //     if(j > k) {
    //         int aux = j;
    //         j = k;
    //         k = aux;
    //     }

    //     invert_nodes(route, j, k);
    //     float current_cost = cvrp_total_cost(routes_inversion, data->n_vehicles, nodes, depot);

    //     if(current_cost >= original_cost)
    //         invert_nodes(route, j, k);
    // }
    // float inversion_cost = cvrp_total_cost(routes_inversion, data->n_vehicles, nodes, depot);

    cvrp_route* routes_2opt = indices_to_routes(solution, data->_routes_indices, data->n_vehicles);
    float best_2opt_cost = cvrp_total_cost(routes_2opt, data->n_vehicles, nodes, depot);

    // swap
    int tries = 0;
    for(int i = 0; i < n_vehicles; i++) {
        cvrp_route route = routes_2opt[i];

        if(route.length == 1) continue;

        int j = rand()%route.length;
        int k = rand()%route.length;
        while(j==k) k = rand()%route.length;
        swap_nodes(route, j, k);

        float current_cost = cvrp_total_cost(routes_2opt, data->n_vehicles, nodes, depot);
        if(current_cost >= best_2opt_cost) {
            swap_nodes(route, j, k);
            if(++tries < 10) i--;
        } else {
            tries = 0;
        }
    }

    // 2-opt
    bool spliced[n_vehicles]; for(int i = 0; i < n_vehicles; i++) spliced[i] = false;
    tries = 0;
    for(int i = 0; i < n_vehicles; i++) {
        cvrp_route* route_i = &routes_2opt[i];
        if(route_i->length == 1) continue;
        for(int j = 0; j < n_vehicles; j++) {
            cvrp_route* route_j = &routes_2opt[j];
            if(i == j || route_j->length == 1 || spliced[j]) continue;

            int a = rand()%(route_i->length-1);
            int b = rand()%(route_j->length-1);

            splice(route_i, route_j, a, b);
            float current_cost = cvrp_total_cost(routes_2opt, data->n_vehicles, nodes, depot);
            bool feasable = cvrp_feasable(routes_2opt, n_vehicles, nodes, data->cap);

            if(current_cost < best_2opt_cost && feasable) {
                spliced[i] = spliced[j] = true;
                best_2opt_cost = current_cost;
                tries = 0;
                break;
            } else {
                splice(route_i, route_j, a, b);
                if(++tries < 15) j--;
            }
        }
    }

    routes_to_indices(routes_2opt, n_vehicles, solution, data->_routes_indices);
}

cvrp_route* cvrp_solve(cvrp_data* data, int iterations, float alpha) {

    int _routes_indices[data->n_vehicles];
    int _best_routes_indices[data->n_vehicles];
    data->_routes_indices = _routes_indices;
    data->_best_routes_indices = _best_routes_indices;

    grasp g = {
        .iterations = iterations,
        .alpha = alpha,
        .max = true,
        .compute_costs = _cvrp_costs,
        .compare_solutions = _cvrp_compare,
        .update_candidates = _cvrp_candidates,
        .post_construction = _cvrp_split,
        .local_search = _cvrp_local_search,
        .data = data
    };

    int solution[data->n_nodes];
    int n_solution = 0;
    grasp_run(&g, (void*)data->nodes, data->n_nodes, solution, &n_solution);

    cvrp_route* routes = indices_to_routes(solution, data->_routes_indices, data->n_vehicles);
    return routes;
}