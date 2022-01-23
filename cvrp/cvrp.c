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

void free_routes(cvrp_route* routes, int n_routes) {
    for(int i = 0; i < n_routes; i++) free(routes[i].stops);
    free(routes);
}

cvrp_route* copy_routes(cvrp_route* routes, int n_routes) {
    cvrp_route* copy = malloc(n_routes*sizeof(cvrp_route));
    for(int i = 0; i < n_routes; i++) {
        copy[i].stops = malloc(routes[i].length*sizeof(int));
        copy[i].length = routes[i].length;
        for(int j = 0; j < routes[i].length; j++) {
            copy[i].stops[j] = routes[i].stops[j];
        }
    }
    return copy;
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

    // apply the nearest-neighbor heuristic
    cvrp_route* routes = indices_to_routes(split_solution, data->_routes_indices, n_vehicles);
    for(int i = 0; i < n_vehicles; i++) {
        cvrp_route* route = &routes[i];
        bool visited[route->length]; for(int j = 0; j < route->length; j++) visited[j] = false;
        int new_stops[route->length];

        cvrp_node last_stop = data->depot;
        for(int j = 0; j < route->length; j++) {
            // find nearest node to last_node
            cvrp_node nearest; int nearest_index;
            float min_distance = INFINITY;
            for(int k = 0; k < route->length; k++) {
                if(visited[k]) continue;
                cvrp_node current = nodes[route->stops[k]];
                float distance = cvrp_distance(last_stop, current);
                if(distance < min_distance) {
                    nearest = current;
                    min_distance = distance;
                    nearest_index = k;
                }
            }
            last_stop = nearest;
            visited[nearest_index] = true;
            new_stops[j] = solution[route->stops[nearest_index]];
        }
        for(int j = 0; j < route->length; j++) route->stops[j] = new_stops[j];
    }

    for(int i = 0; i < n_solution; i++) solution[i] = split_solution[i];
}

void swap_nodes(cvrp_route route, int i, int j) {
    int aux = route.stops[i];
    route.stops[i] = route.stops[j];
    route.stops[j] = aux;
}

void invert_nodes(cvrp_route route, int i, int j) {
    if(j > i) {
        int tmp = i;
        i = j;
        j = tmp;
    }
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

cvrp_route* best_swap_neighbor(cvrp_route* original_routes, int n_routes, int cap, cvrp_node* nodes, cvrp_node depot) {
    cvrp_route* routes = copy_routes(original_routes, n_routes);
    float best_cost = cvrp_total_cost(routes, n_routes, nodes, depot);

    for(int i = 0; i < n_routes; i++) {
        cvrp_route route = routes[i];
        if(route.length == 1) continue;

        int a = rand()%(route.length);
        int b = rand()%(route.length);
        while (b == a){
            b = rand()%(route.length);
        }

        swap_nodes(route, a, b);
        float cost = cvrp_total_cost(routes, n_routes, nodes, depot);
        if(cost >= best_cost)
            swap_nodes(route, a, b);
    }

    return routes;
}

cvrp_route* best_invert_neighbor(cvrp_route* original_routes, int n_routes, int cap, cvrp_node* nodes, cvrp_node depot) {
    cvrp_route* routes = copy_routes(original_routes, n_routes);
    float best_cost = cvrp_total_cost(routes, n_routes, nodes, depot);

    for(int i = 0; i < n_routes; i++) {
        cvrp_route route = routes[i];
        if(route.length == 1) continue;

        int a = rand()%(route.length);
        int b = rand()%(route.length);
        while (b == a) {
            b = rand()%(route.length);
        }

        invert_nodes(route, a, b);
        float cost = cvrp_total_cost(routes, n_routes, nodes, depot);
        if(cost >= best_cost)
            invert_nodes(route, a, b);
    }

    return routes;
}

cvrp_route* best_2opt_neighbor(cvrp_route* original_routes, int n_routes, int cap, cvrp_node* nodes, cvrp_node depot) {
    cvrp_route* routes = copy_routes(original_routes, n_routes);
    float best_cost = cvrp_total_cost(routes, n_routes, nodes, depot);
    bool spliced[n_routes]; for(int i = 0; i < n_routes; i++) spliced[i] = false;

    for(int i = 0; i < n_routes; i++) {
        cvrp_route* route_i = &routes[i];
        for(int j = 0; j < n_routes; j++) {
            cvrp_route* route_j = &routes[j];
            if(i == j || (route_i->length == 1 && route_j->length == 1) || spliced[j]) continue;

            int a = rand()%(route_i->length);
            int b = rand()%(route_j->length);

            splice(route_i, route_j, a, b);
            float current_cost = cvrp_total_cost(routes, n_routes, nodes, depot);
            bool feasable = cvrp_feasable(routes, n_routes, nodes, cap);

            if(current_cost < best_cost && feasable) {
                best_cost = current_cost;
                spliced[i] = spliced[j] = true;
            } else {
                splice(route_i, route_j, a, b);
            }

        }
    }

    return routes;
}

float random_real() {
    return (float)rand()/RAND_MAX;
}

void _cvrp_local_search(grasp* g, void* v_nodes, int n_nodes, int* solution, int* n_solution) {
    cvrp_node* nodes = (cvrp_node*) v_nodes;
	cvrp_data* data = (cvrp_data*) g->data;

    cvrp_node depot = data->depot;
    int n_vehicles = data->n_vehicles;

    // apply simulated anealing
    float temperature = 150;
    float alpha = 0.8;
    cvrp_route* best_routes = indices_to_routes(solution, data->_routes_indices, n_vehicles);
    float best_cost = cvrp_total_cost(best_routes, n_vehicles, nodes, depot);
    while(temperature > 1) {

        cvrp_route* current_routes;
        int r = rand()%3;
        switch (r) {
        case 0:
            current_routes = best_2opt_neighbor(best_routes, n_vehicles, data->cap, nodes, data->depot);
            break;
        case 1:
            current_routes = best_swap_neighbor(best_routes, n_vehicles, data->cap, nodes, data->depot);
            break;
        default:
            current_routes = best_invert_neighbor(best_routes, n_vehicles, data->cap, nodes, data->depot);
            break;
        }
        float opt_cost = cvrp_total_cost(current_routes, n_vehicles, nodes, depot);
        
        if(opt_cost < best_cost) {
            best_cost = opt_cost;
            free_routes(best_routes, n_vehicles);
            best_routes = best_2opt_neighbor(current_routes, n_vehicles, data->cap, nodes, data->depot);
            free_routes(current_routes, n_vehicles);
        }
        else if(exp((best_cost-opt_cost)/temperature) > random_real()){
            free_routes(best_routes, n_vehicles);
            best_routes = current_routes;
        } else {
            free_routes(current_routes, n_vehicles);
        }

        temperature *= alpha;
    }

    routes_to_indices(best_routes, n_vehicles, solution, data->_routes_indices);
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