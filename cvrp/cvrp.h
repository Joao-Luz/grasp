#pragma once

#include "grasp.h"

typedef struct cvrp_node{
    int x, y;
    int demand;
} cvrp_node;

typedef struct cvrp_route {
    int length;
    int* stops;
} cvrp_route;

typedef struct cvrp_data {
    int cap;
    int n_vehicles, n_nodes;
    cvrp_node depot;
    cvrp_node* nodes;
    int* _routes_indices;
    int* _best_routes_indices;
    float sa_alpha, sa_temp;
    bool verbose;
} cvrp_data;

cvrp_route* cvrp_solve(cvrp_data* data, int iterations, float alpha);

float cvrp_total_cost(cvrp_route* routes, int n_routes, cvrp_node* nodes, cvrp_node depot);