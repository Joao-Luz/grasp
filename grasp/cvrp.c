#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "grasp.h"

typedef struct cvrp_node{
    int x, y;
    int demand;
} cvrp_node;


int main(int argc, char** argv) {
    FILE* fd = fopen(argv[1], "r");
    const int n = atoi(argv[2]);
    const int k = atoi(argv[3]);

    char* buff = NULL;
    size_t size = 0;
    getline(&buff, &size, fd);
    while(!strstr(buff, "NODE_COORD_SECTION")) getline(&buff, &size, fd);

    cvrp_node nodes[n];
    for (int i = 0; i < n; i++) {
        getline(&buff, &size, fd);
        strtok(buff, " ");
        int x = atoi(strtok(NULL, " "));
        int y = atoi(strtok(NULL, " "));
        nodes[i] = (cvrp_node){.x=x, .y=y};
    }
    getline(&buff, &size, fd);
    for (int i = 0; i < n; i++) {
        getline(&buff, &size, fd);
        strtok(buff, " ");
        int demand = atoi(strtok(NULL, " "));
        nodes[i].demand = demand;
    }
    fclose(fd);
    free(buff);

	grasp g = {
        .iterations = 20,
        .alpha = 0.6,
        .max = true,
        .compute_costs = cvrp_costs,
        .compare_solutions = cvrp_compare,
        .update_candidates = cvrp_candidates,
        .local_search = cvrp_local_search,
        .problem = &args
    };
}