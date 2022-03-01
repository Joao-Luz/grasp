#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "grasp.h"

#include "cvrp.h"

int main(int argc, char** argv) {
    srand(time(NULL));

    FILE* fd = fopen(argv[1], "r");
    int n;
    int k;

    // read file
    char* buff = NULL;
    size_t size = 0;
    getline(&buff, &size, fd);
    sscanf(buff, "%*[^0-9]%d%*[^0-9]%d", &n, &k);
    while(!strstr(buff, "CAPACITY")) getline(&buff, &size, fd);

    int cap = atoi(buff+11);

    getline(&buff, &size, fd);

    cvrp_node depot;
    cvrp_node nodes[n-1];
    for (int i = 0; i < n; i++) {
        getline(&buff, &size, fd);
        strtok(buff, " ");
        int x = atoi(strtok(NULL, " "));
        int y = atoi(strtok(NULL, " "));
        if(i == 0) {
            depot = (cvrp_node){.x = x, .y = y};
        } else {
            nodes[i-1] = (cvrp_node){.x=x, .y=y};
        }

    }
    getline(&buff, &size, fd);
    for (int i = 0; i < n; i++) {
        getline(&buff, &size, fd);
        strtok(buff, " ");
        int demand = atoi(strtok(NULL, " "));
        if(i == 0) {
            depot.demand = demand;
        } else {
            nodes[i-1].demand = demand;
        }
    }
    fclose(fd);
    free(buff);

    cvrp_data data = {
        .cap = cap,
        .depot = depot,
        .nodes = nodes,
        .n_nodes = n-1,
        .verbose = true
    };

    float alpha=0.5;
    if(argc >= 3) alpha = atof(argv[2]);

    int iter = 500;
    if(argc >= 4) iter = atoi(argv[3]);

    clock_t start, end;
    double elapsed_time;
    start = clock();
    cvrp_route* routes = cvrp_solve(&data, iter, alpha);
    end = clock();
    elapsed_time = (double)(end-start)/CLOCKS_PER_SEC;

    for(int i = 0; i < data.n_vehicles; i++) {
        printf("Route #%d: ", i+1);
        
        cvrp_route route = routes[i];
        for(int j = 0; j < route.length; j++) {
            printf("%d ", route.stops[j]+1);
        }
        printf("\n");
    }

    printf("Cost %.0f\n", cvrp_total_cost(routes, data.n_vehicles, nodes, depot));
    printf("Time %.5f seconds\n", elapsed_time);

    for(int i = 0; i < data.n_vehicles; i++) free(routes[i].stops);
    free(routes);
}