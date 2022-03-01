#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "grasp.h"

#include "cvrp.h"

void parse_args(int argc, char** argv, FILE** fd, float* alpha, int* iter, float* sa_temp, float* sa_alpha, bool* verbose) {

    if (argc < 2) {
        printf("No input file!\n");
        exit(1);
    }

    *fd = fopen(argv[1], "r");
    if (*fd == NULL) {
        printf("Invalid file \"%s\"\n", argv[1]);
        exit(1);
    }
    for (int i = 2; i < argc; i++) {
        char* arg = argv[i];

        // Default
        *alpha = 0.5;
        *iter = 300;
        *sa_temp = 5000;
        *sa_alpha = 0.9;
        *verbose = false;

        if      (!strcmp(arg, "--alpha")) {
            *alpha = atof(argv[++i]);
        }
        else if (!strcmp(arg, "--iter")) {
            *iter = atoi(argv[++i]);
        }
        else if (!strcmp(arg, "--satemp")) {
            *sa_temp = atof(argv[++i]);
        }
        else if (!strcmp(arg, "--saalpha")) {
            *sa_alpha = atof(argv[++i]);
        }
        else if (!strcmp(arg, "--verbose")) {
            *verbose = true;
        }
    }
}

int main(int argc, char** argv) {
    srand(time(NULL));


    FILE* fd;
    float alpha, sa_alpha, sa_temp;
    int iter;
    bool verbose;
    parse_args(argc, argv, &fd, &alpha, &iter, &sa_temp, &sa_alpha, &verbose);

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
        .n_vehicles = k,
        .sa_alpha = sa_alpha,
        .sa_temp = sa_temp,
        .verbose = verbose
    };

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