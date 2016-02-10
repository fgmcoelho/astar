#ifndef PATHFINDING_H__

#define CONSTANT_STRAIGHT_COST 10
#define CONSTANT_DIAGONAL_COST 14
#define PATHFINDING_INFINITY 1000000000
#define POOL_START_SIZE 1024*8

#include "list.h"
#include "modifiedHeap.h"
#include "pool.h"

typedef struct edge_ref_t{
    unsigned i, j;
    struct __pathfinding_info_cell_t* cell;
} edgeRef;

typedef struct __pathfinding_info_cell_t{
    unsigned heapIndex;
    unsigned totalCost, costSoFar;
    char inClosedList, inOpenList;
    edgeRef parent;
    edgeRef* myRef;
} __pathfinding_info_cell;

typedef struct pathFindingStruct_t{
    __pathfinding_info_cell** mainGrid;
    modifiedHeap* openList;
    list* closedList;
    pool* referencesPool;
    unsigned maxX, maxY;
    int (*isWalkable)(void*, int, int);
    int (*heuristicDistance)(int, int, int, int);
    void* userGivenGrid;
} pathFindingStruct;

typedef struct path_to_follow_t {
    list* pathToFollow;
    listIterator it;
} pathToFollow;

void setPathFindingHeuristic(pathFindingStruct*, int (*)(int, int, int, int));
pathFindingStruct* createPathFindingStruct(void* , unsigned , unsigned , int (*)(void*, int, int));
list* processPathFinding(pathFindingStruct* , unsigned , unsigned , unsigned , unsigned );
void pathFindingStructFree(pathFindingStruct* path);
void releasePath(pathFindingStruct* path, list* nodesList);

#endif
