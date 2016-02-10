#include "pathfinding.h"
#include "modifiedHeap.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

static void printHeap(modifiedHeap* h){
    edgeRef* current;
    while ((current = modifiedHeapPopFirstElement(h)) != NULL){
        printf("Front value: %u.\n", current->cell->totalCost);
    }
}

static void checkOpenListValidity(modifiedHeap* h){

    modifiedHeapIterator it;
    modifiedHeapIteratorStart(h, &it);

    unsigned less = ((edgeRef*)modifiedHeapGetFirstElement(h))->cell->totalCost;
    edgeRef* current;
    for (current = modifiedHeapIteratorGetFirstElement(&it); current != NULL; current = modifiedHeapIteratorGetNextElement(&it)){
        if (current->cell->totalCost < less){
            printf("Current %u smaller than the first %u\n", current->cell->totalCost, less);
            printHeap(h);
            abort();
            break;
        }
    }

}

static inline int cellCompare (void* a, void* b){
    __pathfinding_info_cell* first = ((edgeRef*)a)->cell;
    __pathfinding_info_cell* second = ((edgeRef*)b)->cell;
    return first->totalCost < second->totalCost;
}

static inline void cellSetIndex(void* a, unsigned index){
    __pathfinding_info_cell* cell = ((edgeRef*)a)->cell;
    cell->heapIndex = index;
}

static inline unsigned cellGetIndex(void* a){
    __pathfinding_info_cell* cell = ((edgeRef*)a)->cell;
    return cell->heapIndex;
}

static inline edgeRef* getEdgeRef(pathFindingStruct* path, unsigned i, unsigned j, __pathfinding_info_cell* cell){

    edgeRef* newEdge = poolGetElement(path->referencesPool);
    if (newEdge == NULL){
        //printf("Error, out of memory.");
        return NULL;
    }

    newEdge->i = i;
    newEdge->j = j;
    newEdge->cell = cell;

    return newEdge;

}

static void clearPathFindingStruct(pathFindingStruct* path){

    unsigned i,j;
    for (i =0; i < path->maxY; ++i){
        for(j = 0; j < path->maxX; ++j){
            path->mainGrid[i][j].inClosedList = 0;
            path->mainGrid[i][j].inOpenList = 0;
            path->mainGrid[i][j].totalCost = 0;
            path->mainGrid[i][j].costSoFar = 0;
            path->mainGrid[i][j].myRef = NULL;
        }
    }

}

static void clearPathFindingStructByClosedList(pathFindingStruct* path){

    listIterator it;
    listIteratorStart(path->closedList, &it);

    edgeRef* node;
    do{
        node = listIteratorGetFirstElement(&it);
        if (node != NULL){
            path->mainGrid[node->i][node->j].inClosedList = 0;
            path->mainGrid[node->i][node->j].inOpenList = 0;
            path->mainGrid[node->i][node->j].totalCost = 0;
            path->mainGrid[node->i][node->j].costSoFar = 0;
            path->mainGrid[node->i][node->j].myRef = NULL;
            listIteratorRemoveCurrent(&it, NULL);
            poolReturnElement(path->referencesPool, node);
        }
    } while (node != NULL);

}

static void clearPathFindingStructByOpenList(pathFindingStruct* path){

    modifiedHeapIterator it;
    modifiedHeapIteratorStart(path->openList, &it);

    edgeRef* node;
    for (node = modifiedHeapIteratorGetFirstElement(&it); node != NULL; node = modifiedHeapIteratorGetNextElement(&it)){
        path->mainGrid[node->i][node->j].inClosedList = 0;
        path->mainGrid[node->i][node->j].inOpenList = 0;
        path->mainGrid[node->i][node->j].totalCost = 0;
        path->mainGrid[node->i][node->j].costSoFar = 0;
        path->mainGrid[node->i][node->j].myRef = NULL;
        poolReturnElement(path->referencesPool, node);
    }

    modifiedHeapReset(path->openList);

}

static int absolute(int x){
    return x < 0 ? -x : x;
}

static int calculateManhattanDistance(int startingX, int startingY, int endX, int endY){
    return (absolute(endX - startingX) + absolute(endY - startingY)) * CONSTANT_STRAIGHT_COST;
}

static int calculateStraightDistance(int startingX, int startingY, int endX, int endY){

    int dx = absolute(endX - startingX);
    int dy = absolute(endY - startingY);
    if (dx >= dy){
        return (dy * CONSTANT_DIAGONAL_COST) + ((dx-dy) * CONSTANT_STRAIGHT_COST);
    }
    else{
        return (dx * CONSTANT_DIAGONAL_COST) + ((dy-dx) * CONSTANT_STRAIGHT_COST);
    }

}

static int processCell(pathFindingStruct* path, int xAdjust, int yAdjust, edgeRef* edge, edgeRef* end, const unsigned extraCost){

    if(path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].inClosedList == 1){
       return 1;
    }

    unsigned distance = path->mainGrid[edge->i][edge->j].costSoFar + extraCost;
    if (path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].inOpenList == 0 || distance < path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].costSoFar){

        path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].parent.i = edge->i;
        path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].parent.j = edge->j;

        path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].costSoFar = distance;
        path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].totalCost = path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].costSoFar + path->heuristicDistance(edge->j + xAdjust,edge->i + yAdjust, end->j, end->i);

        if (path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].inOpenList == 0){
            edgeRef* newNode = getEdgeRef(path, edge->i + yAdjust, edge->j + xAdjust, &path->mainGrid[edge->i + yAdjust][edge->j + xAdjust]);
            if (newNode == NULL || (modifiedHeapInsert(path->openList, newNode) == 0)){
                free(newNode);
                return 0;
            }

            path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].inOpenList = 1;
            path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].myRef = newNode;
        }
        else{
            modifiedHeapUpdatedValue(path->openList, path->mainGrid[edge->i + yAdjust][edge->j + xAdjust].myRef);
        }

    }

    return 1;
}


static int checkStraightCell(pathFindingStruct* path, int condition, int xAdjust, int yAdjust, char *firstReferencedCondition, char* secondReferencedCondition,
        edgeRef* edge, edgeRef* end){

    if (condition == 1){
        if(path->isWalkable(path->userGivenGrid, edge->i + yAdjust, edge->j + xAdjust) == 1){
            return processCell(path, xAdjust, yAdjust, edge, end, CONSTANT_STRAIGHT_COST);
        }
        else{
            *firstReferencedCondition = 0;
            *secondReferencedCondition = 0;
        }
    }

    return 1;
}

static int checkDiagonalCell(pathFindingStruct* path, int condition, int xAdjust, int yAdjust, edgeRef* edge, edgeRef* end){

    if ((condition == 1) && (path->isWalkable(path->userGivenGrid, edge->i + yAdjust, edge->j + xAdjust) == 1)){
        return processCell(path, xAdjust, yAdjust, edge, end, CONSTANT_DIAGONAL_COST);
    }

    return 1;

}

static inline edgeRef* findNextNode(modifiedHeap* openList){
    return modifiedHeapPopFirstElement(openList);

}

pathFindingStruct* createPathFindingStruct(void* givenGrid, unsigned maxX, unsigned maxY, int (*isWalkable)(void*, int, int)){

    pathFindingStruct* path = (pathFindingStruct*)malloc(sizeof(pathFindingStruct));
    if(path == NULL){
        return NULL;
    }

    path->closedList = listCreate();
    if (path->closedList == NULL){
        free(path);
        return NULL;
    }

    path->openList = modifiedHeapCreate(cellCompare, cellGetIndex, cellSetIndex);
    if (path->openList == NULL){
        listClear(&path->closedList, NULL);
        free(path);
        return NULL;
    }

    path->referencesPool = poolCreate(POOL_START_SIZE, sizeof(edgeRef));
    if (path->referencesPool == NULL){
        listClear(&path->closedList, NULL);
        modifiedHeapClear(&path->openList, NULL);
        free(path);
        return NULL;
    }

    path->mainGrid = (__pathfinding_info_cell**)malloc(sizeof(__pathfinding_info_cell*) * maxY);
    if (path->mainGrid == NULL){
        listClear(&path->closedList, NULL);
        modifiedHeapClear(&path->openList, NULL);
        poolClear(&path->referencesPool);
        free(path);
        return NULL;
    }

    unsigned i;
    for (i = 0; i < maxY; ++i){
        path->mainGrid[i] = (__pathfinding_info_cell*)malloc(sizeof(__pathfinding_info_cell) * maxX);
        if (path->mainGrid[i] == NULL){
            unsigned j;
            for (j = 0; j < i; ++j){
                free(path->mainGrid[j]);
            }
            listClear(&path->closedList, NULL);
            modifiedHeapClear(&path->openList, NULL);
            poolClear(&path->referencesPool);
            free(path);
            return NULL;
        }
    }

    path->maxX = maxX;
    path->maxY = maxY;

    clearPathFindingStruct(path);

    path->userGivenGrid = givenGrid;
    path->isWalkable = isWalkable;
    path->heuristicDistance = calculateStraightDistance;
    //path->heuristicDistance = calculateManhattanDistance;

    return path;
}

void pathFindingStructFree(pathFindingStruct* path){

    unsigned i;
    for (i =0; i < path->maxY; ++i){
        free(path->mainGrid[i]);
    }
    free(path->mainGrid);

    listClear(&path->closedList, free);
    modifiedHeapClear(&path->openList, free);
    poolClear(&path->referencesPool);

    free(path);
};

void setPathFindingHeuristic(pathFindingStruct* path, int (*newFunc)(int, int, int, int)){
    path->heuristicDistance = newFunc;
}

static int generatePath(pathFindingStruct* path, list* retList, int fromX, int fromY, int currentX, int currentY){

    if (fromX == currentX && fromY == currentY){
        edgeRef* newEdge = getEdgeRef(path, currentY, currentX, NULL);
        if (newEdge == NULL){
            //printf("Error, out of memory.");
            return 0;
        }
        if (listPushBack(retList, newEdge) == 0){
            free(newEdge);
            return 0;
        }

        return 1;
    }

    edgeRef* newEdge = getEdgeRef(path, currentY, currentX, NULL);
    if (newEdge == NULL){
        //printf("Error, out of memory.");
        return 0;
    }
    int newX = path->mainGrid[currentY][currentX].parent.j;
    int newY = path->mainGrid[currentY][currentX].parent.i;

    if (generatePath(path, retList, fromX, fromY, newX, newY) == 1){
        if (listPushBack(retList, newEdge) == 0){
            free(newEdge);
            return 0;
        }

    }

    return 1;

}

list* processPathFinding(pathFindingStruct* path, unsigned fromX, unsigned fromY, unsigned toX, unsigned toY){

    edgeRef* currentNode = getEdgeRef(path, fromY, fromX, &path->mainGrid[fromY][fromX]);
    if(modifiedHeapInsert(path->openList, currentNode) == 0){
        free(currentNode);
        return 0;
    }

    int pathFound = 0;
    edgeRef end;
    end.i = toY;
    end.j = toX;

    //struct timeval start, endTime;
    //gettimeofday(&start, NULL);
    while(1){

        currentNode = findNextNode(path->openList);
        if (currentNode == NULL){
            //printf("OpenList vazia... nenhum caminho encontrado...\n");
            break;
        }

        if (listPushBack(path->closedList, currentNode) == 0){
            break;
        }
        path->mainGrid[currentNode->i][currentNode->j].inOpenList = 0;
        path->mainGrid[currentNode->i][currentNode->j].inClosedList = 1;

        if (currentNode->j == end.j && currentNode->i == end.i){
            //printf("Path finished!\n");
            pathFound = 1;
            break;
        }

        char checkL=1, checkLU=1, checkU=1, checkRU=1, checkR=1, checkRD=1, checkD=1, checkLD=1;
        if (currentNode->i == 0){
            checkD  = checkLD = checkRD = 0;
        }
        if (currentNode->i == path->maxY-1){
            checkU =  checkLU = checkRU = 0;
        }
        if (currentNode->j == 0){
            checkL = checkLD = checkLU = 0;
        }
        if (currentNode->j == path->maxX-1){
            checkR = checkRD = checkRU = 0;
        }

        checkStraightCell(path, checkL, -1,  0, &checkLD, &checkLU, currentNode, &end);
        checkStraightCell(path, checkR,  1,  0, &checkRD, &checkRU, currentNode, &end);
        checkStraightCell(path, checkU,  0,  1, &checkLU, &checkRU, currentNode, &end);
        checkStraightCell(path, checkD,  0, -1, &checkLD, &checkRD, currentNode, &end);
        checkDiagonalCell(path, checkLD, -1, -1, currentNode, &end);
        checkDiagonalCell(path, checkLU, -1,  1, currentNode, &end);
        checkDiagonalCell(path, checkRU,  1,  1, currentNode, &end);
        checkDiagonalCell(path, checkRD,  1, -1, currentNode, &end);

    }
    //gettimeofday(&endTime, NULL);
    //long totalTime = (endTime.tv_sec - start.tv_sec) * 1000000 + (endTime.tv_usec - start.tv_usec);
    //printf("Total search time %lu.\n", totalTime);

    if (pathFound == 0){
        return NULL;
    }

    list* retList = listCreate();
    if(generatePath(path, retList, fromX, fromY, toX, toY) == 0){
        //printf("Error creating the path.");
        clearPathFindingStructByOpenList(path);
        clearPathFindingStructByClosedList(path);
        listClear(&retList, free);
        return NULL;
    }

    clearPathFindingStructByOpenList(path);
    clearPathFindingStructByClosedList(path);

    return retList;

}

void releasePath(pathFindingStruct* path, list* nodesList){
    listIterator it;
    listIteratorStart(nodesList, &it);
    edgeRef* node;
    for (node = listIteratorGetFirstElement(&it); node != NULL; node = listIteratorGetNextElement(&it)){
        poolReturnElement(path->referencesPool, node);
    }

    listClear(&nodesList, NULL);
}
