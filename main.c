#include <stdio.h>
#include <string.h>

#include "pathfinding.h"

#define MAIN_GRID_SIZE 8000

int len, lines;
static int walkable_interface(void* pt, int y, int x){
    if (y < 0 || x < 0 || y >= lines || x >= len){
        return 0;
    }
    unsigned char** matrix = (unsigned char**)pt;
    return (int)matrix[y][x];
}

static void printPath(unsigned char** mainGrid, list* l){

    listIterator it;
    listIteratorStart(l, &it);
    edgeRef* pathEdge;

    for (pathEdge = listIteratorGetFirstElement(&it); pathEdge != NULL; pathEdge = listIteratorGetNextElement(&it)){
        mainGrid[pathEdge->i][pathEdge->j] = 2;
    }

    int i, j;
    for (i = 0; i < lines; ++i){

        for (j = 0; j < len; ++j){
            if (mainGrid[i][j] == 0){
                printf("0");
            }
            else if (mainGrid[i][j] == 1){
                printf(".");
            }
            else if (mainGrid[i][j] == 2){
                printf("x");
            }
        }
        printf("\n");
    }

    listIteratorReset(&it);
    for (pathEdge = listIteratorGetFirstElement(&it); pathEdge != NULL; pathEdge = listIteratorGetNextElement(&it)){
        mainGrid[pathEdge->i][pathEdge->j] = 1;
    }

}

int main(int argc, char* argv[]){
    (void)argc;
    (void)argv;

    char line[MAIN_GRID_SIZE];
    edgeRef begin, end;

    int i = 0, j;
    FILE* mapFile = fopen("G:\\projects\\Astar\\bin\\Debug\\input05.txt", "r");
    if (mapFile == NULL){
        printf("Error openning file.\n");
        return 1;
    }

    unsigned char** mainGrid = (unsigned char**)malloc(MAIN_GRID_SIZE*sizeof(unsigned char*));
    for (j = 0; j < MAIN_GRID_SIZE; ++j){
        mainGrid[j] = (unsigned char*)malloc(MAIN_GRID_SIZE*sizeof(unsigned char));
    }

    int beginSet = 0, endSet = 0;
    while(fgets(line, sizeof(line), mapFile)){
        //printf("%s", line);
        len = strlen(line);
        j = 0;
        while(line[j] != '\n' && line[j] != '\0'){
            switch(line[j]){

                case '.':
                    mainGrid[i][j] = 1;
                break;

                case 's':
                    mainGrid[i][j] = 1;
                    begin.i = i;
                    begin.j = j;
                    beginSet = 1;

                break;
                case 'e':
                    mainGrid[i][j] = 1;
                    end.i = i;
                    end.j = j;
                    endSet = 1;
                break;

                default:
                    mainGrid[i][j] = 0;
                break;


            }
            j++;
        }
        i++;
    }
    lines = i;
    fclose(mapFile);
    len--;

    if (beginSet == 0  || endSet == 0){
        printf("Invalid entry file.\n");
        return 0;
    }

    pathFindingStruct* path = createPathFindingStruct(mainGrid, len, lines, walkable_interface);
    if (path == NULL){
        printf("Error.");
        return 0;
    }

    list* pathTo = processPathFinding(path, begin.j, begin.i, end.j, end.i);
    if (pathTo == NULL){
        pathFindingStructFree(path);
        for (j = 0; j < MAIN_GRID_SIZE; ++j){
            free(mainGrid[j]);
        }
        free(mainGrid);
        return 0;
    }
    printPath(mainGrid, pathTo);
    releasePath(path, &pathTo);

    pathFindingStructFree(path);
    for (j = 0; j < MAIN_GRID_SIZE; ++j){
        free(mainGrid[j]);
    }
    free(mainGrid);

    return 0;
}
