#include <Arduino.h>
#include <math.h>
#include "path_finder/path_finder.h"

void PathFinder::init(){
    // test starting positions
    bot_x = 3;
    bot_y = 3;
    bot_r = 1;

    // Initialize the map
    for(size_t i = 0; i < TILE_ROWS; i++)
        for(size_t j = 0; j < TILE_COLS; j++) {
            map[i][j].terrain = 1;
            map[i][j].f_cost = INF_COST;
            map[i][j].g_cost = INF_COST;
            map[i][j].h_cost = INF_COST;
            map[i][j].inClosed = false;
            map[i][j].inOpen = false;
            // parrent value to show that the tile has not been evaluated yet
            map[i][j].parent = 5;
        }
    // test terrain
    map[5][1].terrain = 0;
    map[4][3].terrain = 0;
    map[1][1].terrain = 0;
}

void PathFinder::planPath(uint8_t target_x, uint8_t target_y)
{
    // TODO @JordanSlater make these static class constants
    const int8_t x_dir[4] = {1,0,-1,0};
    const int8_t y_dir[4] = {0,1,0,-1};
    /* the tile (x,y) is the current tile that is being evaluated in the 
        algorithm, start at the starting position of the bot */
    uint8_t x = bot_x;
    uint8_t y = bot_y;
    boolean done = false; // true when a plan has been found
    uint16_t count = 0; // number of steps the algorithm has taken
    
    // TODO @JordanSlater rename this to DEBUG
    Serial.println("Planning to Target: (" + String(target_x) + ", " 
        + String(target_y) + ")");

    // evaluate the starting tile
    map[y][x].inOpen = true;
    map[y][x].g_cost = 0; // no cost to get to the starting tile
    /* because the bot can only move in cardinal directions the cost to get to 
        The target tile is just the sum of the dimentional differences */
    map[y][x].h_cost = TILE_COST * (abs(target_x - x) + abs(target_y - y));
    map[y][x].f_cost = map[y][x].g_cost + map[y][x].h_cost;
    /* The 'parent' of the start tile is behind the bot. This is later used to
        calculate the turn cost */
    map[y][x].parent = (bot_r + 2) % 4;
    // temp variables
    uint16_t g_cost;
    uint16_t h_cost;

    /* iterate through the algorithm until a path is found or there are too many
        steps */
    while(!done && count < 100) {
        // find the tile with the lowest f_cost in the open set
        // TODO @Jordan Slater make this into a private function
        /* TODO @Jordan Slater make this use function prioritize the lower 
            h_cost tiles first */
        uint8_t best_x = 0;
        uint8_t best_y = 0;
        uint16_t best_f = INF_COST;
        // search the map
        for(size_t i = 0; i < TILE_ROWS; i++)
            for(size_t j = 0; j < TILE_COLS; j++)
                if (map[j][i].inOpen && map[j][i].f_cost < best_f) {
                    best_f = map[j][i].f_cost;
                    best_x = i;
                    best_y = j;
                }
        x = best_x;
        y = best_y;

        // we are going to evaluate the tile so it is now in the closed set
        map[y][x].inOpen = false;
        map[y][x].inClosed = true;

        if (x == target_x && y == target_y)
            done = true; // we found the target tile
        else {
            // evaluate at each of the cardinal neighbours of the current tile
            for(size_t dir = 0; dir < 4; dir++)
            {
                // neighbour coords
                // TODO @JordanSlater replace these with math
                int8_t nx = x + x_dir[dir];
                int8_t ny = y + y_dir[dir];
                // if the neighbour is in map bounds
                if (nx < TILE_COLS && nx >= 0 && ny < TILE_ROWS && ny >= 0)
                    // is the tile isn't water and it's not in closed
                    if (map[ny][nx].terrain != 0 && !map[ny][nx].inClosed)
                    {
                        // compute the h_cost if the bot was to move to the tile
                        h_cost = TILE_COST * (abs(target_x - nx) 
                            + abs(target_y - ny));
                        // same for g_cost
                        g_cost = TILE_COST + map[y][x].g_cost;
                        // if the bot has to turn
                        if (map[y][x].parent != (dir + 2) % 4)
                            g_cost += TURN_COST;
                        /* if the resulting f_cost is better than the f_cost 
                            currently in the neighbour tile */
                        if (g_cost + h_cost < map[ny][nx].f_cost)
                        {
                            // update the costs of the neighbour tile
                            map[ny][nx].g_cost = g_cost;
                            map[ny][nx].h_cost = h_cost;
                            map[ny][nx].f_cost = g_cost + h_cost;
                            /* parent points towards the x,y tile which is the 
                                opposite direction of dir so add 2 and mod 4
                            */
                            map[ny][nx].parent = (dir + 2) % 4;
                        }
                        // the tile has now been evaluated
                        map[ny][nx].inOpen = true;
                    }
            }
        }
        count++;
        /*
        // TODO @JordanSlater change to DEBUG print statements
        // DEBUGGING CODE
        Serial.println("count: " + String(count) + " at: (" + String(x) + ", " 
            + String(y) + ")");
        printMapParents();
        //printMapFCosts();
        Serial.println();
        */
    }
    if (done)
        Serial.println("Done planning path");
    else
        Serial.println("Ran out of steps");
}

void PathFinder::printMapTerrain() {
    String s;
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + map[i][j].terrain + ",";
        s = s + "\n";
    }
    Serial.print(s);
}

void PathFinder::printMapParents() {
    String s;
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + map[i][j].parent + ",";
        s = s + "\n";
    }
    Serial.print(s);
}

void PathFinder::printMapFCosts() {
    String s;
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + map[i][j].f_cost + ",";
        s = s + "\n";
    }
    Serial.print(s);
}

