#include <Arduino.h>
#include <math.h>
#include "path_finder/path_finder.h"

void PathFinder::init(){
    bot_x = 0;
    bot_y = 0;
    bot_r = 1;

    for(size_t i = 0; i < TILE_ROWS; i++)
        for(size_t j = 0; j < TILE_COLS; j++) {
            map[i][j].terrain = 1;
            map[i][j].f_cost = INF_COST;
            map[i][j].g_cost = INF_COST;
            map[i][j].h_cost = INF_COST;
            map[i][j].inClosed = false;
            map[i][j].inOpen = false;
            map[i][j].parent = 5;
        }
    map[1][1].terrain = 0;
}

void PathFinder::planPath(uint8_t target_x, uint8_t target_y)
{
    const int8_t x_dir[4] = {1,0,-1,0};
    const int8_t y_dir[4] = {0,1,0,-1};

    uint8_t x = bot_x;
    uint8_t y = bot_y;

    boolean done = false;
    uint16_t count = 0;
        
    Serial.println("Planning to Target: (" + String(target_x) + ", " 
        + String(target_y) + ")");

    map[y][x].inOpen = true;
    map[y][x].g_cost = 0;
    map[y][x].h_cost = TILE_LEN * (abs(target_x - x) + abs(target_y - y));
    map[y][x].f_cost = map[y][x].g_cost + map[y][x].h_cost;
    map[y][x].parent = (bot_r + 2) % 4;

    uint16_t g_cost;
    uint16_t h_cost;

    while(!done && count < 100) {
        // find the tile with the lowest f_cost in the open set
        // TODO @Jordan Slater make this into a private function
        /* TODO @Jordan Slater make this use function prioritize the lower 
            h_cost tiles first */
        uint8_t best_x = 0;
        uint8_t best_y = 0;
        uint16_t best_f = INF_COST;
        for(size_t i = 0; i < TILE_ROWS; i++)
            for(size_t j = 0; j < TILE_COLS; j++)
                if (map[j][i].inOpen && map[j][i].f_cost < best_f) {
                    best_f = map[j][i].f_cost;
                    best_x = i;
                    best_y = j;
                }
        x = best_x;
        y = best_y;

        map[y][x].inOpen = false;
        map[y][x].inClosed = true;

        if (x == target_x && y == target_y)
            done = true;
        else {
            for(size_t dir = 0; dir < 4; dir++)
            {
                // neighbour coords
                int8_t nx = x + x_dir[dir];
                int8_t ny = y + y_dir[dir];
                if (nx <= TILE_COLS && nx >= 0 && ny <= TILE_ROWS && ny >= 0)
                    if (map[ny][nx].terrain != 0 && !map[ny][nx].inClosed)
                    {
                        //Serial.println(String(nx) + " " + String(ny));
                        
                        h_cost = TILE_LEN * (abs(target_x - nx) 
                            + abs(target_y - ny));
                        /* TODO @JordanSlater make the turning penalty 
                            proportional to the amount the bot has to turn */
                        g_cost = TILE_LEN + map[y][x].g_cost;
                        if (map[y][x].parent != (dir + 2) % 4)
                            g_cost += TURN_COST;
                        if (g_cost + h_cost < map[ny][nx].f_cost)
                        {
                            map[ny][nx].g_cost = g_cost;
                            map[ny][nx].h_cost = h_cost;
                            map[ny][nx].f_cost = g_cost + h_cost;
                            /* parent points towards the x,y tile which is the 
                                opposite direction of dir so add 2 and mod 4 to get 
                                opposite dir
                            */
                            map[ny][nx].parent = (dir + 2) % 4;
                        }
                        map[ny][nx].inOpen = true;
                    }
            }   
            
        }
        count++;
        Serial.println("count: " + String(count) + " at: (" + String(x) + ", " 
            + String(y) + ")");
        printMapParents();
        printMapFCosts();
        Serial.println();

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

