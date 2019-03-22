#include <Arduino.h>
#include <math.h>
#include <common.h>
#include "path_finder/path_finder.h"


Terrain course[6][6] = {
    {Terrain::WOOD, Terrain::WOOD, Terrain::GRAVEL, Terrain::WOOD, Terrain::WOOD, Terrain::WOOD},
    {Terrain::WOOD, Terrain::WATER, Terrain::WOOD, Terrain::WOOD, Terrain::SAND, Terrain::WOOD},
    {Terrain::WOOD, Terrain::WOOD, Terrain::SAND, Terrain::WOOD, Terrain::WOOD, Terrain::WATER},
    {Terrain::GRAVEL, Terrain::WOOD, Terrain::WOOD, Terrain::WOOD, Terrain::WOOD, Terrain::WOOD},
    {Terrain::WOOD, Terrain::SAND, Terrain::WOOD, Terrain::WOOD, Terrain::GRAVEL, Terrain::WOOD},
    {Terrain::WOOD, Terrain::WOOD, Terrain::WOOD, Terrain::WATER, Terrain::WOOD, Terrain::WOOD}
};

void PathFinder::init() {
    planned = false;
    path_populated = false;
    bot_set = false;
    target_set = false;

    randomSeed(analogRead(0));

    // Initialize the map
    clearPlan();
    for(size_t i = 0; i < TILE_ROWS; i++)
        for(size_t j = 0; j < TILE_COLS; j++)
            #ifdef USE_KNOWN_MAP
                map[i][j].terrain = course[i][j];
            #else
                map[i][j].terrain = Terrain::UNKNOWN; //getRandomTerrain();
            #endif
}

void PathFinder::clearPlan() {
    for(size_t i = 0; i < TILE_ROWS; i++)
        for(size_t j = 0; j < TILE_COLS; j++) {
            map[i][j].f_cost = INF_COST;
            map[i][j].g_cost = INF_COST;
            map[i][j].h_cost = INF_COST;
            map[i][j].inClosed = false;
            map[i][j].inOpen = false;
            // parrent value to show that the tile has not been evaluated yet
            map[i][j].parent = 5;
        }
}

void PathFinder::setTargetPosition(uint8_t x, uint8_t y) {
    target_x = x;
    target_y = y;
    planned = false;
    target_set = true;
}

void PathFinder::setBotPosition(uint8_t x, uint8_t y, uint8_t r) {
    bot_x = x;
    bot_y = y;
    bot_r = r;
    planned = false;
    bot_set = true;
}

void PathFinder::findNext(uint8_t & best_x, uint8_t & best_y) {
    // If needed optimize this function to prioritize the lowest h_cost as well
    best_x = 0;
    best_y = 0;
    uint16_t best_f = INF_COST;
    // search the map
    for(size_t i = 0; i < TILE_ROWS; i++)
        for(size_t j = 0; j < TILE_COLS; j++)
            if (map[j][i].inOpen && map[j][i].f_cost < best_f) {
                best_f = map[j][i].f_cost;
                best_x = i;
                best_y = j;
            }
}

bool PathFinder::planPath(int8_t unknown_cost) {
    if (!bot_set || !target_set) {
        DEBUG_PRINT("Set bot and target before planning! ");
        return false;
    }

    clearPlan();
    path.clear();

    planned = false;
    /* the tile (x,y) is the current tile that is being evaluated in the 
        algorithm, start at the starting position of the bot */
    uint8_t x = bot_x;
    uint8_t y = bot_y;
    uint16_t count = 0; // number of steps the algorithm has taken
    
    DEBUG_PRINT("Planning from position: (" + String(bot_x) + "," + String(bot_y) + ")");
    DEBUG_PRINT("Planning to Target: (" + String(target_x) + ", " + String(target_y) + ")");

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
    while(count < MAX_ALGORITHM_STEPS) {
        findNext(x,y);

        // we are going to evaluate the tile so it is now in the closed set
        map[y][x].inOpen = false;
        map[y][x].inClosed = true;

        if (x == target_x && y == target_y)
        {
            DEBUG_PRINT("Done planning path");
            planned = true;
            break;
        } else {
            // evaluate at each of the cardinal neighbours of the current tile
            for(size_t dir = 0; dir < 4; dir++)
            {
                // get the coords of the neighbouring tile
                int8_t nx = x + X_DIR[dir];
                int8_t ny = y + Y_DIR[dir];
                // if the neighbour is in map bounds
                if (nx < TILE_COLS && nx >= 0 && ny < TILE_ROWS && ny >= 0)
                    // it's not in closed
                    if (!map[ny][nx].inClosed) // && map[ny][nx].terrain != WATER
                    {
                        // compute the h_cost if the bot was to move to the tile
                        h_cost = TILE_COST * (abs(target_x - nx) 
                            + abs(target_y - ny));
                        if (map[ny][nx].terrain == Terrain::GRAVEL)
                            h_cost+= GRAVEL_COST;
                        else if (map[ny][nx].terrain == Terrain::SAND)
                            h_cost+= SAND_COST;
                        else if (map[ny][nx].terrain == Terrain::WATER)
                            h_cost+= WATER_COST;
                        else if (map[ny][nx].terrain == Terrain::UNKNOWN)
                            h_cost+= unknown_cost;
                        // compute the g_cost of getting to the tile
                        g_cost = TILE_COST + map[y][x].g_cost;
                        // if the bot has to turn
                        if (map[y][x].parent != (dir + 2) % 4) {
                            g_cost += TURN_COST;
                            if (map[y][x].terrain == Terrain::SAND || map[y][x].terrain == Terrain::GRAVEL) {
                                g_cost += SAND_TURN_COST;
                            } 
                        }
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
        
        // TODO @JordanSlater change to DEBUG print statements
        // DEBUGGING CODE
        DEBUG_PRINT("count: " + String(count) + " at: (" + String(x) + ", " 
            + String(y) + ")");
        printMapParents();
        //printMapFCosts();        
    }
    
    if (!planned) {
        DEBUG_PRINT("Ran out of steps");
        return false;
    }

    // work backwards from the target tile to the bot's tile
    x = target_x;
    y = target_y;

    // the parent of the previous tile
    uint8_t prev_parent;
    plan_steps = 0; 
    while(plan_steps < MAX_PLAN_SIZE)
    {
        // found the bot's tile
        if (x == bot_x && y == bot_y) {
            path_populated = true;
            copyPlanToPath();

            Serial.println("Steps: " + String(plan_steps));
            String s = "";
            for(int8_t i = plan_steps - 1; i >= 0; i--)
                s = s + String(plan[i]) + ",";
            DEBUG_PRINT(s);
            return true;
        }
        Terrain prev_terrain = map[y][x].terrain;
        // store the previous parent
        prev_parent = map[y][x].parent;
        // move backward in the path
        x+= X_DIR[prev_parent];
        y+= Y_DIR[prev_parent];
        // make sure the new tile is in the map
        if (x < 0 || x >= TILE_COLS || y < 0 || y >= TILE_ROWS) {
            DEBUG_PRINT("Error while creating plan, out of bounds at step: " + String(plan_steps));
            return false;
        }
        if (prev_terrain != Terrain::WATER) {
            // add a forward move to the path
            plan[plan_steps++] = 0;
            path.push(Move{Move_t::FORWARD, 30});
        } else {
            plan[plan_steps++] = 2;
            path.push(Move{Move_t::MOVE_ONTO_WATER, 30}); // get actual distance to move one tile
        }

        // if the bot has changed orientation since the last tile
        if (prev_parent != map[y][x].parent) {
            if ((map[y][x].parent + 1) % 4 == prev_parent) {
                // the bot turned left
                plan[plan_steps++] = 1;
            }
            else if ((prev_parent + 1) % 4 == map[y][x].parent) {
                // the bot turned right
                plan[plan_steps++] = -1;
            }
            else { // the parent tile points back at the current tile
				if (x == bot_x && y == bot_y) {
					plan[plan_steps++] = 1;
					plan[plan_steps++] = 1;
				} else {
					DEBUG_PRINT("Error while creating plan, repeat tile at step: " 
						+ String(plan_steps));
					return false;
				}
            }
        }
    }
    // path is too long for path array
    path_populated = false;
    return false;
}

// Copy the plan array into the path, flattening multiple move commands into a single command
void PathFinder::copyPlanToPath() {
    uint8_t num_forward_moves = 0;
    path.clear();

    for (int i=0; i<plan_steps; i++) {
        if (plan[i] != 0) {
            if (num_forward_moves != 0) {
                DEBUG_PRINT("Added move with turn");
                path.push(Move{FORWARD, 30*num_forward_moves});
                num_forward_moves = 0;
            }
            DEBUG_PRINT("Added turn");
            if (plan[i] == 1) {
                path.push(Move{TURN_LEFT, 90});
            } else if (plan[i] == -1) {
                path.push(Move{TURN_RIGHT, -90});
            }
        }
        else if (plan[i] == 0) {
            num_forward_moves++;
        }
    }
    if (num_forward_moves != 0) {
        path.push(Move{FORWARD, 30*num_forward_moves});
        DEBUG_PRINT("Added move " + String(30*num_forward_moves));
    }
    DEBUG_PRINT(path.size());
}


Terrain PathFinder::getRandomTerrain() {
    return TILE_PROBABILITY[(uint8_t)(random(0,36))];
}

bool PathFinder::setTerrain(uint8_t x, uint8_t y, Terrain terrain) {
    if (!(x < TILE_COLS && y < TILE_ROWS))
        return false;
    map[y][x].terrain = terrain;
    return true;
}

Terrain PathFinder::getTerrain(uint8_t x, uint8_t y) {
    if (!(x < TILE_COLS && y < TILE_ROWS))
        return Terrain::ERROR;
    return map[y][x].terrain;
}

void PathFinder::printMapTerrain() {
    String s = "\n";
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + (uint8_t)map[i][j].terrain + ",";
        s = s + "\n";
    }
    DEBUG_PRINT(s);
}

void PathFinder::printMapParents() {
    String s = "\n";
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + map[i][j].parent + ",";
        s = s + "\n";
    }
    DEBUG_PRINT(s);
}

void PathFinder::printMapFCosts() {
    String s = "\n";
    for(int8_t i = TILE_ROWS - 1; i >= 0; i--) {
        for(size_t j = 0; j < TILE_COLS; j++)
            s = s + map[i][j].f_cost + ",";
        s = s + "\n";
    }
    DEBUG_PRINT(s);
}
