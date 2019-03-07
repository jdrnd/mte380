#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#define TILE_ROWS 6
#define TILE_COLS 6
#define INF_COST 65535
#define TILE_COST 10
#define TURN_COST 6 // (approximately 0.524 of the TILE_COST)

/* NOTE: all rotation and parent directions are encoded as 0,1,2,3
    the integer corresponds to that many multiples of 90 degrees counter 
    clockwise from the x axis. So: 
        0 - right
        1 - up
        2 - left
        3 - down
*/

class PathFinder {
    uint8_t bot_x; // the starting x position of the bot
    uint8_t bot_y; // the starting y position of the bot
    uint8_t bot_r; // the starting rotation of the bot

    struct tile {
        uint8_t terrain; // the type of terrain in the tile
        /* the distance from the tile to the target plus the cost of progressing
            through the tile */
        uint16_t h_cost;
        // the cost of getting to the tile from the start tile
        uint16_t g_cost;
        uint16_t f_cost; // g_cost + h_cost
        bool inOpen; // true if the tile has been discovered and assigned a cost
        bool inClosed; // true if the tile has been evaluated
        uint8_t parent; // the direction from which the cost was calculated
    };

    tile map[TILE_ROWS][TILE_COLS]; // declare the map of tiles
    
    public:
        void init();
        void printMapTerrain();
        void printMapParents();
        void printMapFCosts();
        // plans a path from the bot's location to the target x and target y
        void planPath(uint8_t target_x, uint8_t target_y);
        // TODO @JordanSlater add functions to set the bot and target positions
        // as well as a function to retrieve the path. 
        
}; // class PathFinder

#endif // PATH_FINDER_H