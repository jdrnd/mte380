#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#define TILE_ROWS 3
#define TILE_COLS 3
#define INF_COST 65535
#define TILE_LEN 10
#define TURN_COST 2

class PathFinder {

    uint8_t bot_x;
    uint8_t bot_y;
    uint8_t bot_r;

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

    tile map[TILE_ROWS][TILE_COLS];
    
    public:
        void init();
        void printMapTerrain();
        void printMapParents();
        void printMapFCosts();
        void planPath(uint8_t target_x, uint8_t target_y);
}; // class PathFinder

#endif // PATH_FINDER_H