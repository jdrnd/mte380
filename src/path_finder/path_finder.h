#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#define TILE_ROWS 6
#define TILE_COLS 6
#define INF_COST 65535
#define MAX_PLAN_SIZE 72
#define MAX_ALGORITHM_STEPS 1000

#define WATER 0
#define WOOD 1
#define GRAVEL 2
#define SAND 3
#define DISCOVERED 4
#define UNKNOWN 5

#define TILE_COST 10
#define TURN_COST 6 // (approximately 0.524 of the TILE_COST)
#define SAND_COST 5 // in addition to the base tile cost
#define GRAVEL_COST 2 // again in addition
#define UNKNOWN_COST 1

/* NOTE: all rotation and parent directions are encoded as 0,1,2,3
    the integer corresponds to that many multiples of 90 degrees counter 
    clockwise from the x axis. So: 
        0 - right
        1 - up
        2 - left
        3 - down
*/

class PathFinder {
    /*  When generating a random tile (for testing) a tile is randomly selected
        from this terrain distribution */
    const uint8_t TILE_PROBABILITY[36] = {
        GRAVEL,GRAVEL,GRAVEL,GRAVEL,WOOD,WOOD,
        SAND,SAND,SAND,SAND,WOOD,WOOD,
        WOOD,WOOD,WOOD,WOOD,WOOD,WOOD,
        WOOD,WOOD,WOOD,WOOD,WOOD,WOOD,
        WOOD,WOOD,WOOD,WOOD,WOOD,WOOD,
        WOOD,WOOD,WOOD,WOOD,WOOD,WOOD,
    };

    /* If true the map has been populated with parent values from the bot to 
        target. */
    bool planned;
    /* If true the path array has been populated with instructions from the bot 
        to the target. */
    bool path_populated;
    // If true then position of the bot has been set
    bool bot_set;
    // If true then the position of the target has been set
    bool target_set;
    // The starting x position of the bot
    uint8_t bot_x;
    // The starting y position of the bot
    uint8_t bot_y;
    // The starting rotation of the bot
    uint8_t bot_r;
    // The x target of the bot
    uint8_t target_x;
    // The y target of the bot
    uint8_t target_y;
    /* The plan to move from the current bot position to the target. 
        Note: the plan is populated in reverse order */
    int8_t plan[MAX_PLAN_SIZE];
    // the number of steps in the plan
    uint8_t plan_steps;

    // Stores all data required to plan a path
    struct tile {
        /*  0: water,
            1: wood,
            2: gravel,
            3: sand, 
            4: unknown but will be discovered once the path is executed,
            5: unknown undiscovered */
        uint8_t terrain; // the type of terrain in the tile
        /* The distance from the tile to the target plus the cost of progressing
            through the tile */
        uint16_t h_cost;
        // The cost of getting to the tile from the start tile
        uint16_t g_cost;
        // g_cost + h_cost
        uint16_t f_cost;
        // true if the tile has been discovered and assigned a cost
        bool inOpen;
        // true if the tile has been evaluated
        bool inClosed;
        /* the direction from which the cost was calculated, i.e. points to the previous tile */
        uint8_t parent;
    };

    // map of all tiles on the field
    tile map[TILE_ROWS][TILE_ROWS];

    // Holds the x component of the direction vector
    const int8_t X_DIR[4] = {1,0,-1,0};

    // Holds the y component of the direction vector
    const int8_t Y_DIR[4] = {0,1,0,-1};

    /* Reads the parrent values of the map to create a path from bot to target*/
    bool createPath();

    public:
        void init();
        void printMapTerrain();
        void printMapParents();
        void printMapFCosts();
        // Generates a random terrain value
        uint8_t getRandomTerrain();
        // Sets the starting position of the bot
        void setBotPosition(uint8_t x, uint8_t y, uint8_t r);
        // Sets the target position
        void setTargetPosition(uint8_t x, uint8_t y);
        /* Uses A* path finding to plan a path from the bot's location to the 
            target x and target y. Populates the parrent values that are used by
            createPath() to actually create the path. Returns true if the target
            was reached. */
        bool planPath();
        /* Retrieves the path array if it contains valid information. 
            success is true if the path array was actually returned otherwise 
            the pointer is null. steps is the number of entries in the path 
            array. Returns null if path was invalid, returns the path array 
            otherwise. Notes on path array: 
                0: move one tile forward, 
                -1: turn 90 degrees right, 
                1: turn 90 degrees left. */
        int8_t* retrievePlan(bool & success, uint8_t & steps);
}; // class PathFinder

#endif // PATH_FINDER_H