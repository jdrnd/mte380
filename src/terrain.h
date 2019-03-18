#ifndef TERRAIN_H
#define TERRAIN_H

// 0 -> grav, 1 -> water, 2 -> wood, 3-> sand
enum class Terrain: uint8_t {
    GRAVEL = 0,
    WOOD = 1,
    SAND = 2,
    WATER = 3,
    UNKNOWN = 10,
    ERROR = 20,
}; 

#endif