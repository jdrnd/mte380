#ifndef TERRAIN_H
#define TERRAIN_H

enum class Terrain: uint8_t {
    GRAVEL = 0,
    WOOD = 1,
    SAND = 2,
    WATER = 3,
    UNKNOWN = 10,
    ERROR = 20,
}; 

#endif