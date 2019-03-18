#include <Arduino.h>

#include "path_finder/path_finder.h"
#include "path_finder/examples/path_finder_demo.h"

/*
void doPathFinderDemo() {
    Serial.println("\n   PATH FINDER DEMO");
    
    PathFinder pathFinder;
    pathFinder.init();
    pathFinder.setBotPosition(0,0,0);
    if(!pathFinder.setTerrain(3,0,WATER))
        Serial.println("Could not set terrain");
    if(pathFinder.getTerrain(3,0) == ERROR_TERRAIN 
    || pathFinder.getTerrain(3,0) != WATER)
        Serial.println("Could not get terrain");
    pathFinder.printMapTerrain();
    pathFinder.setTargetPosition(TILE_COLS - 1, TILE_ROWS - 1);

    pathFinder.planPath();
    pathFinder.printMapParents();
    bool success = true;
    uint8_t steps = 0;
    int8_t* plan = pathFinder.retrievePlan(success,steps);
    if (success)
    {
        Serial.println("Steps: " + String(steps));
        String s = "";
        for(size_t i = 0; i < steps; i++)
            s = s + String(plan[i]) + ",";
        Serial.println(s);
    }
    //pathFinder.printMapFCosts();
}
*/
