/*
 * 
 */
package PathPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

/**
 *
 * @author Matt Cook
 * 
 * Class: RandomPlanner
 * 
 * Description: This class is used for the randomized path planning. That is,
 *  it outputs a path by picking contiguous cells randomly.
 **/ 
public class RandomPlanner {
    
    /* 
     * Method: Random
     * 
     * Input: OceanCell that is the start, OceanGrid to path plan on, and a 
     *  double for the max mission length
     * 
     * Output: OceanPath that is the random path created
     * 
     * Details: This path planning algorithm picks a random neighbor to 
     *  travel to, although it does not allow it to return to the cell 
     *  it just came from. This is to provide more interesting results, as 
     *  going back to the same cell immediately would not give much reward. 
     *  Note that this does not mean it cannot revisit cells, as it still can
     *  if the path made a loop around.
     * 
     */
    public static OceanPath Random(OceanCell start, OceanGrid grid, double missionLength) {
        // generator for randomly picking neighbors
        Random generator = new Random();
        
        // initialize the correct start time and time interval
        double timeInterval = Planner.timeInterval;
        double startTime = Planner.hourStartIndex*timeInterval;
        double maxMissionTime = startTime + missionLength;   
        
        // initialize start of path
        OceanPath randomPath = new OceanPath();
        randomPath.add(start);
        randomPath.timeElapsed = startTime;
        
        boolean timeLeft = true;
        while (timeLeft) {
            OceanCell currentCell = randomPath.get(randomPath.size()-1);
            if (randomPath.size() > 1) {
                int time = (int)Math.floor(randomPath.timeElapsed / timeInterval);
                // if we ran out of time data, just use latest time data
                if (time > Planner.hourEndIndex) {
                    System.out.println("Need more time data! Path Planning"
                            + " will just use data from latest timestep");
                    time = Planner.hourEndIndex;
                }
                // if we moved onto the next timestep 
                if (time > randomPath.get(randomPath.size()-2).getTime()) {
                    OceanCell sameCell = grid.getCell(time, 
                            currentCell.getDepth(), currentCell.getLat(),
                            currentCell.getLon());
                    sameCell.copyData(currentCell);
                    currentCell = sameCell;
                }  
            } 
            // record current path if tracing
            if (Planner.mathematica) {
                Planner.recordInstance(randomPath.path, false, grid);
            }
            
            int t = currentCell.getTime();
            int z = currentCell.getDepth();
            int x = currentCell.getLat();
            int y = currentCell.getLon();         
            
            ArrayList<OceanCell> neighbors = new ArrayList<OceanCell>();
            for (int dir = 0; dir < Planner.numDirections; dir++) {
                int dirx = Planner.directions[dir * 2];
                int diry = Planner.directions[dir * 2 + 1];
                int newx = x + dirx;
                int newy = y + diry;
                
                // If heading toward a feasible location
                if ((newy >= 0) && (newy < grid.getLonLength())
                        && (newx >= 0) && (newx < grid.getLatLength())) {            
                    // since location is within boundaries, check this neighbor
                    OceanCell neighbor = grid.getCell(t,z,newx,newy);
                    double timeTaken = AUV.travelTime(currentCell, neighbor);

                    // impossible to reach cell, skip to next neighbor
                    if (timeTaken < 0 || !neighbor.validCell) {
                        continue;
                    }
                    // skip if we are going back to same cell
                    if (randomPath.size() > 1) {
                        OceanCell prevCell = 
                                randomPath.get(randomPath.size()-2);
                        if (neighbor.equals(prevCell)) {
                            continue;
                        }
                    }
                    // if we can reach neighbor in time, add it to the 
                    // list of possible neighbors
                    if ((randomPath.timeElapsed+timeTaken) <= maxMissionTime) {
                        neighbors.add(neighbor);
                    }  
                }
            }
            // if there are no neighbors, that means there is no time left
            // to reach a new cell
            if (neighbors.isEmpty()) {
                timeLeft = false;
            }
            else {
                // pick a random neighbor from list of possible neighbors
                int randomIndex = generator.nextInt(neighbors.size());
                OceanCell randomNeighbor = neighbors.get(randomIndex);
                // update information about path
                double timeTaken = AUV.travelTime(currentCell, randomNeighbor);
                randomPath.gScore += 
                        AStarPlanner.addObjective(randomPath, randomNeighbor);
                randomPath.add(randomNeighbor);
                randomPath.timeElapsed += timeTaken;
                randomPath.fScore = randomPath.gScore;
            }
        }
    // record final path if tracing in mathematica    
    if(Planner.mathematica) {
        Planner.recordInstance(randomPath.path, true, grid);
        Planner.recordHistory(new File(Planner.historyFile));
    }
    // fill in cells on path with accurate information
    randomPath.recordData(grid, maxMissionTime);
    return randomPath;
    }
}
