/*
 * 
 */
package PathPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

/**
 *
 * @author atthewco
 */
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
     *  it just came from.
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
            //OceanCell currentCell = (OceanCell) Q.dequeue();
            OceanCell currentCell = randomPath.get(randomPath.size()-1);
            //System.out.println(randomPath);
            
            if (randomPath.size() > 1) {
                int time = (int)Math.floor(randomPath.timeElapsed / timeInterval);
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
            
            if (Planner.mathematica) {
                Planner.recordInstance(randomPath.path, false, grid);
            }
           

            int t = currentCell.getTime();
            int z = currentCell.getDepth();
            int x = currentCell.getLat();
            int y = currentCell.getLon();         
            
            ArrayList<OceanCell> neighbors = new ArrayList<>();
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
                    if (timeTaken < 0) {
                        continue;
                    }
                    // skip if we are going back to same cell
                    if (randomPath.size() > 1) {
                        if (neighbor.equals(randomPath.get(randomPath.size()-2))) {
                            continue;
                        }
                    }
                    
                    
                    if ((randomPath.timeElapsed + timeTaken) <= maxMissionTime) {
                        /*
                        ArrayList<OceanCell> newPath = new ArrayList<>();
                        for (int i = 0; i < currentPath.size(); ++i) {
                            OceanCell copy = new OceanCell(currentPath.get(i));
                            newPath.add(copy);
                        }
                        */
                        neighbors.add(neighbor);
                    }  
                }
            }
            
            if (neighbors.isEmpty()) {
                timeLeft = false;
            }
            else {
                int randomIndex = generator.nextInt(neighbors.size());
                //System.out.println("chose " + randomIndex + " out of " + neighbors.size());
                OceanCell randomNeighbor = neighbors.get(randomIndex);
                double timeTaken = AUV.travelTime(currentCell, randomNeighbor);
                randomPath.gScore += AStarPlanner.addObjective(randomPath, randomNeighbor);
                randomPath.add(randomNeighbor);
                randomPath.timeElapsed += timeTaken;
                randomPath.fScore = randomPath.gScore;
            }
        }
    if(Planner.mathematica) {
    Planner.recordInstance(randomPath.path, true, grid);
    Planner.recordHistory(new File(Planner.historyFile));
    }
    randomPath.recordData(grid);
    return randomPath;
    }
}
