/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import static PathPlanner.AStarPlanner.addObjective;
import java.io.File;
import java.util.ArrayList;
import java.util.Random;

/**
 *
 * @author atthewco
 */
public class RandomPlanner {
    
    /* 
     * Random Search Algorithm picks a random neighbor to travel to, although
     * it does not allow it to return to the cell it just came from.
     * 
     */
    public static OceanPath Random(OceanCell start, OceanGrid grid, double missionLength) {
        
        // Print out the net distance to get an idea of how far the end is from
        // the start location
        //double netDistance = AUV.distance(xStart, yStart, xEnd, yEnd, 'K');
        //netDistance *= 1000;
        //System.out.println("The net distance is " + netDistance + " meters");
        boolean record = Planner.mathematica;
        Random generator = new Random();
        
        double timeInterval = Planner.timeInterval;
        double startTime = Planner.hourStartIndex*timeInterval;
        double maxMissionTime = startTime + missionLength;
        
        
        OceanPath randomPath = new OceanPath();
        randomPath.add(start);
        randomPath.latestTime = startTime;

        //initialize what the maximum delta is to -1
        //double maxDelta = -1;
        // counter for debugging
        //int count = 0;
        
        boolean neighborsLeft = true;
        while (neighborsLeft) {
            //OceanCell currentCell = (OceanCell) Q.dequeue();
            OceanCell currentCell = randomPath.get(randomPath.size()-1);
            //System.out.println(randomPath);
            
            if (randomPath.size() > 1) {
                int time = (int)Math.floor(randomPath.latestTime / timeInterval);
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
            
            if (record) {
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
                    
                    
                    if ((randomPath.latestTime + timeTaken) <= maxMissionTime) {
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
                neighborsLeft = false;
            }
            else {
                int randomIndex = generator.nextInt(neighbors.size());
                //System.out.println("chose " + randomIndex + " out of " + neighbors.size());
                OceanCell randomNeighbor = neighbors.get(randomIndex);
                double timeTaken = AUV.travelTime(currentCell, randomNeighbor);
                randomPath.gScore += AStarPlanner.addObjective(randomPath, randomNeighbor);
                randomPath.add(randomNeighbor);
                randomPath.latestTime += timeTaken;
                randomPath.fScore = randomPath.gScore;
            }
        }
    if(record) {
    Planner.recordInstance(randomPath.path, true, grid);
    Planner.recordHistory(new File(Planner.historyFile));
    }
    randomPath.recordData(grid);
    return randomPath;
    }
}
