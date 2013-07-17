/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;

/**
 *
 * @author atthewco
 */
public class AStarPlanner {
        
    public static double objectiveEstimate(OceanPath currentPath, OceanGrid grid) {
        /*
        double heuristicRate = 0;
        OceanCell currentCell = currentPath.get(currentPath.size()-1);
        
        ArrayList<OceanCell> neighbors = Planner.findNeighbors(currentCell, grid, 2);
        for (int i = 0; i < neighbors.size(); ++i) {
            OceanCell neighbor = neighbors.get(i);
            if (currentPath.contains(neighbor)) {
                heuristicRate -= neighbor.getTempErr();
            }
            else {
                heuristicRate += neighbor.getTempErr();
            }
        }
        */
        //heuristicRate = (heuristicRate/neighbors.size()) * Planner.weighting;
        
        //System.out.println(heuristicRate);
        double timeLeft = Planner.missionLength - currentPath.latestTime + (Planner.hourStartIndex*Planner.timeInterval);
        //System.out.println("time left is " + timeLeft);
        double predCells = (currentPath.size()/currentPath.latestTime)*timeLeft;
        double heuristicRate = (0.2289 * Planner.weighting);
        double predScore = heuristicRate * predCells;
        //System.out.println("predicted score left is " + predScore);
        return predScore;
    }
    
    public static double addObjective(OceanPath currentPath, OceanCell neighbor) {
        double tempScore = neighbor.getTempErr();

        // lower score added if cell was visited recently.
        // receive full reward if cell has not been visited within 12 hours
        // NOTE: this is a compounding decay, which means it can go negative
        // if the cell has been visited multiple times in last 12 hours
        double decayScore = neighbor.getTempErr()/12;
        for (int i = currentPath.size()-1; i >= 0; --i) {
            double decayedScore = neighbor.getTempErr();
            OceanCell pathCell = currentPath.get(i);
            if (pathCell.equals(neighbor)) {
                tempScore -= decayedScore;
            }
            if (decayedScore > 0) {
                decayedScore -= decayScore;
            }
        }
        return tempScore;
    }
    
    /* 
     * A* Search path planning algorithm. It takes in
     * 7 inputs: the x, y, and z coordinates of the start and end
     * locations, and the maximum time the mission can take.
     * The path planner looks at all the neighbors 
     * and finds the most feasible path given the 
     * data from ROMS on the currents. The objective of the path planner
     * is to maximize the change in temperature from waypoint to waypoint.
     * 
     */
    public static OceanPath AStar(OceanCell start, OceanGrid grid, double missionLength) {
        
        // Print out the net distance to get an idea of how far the end is from
        // the start location
        //double netDistance = AUV.distance(xStart, yStart, xEnd, yEnd, 'K');
        //netDistance *= 1000;
        //System.out.println("The net distance is " + netDistance + " meters");
        boolean record = Planner.mathematica;
        
        double timeInterval = Planner.timeInterval;
        double startTime = Planner.hourStartIndex*timeInterval;
        double maxMissionTime = startTime + missionLength;
        
        // priority queue to hold the OceanCells
        PriorityQueue<OceanPath> Q = new PriorityQueue<>(10,
                new Comparator<OceanPath>() {
                    @Override
                    public int compare(OceanPath c1, OceanPath c2) {
                        double diff = c1.fScore - c2.fScore;
                        if (diff < 0) {
                            // make 1 for maximum, -1 for minimum
                            return 1;
                        }
                        else if (diff > 0) {
                            return -1;
                        }
                        else {
                            return 0;
                        }
                    }
        });
        
        OceanPath startPath = new OceanPath();
        startPath.add(start);
        startPath.latestTime = startTime;
        Q.add(startPath);

        //initialize what the maximum delta is to -1
        //double maxDelta = -1;
        // counter for debugging
        //int count = 0;

        while (!Q.isEmpty()) {
            //OceanCell currentCell = (OceanCell) Q.dequeue();
            OceanPath currentPath = (OceanPath) Q.poll();
            OceanCell currentCell = currentPath.get(currentPath.size()-1);
            //System.out.println(currentPath);
            
            if (currentPath.size() > 1) {
                int time = (int)Math.floor(currentPath.latestTime / timeInterval);
                if (time > Planner.hourEndIndex) {
                    System.out.println("Need more time data! Path Planning"
                            + " will just use data from latest timestep");
                    time = Planner.hourEndIndex;
                }
                // if we moved onto the next timestep 
                if (time > currentPath.get(currentPath.size()-2).getTime()) {
                    OceanCell sameCell = grid.getCell(time, 
                            currentCell.getDepth(), currentCell.getLat(),
                            currentCell.getLon());
                    sameCell.copyData(currentCell);
                    currentCell = sameCell;
                }  
            }
            
            if (record) {
                Planner.recordInstance(currentPath.path, false, grid);
            }

            int t = currentCell.getTime();
            int z = currentCell.getDepth();
            int x = currentCell.getLat();
            int y = currentCell.getLon();         
            
            boolean moreNeighbors = false;
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
                    double neighborScore = currentPath.gScore 
                            + addObjective(currentPath, neighbor);

                    // impossible to reach cell, skip to next neighbor
                    if (timeTaken < 0) {
                        continue;
                    }                  
                    
                    if ((currentPath.latestTime + timeTaken) <= maxMissionTime) {
                        /*
                        ArrayList<OceanCell> newPath = new ArrayList<>();
                        for (int i = 0; i < currentPath.size(); ++i) {
                            OceanCell copy = new OceanCell(currentPath.get(i));
                            newPath.add(copy);
                        }
                        */
                        moreNeighbors = true;
                        OceanPath newPath = new OceanPath(currentPath);
                        newPath.add(neighbor);
                        newPath.latestTime += timeTaken;
                        newPath.gScore = neighborScore;
                        newPath.fScore = newPath.gScore + objectiveEstimate(newPath, grid);
                        Q.add(newPath);
                    }      
                 }
            }
            // Reached the destination within a minute of max time alotted
            //if ((maxMissionTime-2000) <= currentPath.latestTime
            //        && currentPath.latestTime <= maxMissionTime) {
            if (!moreNeighbors) {
                if(record) {
                    Planner.recordInstance(currentPath.path, true, grid);
                    Planner.recordHistory(new File(Planner.historyFile));
                }
                currentPath.recordData(grid);
                return currentPath;
            }
        }
    System.out.println("There are no possible paths to this destination");
    return null;
    }
    
}
