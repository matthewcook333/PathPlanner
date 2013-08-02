/*
 * 
 */
package PathPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;

/**
 *
 * @author Matt Cook
 * 
 * Class: AStarPlanner
 * 
 * Description: This class is holds the methods used for the A* Path Planning
 *  Algorithm. Note that this class relies on variables in the Planner class
 *  as well as using the OceanCell, OceanPath, and OceanGrid classes.
 */
public class AStarPlanner {
    
    /*
     * Method: objectiveEstimate
     * 
     * Input: OceanPath that is the current path, OceanGrid, and a double that
     *  is the max mission length.
     * 
     * Output: a double for the estimate of objective remaining on the path
     * 
     * Details: The objective function is currently maximizing the uncertainty
     *  in temperature information.
     *  This method uses the remaining time left for the mission along
     *  with the current number of cells travels to estimate how many cells 
     *  the path can still go. 
     *  Currently the estimation for how much temperature uncertainty the AUV
     *  can gather is based on the average temperature uncertainty in the grid
     *  multiplied by some weighting. A weighting of 0 would mean to not
     *  consider the heuristic at all, and a higher weighting would make the
     *  heuristic more important in comparing paths, thus making the path planner
     *  explore more paths (become close to a breadth first search)
     */
    public static double objectiveEstimate(OceanPath currentPath, OceanGrid grid, double maxMissionTime) {
        OceanCell currentCell = currentPath.get(currentPath.size()-1);
        double avgTempErr = grid.averageTempErr(currentCell.getTime());
        double timeLeft = maxMissionTime - currentPath.timeElapsed;
                // no longer need (Planner.hourStartIndex*Planner.timeInterval);
        double predCells = (currentPath.size()/currentPath.timeElapsed)*timeLeft;
        double heuristicRate = (avgTempErr * Planner.weighting);
        double predScore = heuristicRate * predCells;
        //System.out.println("average: " + avgTempErr);
        return predScore;
    }
    
        /*
     * Method: objectiveEstimate2
     * 
     * Input: OceanPath that is the current path, OceanGrid, and a double 
     *  that is the maximum mission length.
     * 
     * Output: a double for the estimate of objective remaining on the path
     * 
     * Details: The objective function is maximizing the uncertainty
     *  in temperature information.
     *  This method uses the remaining time left for the mission along
     *  with the current number of cells travels to estimate how many cells 
     *  the path can still go. 
     *  This objective function looks at the neighbors around the most recent 
     *  cell in the path to determine how much potential uncertainty gain there
     *  is on the path.
     */
    public static double objectiveEstimate2(OceanPath currentPath, OceanGrid grid, double maxMissionTime) {
        //double timeLength = 12;
        double heuristicRate = 0;
        OceanCell currentCell = currentPath.get(currentPath.size()-1);
        
        ArrayList<OceanCell> neighbors = Planner.findNeighbors(currentPath, grid, grid.NLAT/4);
        for (int i = 0; i < neighbors.size(); ++i) {
            OceanCell neighbor = neighbors.get(i);
            double neighborReward = neighbor.getTempErr();
            if (currentPath.contains(neighbor)) {
                /*
                double timeDiff = neighbor.getTime() - currentCell.getTime(); 
                neighborReward = neighborReward * 
                        (1 - Math.pow(Planner.discount, timeDiff));
                */ 
                heuristicRate += neighborReward;
            }
            else {
                heuristicRate += neighborReward;
            }
        } 
        heuristicRate = heuristicRate/(neighbors.size());
      
        double errRateChange = 0;
        int i = 1;
        while (i < 6 && i < currentPath.size()) {
            OceanCell cell = currentPath.get(currentPath.size()-i);
            OceanCell parentCell = currentPath.get(currentPath.size()-i-1);
            errRateChange += cell.getTempErr() / parentCell.getTempErr();
            ++i;
        }
        errRateChange = errRateChange / i;
               
        double timeLeft = maxMissionTime - currentPath.timeElapsed;
        double predCells = (Planner.propulsion / 3000) * timeLeft;
       
        double predImmCells = grid.NLAT/2;
        
        if (predCells < predImmCells) {
            predImmCells = predCells;
            predCells = 0;
        }
        else {
            predCells -= predImmCells;
        }
        //double predImmCells = predCells / 5;

        double avgTempErr = grid.averageTempErr(currentCell.getTime());
        double predScore = avgTempErr * predCells;
        
        heuristicRate *= errRateChange;
        double predImmScore = heuristicRate * predImmCells;
        
        predScore += predImmScore;
        predScore *= Planner.weighting; 
        return predScore;
    }
    
    
    /*
     * Method: addObjective
     * 
     * Input: OceanPath for the current path, OceanCell for the neighbor being
     *  added.
     * 
     * Output: a double representing the additional objective fromm adding
     *  this neighbor
     * 
     * Details: The objective currently being considered is temperature
     *  uncertainty. In order to discourage visiting the same cells spatially
     *  within a short timespan, there is a linear decay for the objective for 
     *  revisiting the same cell in space within the last 12 hours.
     *  i.e. if the path makes a u-turn, it will only get 1/12 or 2/12 of the 
     *  reward for going there, depending on how long it takes. 
     *  This means that after 12 hours it can get the full reward again
     *  for revisiting the same cell spatially.
     */
    public static double addObjective(OceanPath currentPath, OceanCell neighbor) {
        double tempScore = neighbor.getTempErr(); 
        //double timeLength = 12;
       // double timeLength = Planner.missionLength/Planner.timeInterval;
        //double decayScore = neighbor.getTempErr() / timeLength;
        for (int i = currentPath.size()-1; i >= 0; --i) {
            OceanCell pathCell = currentPath.get(i);
            int timeDiff = neighbor.getTime() - pathCell.getTime();
            if (pathCell.equals(neighbor)) {// && timeDiff < timeLength) {
                //double timePenalty = timeLength - timeDiff;
                //tempScore -=decayScore * timePenalty; 
                tempScore = tempScore * 
                        (1 - Math.pow(Planner.discount, timeDiff));
                //System.out.println("added score: " + tempScore);
                return tempScore;
            }
        }
        //System.out.println("added score: " + tempScore);
        return tempScore;
    }
    
    /* 
     * Method: AStar
     * 
     * Input: OceanCell that is the start, OceanGrid for searching on, and
     *  a double which is the max mission length.
     * 
     * Output: OceanPath that holds the cells of the final path.
     * 
     * Details: This A* path planning algorithm does not take in a destination
     *  as it just finds the optimal path to gather the maximum objective 
     *  within the grid given a start location.
     * 
     */
    public static OceanPath AStar(OceanCell start, OceanGrid grid, double missionLength) {
        
        double timeInterval = Planner.timeInterval;
        double startTime = Planner.hourStartIndex*timeInterval;
        double maxMissionTime = startTime + missionLength;
        //Planner.missionLength = maxMissionTime;
        
        // priority queue to hold the OceanPaths
        // The top of the queue is the OceanPath with the highest f score, 
        // which is the sum of the objective gathered so far with the heuristic
        // for predicted objective remaining
        PriorityQueue<OceanPath> Q = new PriorityQueue<OceanPath>(10,
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
        
        // create the start OceanPath to enqueue
        OceanPath startPath = new OceanPath();
        startPath.add(start);
        startPath.timeElapsed = startTime;
        Q.add(startPath);
        
        int count = 0;      

        // while we have unexplored paths, continue searching
        while (!Q.isEmpty()) {
            count++;
            OceanPath currentPath = (OceanPath) Q.poll();
            OceanCell currentCell = currentPath.get(currentPath.size()-1);

            System.out.println(currentPath.gScore + ", " + (currentPath.fScore-currentPath.gScore));
            System.out.println(currentCell);
            
            /*
             if (count > 20000) {
                 System.out.println("too high!");
                 return null;
             }
             * */
            // Record path if tracing paths in mathematica
            if (Planner.mathematica) {
                Planner.recordInstance(currentPath.path, false, grid);
            }
            
            int t = currentCell.getTime();
            int z = currentCell.getDepth();
            int x = currentCell.getLat();
            int y = currentCell.getLon();         
            // initialize whether we have more feasible neighbors to travel to
            // on this path to false
            boolean moreNeighbors = false;
            for (int dir = 0; dir < Planner.numDirections; dir++) {
                // find grid indices of the neighbors
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
                    
                    // if we can travel to this neighbor within the time limit,
                    // create a new path including this neighbor and enqueue
                    // it in the priority queue
                    if ((currentPath.timeElapsed + timeTaken) <= maxMissionTime) {
                        // check if the neighbor is in a different timestep
                        int time = (int)Math.floor(
                                (currentPath.timeElapsed + timeTaken)
                                / timeInterval);
                        // if there isn't enough time data, just use the latest one
                        if (time > Planner.hourEndIndex) {
                            System.out.println("Need more time data! Path Planning"
                                    + " will just use data from latest timestep");
                            time = Planner.hourEndIndex;
                        }
                        // if we moved onto the next timestep, we get the cell data
                        // associated with that space and time              
                        if (time > currentPath.get(currentPath.size()-1).getTime()) {
                            neighbor = grid.getCell(time, 
                                    neighbor.getDepth(), neighbor.getLat(),
                                    neighbor.getLon());
                        }  
                        moreNeighbors = true;
                        OceanPath newPath = new OceanPath(currentPath);
                        newPath.add(neighbor);
                        newPath.timeElapsed += timeTaken;
                        newPath.distance += AUV.distance(
                                currentCell.getLatValue(), currentCell.getLonValue(),
                                neighbor.getLatValue(), neighbor.getLonValue(), 'K');
                        newPath.gScore = currentPath.gScore 
                            + addObjective(currentPath, neighbor);
                        newPath.fScore = newPath.gScore + objectiveEstimate2(newPath, grid, maxMissionTime);
                        Q.add(newPath);
                    }      
                }
            }
            // If we have run out of neighbors to travel to, meaning we have
            // found the optimal path, return the path
            if (!moreNeighbors) {
                // if we are recording, record the final path
                if(Planner.mathematica) {
                    Planner.recordInstance(currentPath.path, true, grid);
                    Planner.recordHistory(new File(Planner.historyFile));
                }
                // Fill the correct information into the OceanCells in the path
                currentPath.recordData(grid, maxMissionTime);
                return currentPath;
            }
        }   
        // return null if there are no possible paths
        System.out.println("There are no possible paths to this destination");
        return null;
    }
}
