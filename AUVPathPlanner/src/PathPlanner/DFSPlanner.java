/*
 *
 */
package PathPlanner;

import java.io.File;
import java.util.ArrayList;
import java.util.Stack;

/**
 *
 * @author Matt Cook
 * 
 * class: DFSPlanner
 * 
 * Description: Depth First Search path planner. This path planning algorithm
 *  does an exhaustive search of all possible paths given a start location,
 *  and outputs the best possible path given an objective function. 
 * 
 * TODO: Currently this method only works for shorter paths as the computation
 *  time gets far too large as it grows exponentially, particularly if the
 *  possible directions to travel include diagonals or different depths. 
 *  One way to avoid this is to path plan on a coarser resolution, and 
 *  recursively path plan on finer resolutions along the path.
 */
public class DFSPlanner {
      
    /*
     * Method evalObj
     * 
     * Input: OceanPath that is the current path, and OceanCell that 
     *  is the current cell.
     * 
     * Output: a double that is the objective score from adding the current
     *  cell.
     * 
     * Details: This objective evaluation function looks only at whether it has
     *  found a higher maximum temperature or lower mininum temperature along
     *  a path. Thus, it is looking to maximize the temperature extremes found
     *  on a path.
     */ 
    public static double evalObj(OceanPath currentPath, OceanCell currentCell) {
        double obj = 0;
        if (currentCell.getTemp() > currentPath.maxTemp) {
            obj = Math.abs(currentCell.getTemp()-currentPath.maxTemp) ;
            currentPath.maxTemp = currentCell.getTemp();
        }
        if (currentCell.getTemp() < currentPath.minTemp) {
            obj = Math.abs(currentCell.getTemp()-currentPath.minTemp);
            currentPath.minTemp = currentCell.getTemp();
        }
      return obj;
    }
    
    
    /*
     * Method: evalObj2
     * 
     * Input: OceanCell that is the parent cell, and OceanCell that is the 
     *  current cell.
     * 
     * Output: a double that is the absolute difference between the temperature
     *  of the two cells.
     * 
     * Details: This objective evaluating method looks to maximize the 
     *  temperature differences between cells on a path. Thus, it is finding
     *  a path that has parts of the ocean with fast changing temperatures, 
     *  like a cold front.
     */
    public static double evalObj2(OceanCell parent, OceanCell currentCell) {
        return Math.abs(parent.getTemp() - currentCell.getTemp());
    }
    
    /* 
     * 
     * Method: DFS
     * 
     * Input: OceanCell for start, OceanCell for dest (null for no dest), 
     *  OceanGrid to do the search on, a double for the maximum mission length
     * 
     * Output: OceanPath that is the best path found. 
     * 
     * Details: Depth First Search path planning algorithm thatdoes an 
     *  exhaustive search of  the possible paths, and returns the path that 
     *  best satisfies the objective chosen.
     * 
     */
    public static OceanPath DFS(OceanCell start, OceanCell dest, OceanGrid grid, double missionLength) {
        
        double timeInterval = Planner.timeInterval;
        double startTime = Planner.hourStartIndex*timeInterval;
        double maxMissionTime = startTime + missionLength;
        
        // initialize lists for visited cells and best path found
        ArrayList<OceanCell> visited = new ArrayList<>();
        OceanPath bestPath = new OceanPath();
        // Stack for DFS
        Stack<OceanCell> S = new Stack<>();
        
        S.push(start);
        // We also need to keep track of the depth to keep track of visited
        // cells on the current path we are checking
        start.treeDepth.push(0);
        start.timeArrived = startTime;
        
        //initialize what the maximum score is to -1
        double maxScore = -1;
        int counter = 0;

        while (!S.isEmpty()) {
            counter++;     
            boolean validCell = true;
            OceanCell currentCell = (OceanCell)S.pop();
            int currentDepth = currentCell.treeDepth.pop();
            
            // clear the visited cells from previous paths we've searched
            int visitedSize = visited.size();
            for (int i = visitedSize; i > currentDepth; --i) {
                visited.remove(i-1);
            }
            
            
            if (visited.size() > 0) {
                OceanCell parent = visited.get(visited.size()-1);
                double timeTaken = AUV.travelTime(parent, currentCell);
                currentCell.timeArrived = parent.timeArrived + timeTaken;

                // impossible to reach this neighbor, skip to next neighbor
                if (timeTaken < 0 || currentCell.timeArrived > maxMissionTime) {
                    // we could not reach this cell in time allotted
                    validCell = false;
                }
                currentCell.gScore = parent.gScore + evalObj2(parent, currentCell); 
                              
                int time = (int)Math.floor(currentCell.timeArrived / timeInterval);
                if (time > Planner.hourEndIndex) {
                    System.out.println("Need more time data! Path Planning"
                            + " will just use data from latest timestep");
                    time = Planner.hourEndIndex;
                }
                // if we moved onto the next timestep 
                if (time > parent.getTime()) {
                    OceanCell sameCell = grid.getCell(time, 
                            currentCell.getDepth(), currentCell.getLat(),
                            currentCell.getLon());
                    sameCell.copyData(currentCell);
                    currentCell = sameCell;
                }        
            }
            // add current cell to the list of visited cells
            visited.add(currentCell);

            if(Planner.mathematica){
		Planner.recordInstance(visited, false, grid);
	    }
            
            //if we have found a better path, replace best path with new path
            if (currentCell.gScore > maxScore 
                    && (currentCell.equals(dest) || dest == null)
                    && validCell){ 
                maxScore = currentCell.gScore;
                bestPath.clear();
                int size = visited.size()-1;
                for (int i = 0; i < size; --i) {
                    OceanCell copy = new OceanCell(visited.get(i));
                    bestPath.add(copy); 
                }          
            }

            // get indices for the current cell in order to find the 
            //neighboring cells
            int t = currentCell.getTime();
            int x = currentCell.getLat();
            int y = currentCell.getLon();
            int z = currentCell.getDepth();
     
            // Look at all the neighbors
            for (int dir = 0; dir < Planner.numDirections; dir++) {
                // if we could not reach the current cell in time allotted,
                // no need to check neighbors.
                if (!validCell) {
                    break;
                }
                // Get the indices for the neighbor with the array of possible
                // directions
                int dirx = Planner.directions[dir * 2];
                int diry = Planner.directions[dir * 2 + 1];
                int newx = x + dirx;
                int newy = y + diry;
                
                // If heading toward a feasible location
                if ((newy >= 0) && (newy < grid.getLonLength())
                        && (newx >= 0) && (newx < grid.getLatLength())) {
                    
                    OceanCell neighbor = grid.getCell(t,z,newx,newy);

                    // check if this neighbor has already been visited on 
                    // the current path
                    boolean visitedFlag = false;
                    for (int i = 0; i < visited.size(); ++i) {
                        if (neighbor.equals(visited.get(i))) {
                            visitedFlag = true;
                        }
                    }
                    // if not, update the data members of the cell and push
                    // it onto the stack. Also, we push the length of the path
                    // up to this cell.
                    if (!visitedFlag) {
                        neighbor.treeDepth.push(currentDepth+1);
                        S.push(neighbor);
                    }
                }
            }
        }
        
        System.out.println("checked " + counter + " cells");
        if  (bestPath.isEmpty()) {
            System.out.println("Cannot reach the destination in time limit!");
            System.exit(-1);
        }
        if(Planner.mathematica){
                Planner.recordInstance(visited, false, grid);
                Planner.recordInstance(bestPath.path, true, grid);
        }
        System.out.println("maximum score is " + maxScore);
        Planner.recordHistory(new File(Planner.historyFile));
        return bestPath;
    }
    
    
    /*
     * Method: DFSPathPlanner
     * 
     * Input: OceanCell start, OceanCell dest (can be null if no dest),
     *  OceanGrid to search on, double for the max distance, int for number 
     *  of iterations we can have of a coarser resolution.
     * 
     * TODO: Method not complete yet! This is to combine the reducing resolution
     *  with the DFS. Also, need to change max distance to max mission time.
     */
    public static ArrayList<OceanCell> DFSPathPlanner(OceanCell start, OceanCell dest, OceanGrid grid, double maxDistance, int iterator) {
        OceanGrid origGrid = grid;
        int origIter = iterator;
        while (iterator != 0) {
            grid = grid.ReduceResolution();
            --iterator;
        }
        OceanPath bestPath = DFS(start, dest, grid, maxDistance);
        OceanPath bestHighResPath = null;
        for (int i = 0; i < bestPath.size()-1; ++i) {
            OceanCell pt = bestPath.get(i);
            OceanCell nextPt = bestPath.get(i+1);
            maxDistance = AUV.distance(pt.getLatValue(), pt.getLonValue(),
                    nextPt.getLatValue(), nextPt.getLonValue(), 'K');
            maxDistance *= 1000;
            bestHighResPath.path.addAll(DFSPathPlanner(pt, nextPt, origGrid, maxDistance, origIter-1));
            bestHighResPath.path.remove(bestHighResPath.size()-1);
            }
        return bestHighResPath.path;
    }
    
}
