/*
 *
 */
package PathPlanner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Stack;

/**
 *
 * @author atthewco
 * 
 * class: Planner
 * 
 * Description: This class is where the primary path planning takes place.
 * At the top of the class is all the global variables that can be changed
 * to fit the desired parameters, and should be the only parameters that need to
 * be changed for the path planner to work properly. 
 */
public class DFSPlanner {
      
//    public static double evalObj(OceanCell currentCell, OceanCell neighbor) {
//        return Math.abs(currentCell.getTemp() - neighbor.getTemp());
//    }
     
      public static double evalObj(OceanCell parent, OceanCell currentCell) {
          double obj = 0;
          if (currentCell.getTemp() > parent.maxTemp) {
              obj = Math.abs(currentCell.getTemp()-parent.maxTemp) ;
              currentCell.maxTemp = currentCell.getTemp();
          }
          else {
              currentCell.maxTemp = parent.maxTemp;
          }
          if (currentCell.getTemp() < parent.minTemp) {
              obj = Math.abs(currentCell.getTemp()-parent.minTemp);
              currentCell.minTemp = currentCell.getTemp();
          }
          else {
              currentCell.minTemp = parent.minTemp;
          }
        return obj;
      }
     
     
    
    /* 
     * Depth First Search path planning algorithm. It takes in
     * 7 inputs: the x, y, and z coordinates of the start and end
     * locations, and the maximum time the mission can take.
     * The path planner looks at all the neighbors 
     * and finds the most feasible path given the 
     * data from ROMS on the currents. The objective of the path planner
     * is to maximize the change in temperature from waypoint to waypoint.
     * 
     */
    public static OceanPath DFS(OceanCell start, OceanCell dest, OceanGrid grid, double missionLength) {
        
        boolean record = Planner.mathematica;
        
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
                // distance calculation, not time
                //double timeTaken = AUV.distance(parent.getLatValue(), parent.getLonValue(),
                        //currentCell.getLatValue(), currentCell.getLonValue(), 'K');
                //timeTaken *= 1000;
                //double timeTaken = 1;
                currentCell.timeArrived = parent.timeArrived + timeTaken;

                // impossible to reach this neighbor, skip to next neighbor
                if (timeTaken < 0 || currentCell.timeArrived > maxMissionTime) {
                    // we could not reach this cell in time allotted
                    validCell = false;
                }
                currentCell.gScore = parent.gScore + evalObj(parent, currentCell); 
                              
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

            if(record){
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
            
            //System.out.println("Current Cell: " + x + ", " + y + ", " + z);
            
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
        if(record){
                Planner.recordInstance(visited, false, grid);
                Planner.recordInstance(bestPath.path, true, grid);
        }
        System.out.println("maximum score is " + maxScore);
        Planner.recordHistory(new File(Planner.historyFile));
        return bestPath;
    }
    
   
        
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
