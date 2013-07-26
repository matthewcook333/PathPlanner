/*
 *
 */
package PathPlanner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

/**
 *
 * @author Matt Cook
 * 
 * class: Planner
 * 
 * Description: This class is where the primary path planning takes place.
 * At the top of the class is all the global variables that can be changed
 * to fit the desired parameters, and should be the only parameters that need to
 * be changed for the path planner to work properly. 
 */
public class Planner {

    // fileName is the name of the netCDF forecast data file
    // This can be a local file or an http address to pull the file 
    // from a THREDDS server 
    static String fileName = "ca_subCA_fcst_2013070203.nc";
    //static String fileName = "ca_subCA_das_2013061209.nc";
    //static String fileName = "http://west.rssoffice.com:8080/thredds/dodsC/pacific/CA3km-forecast/CA/ca_subCA_fcst_2013070203.nc";
    //static String fileName = "http://west.rssoffice.com:8080/thredds/dodsC/pacific/CA3km-forecast/CA/ca_subCA_errfcst_2013062403.nc";
    
    // errFileName is the netCDF forecast data from ensemble mode
    // Same ways to retrive file as for the forecast file
    static String errFileName = "ca_subCA_errfcst_2013070103.nc";
    // name of output text VectorMap file and KML file
    static String outputFile = "test.txt";
    static String KMLFile = "test1.kml";
    // name of test output file for testing
    static String testFile = "testOutput1.txt";
    
    // name of the search algorithm to use
    // Choices: AStar, DFS, Random, TEST
    static String SearchAlg = "TEST2";
    // Used for DFS, set to true to find a destination cell
    static boolean findDest = false;
    // Switch to false when testing without specified start location
    static boolean useStart = false;
    
    //start and end coordinates
    static double latStart = 33.2;
    static double lonStart = 360-118.4;  
    // Used for DFS
    static double latDest = 32.16;
    static double lonDest = 360-118.6;
    
    // weighting for heuristic, 0 is for greedy algo and higher is 
    // closer to breadth first search
    static double weighting = 1;
    
    // maximum length of the mission in seconds.
    static double missionLength = 259200;
    // propulsion of the AUV in m/s
    static double propulsion = 1.3;
    
    // Long Beach LAT LON Boundaries
    //  33deg 18' 39.66" N
    //final static double LOWLAT = 33.311017;
    final static double LOWLAT = 33.15;
    // FOR 0,0 final static double LOWLAT = 32.03;
    //final static double LOWLAT = 31.6;
    
    
    // 33 deg 44' 18.58" N
    final static double HIGHLAT = 33.738494;
    //final static double HIGHLAT = 34;
    // 81x81 final static double HIGHLAT = 34.43;
    //final static double HIGHLAT = 32.5;
    
    
    // 118 deg 27' 34.82" W
    //final static double LOWLON = 360-118.459672;
    final static double LOWLON = 360 - 118.6;
    // FOR 0,0 final static double LOWLON = 360-120;
    //final static double LOWLON = 360-120.5;
    
    
    // 118 deg 0' 57.54" W
    final static double HIGHLON = 360-118.015983;
    // 81x81
    //final static double HIGHLON = 360-117.6;
    //final static double HIGHLON = 360-119.5;
    
    
    // number of depth levels to retrieve
    final static int depthDim = 7;
    // time indices. NOTE: Index 0 is 1 hour after 3AM on the day of file
    final static int hourStartIndex = 0;
    final static int hourEndIndex = 72;
    // For converting to seconds for the path planning
    final static int timeInterval = 3600;
    
    // degree interval between each lat and lon
    // NOTE: Should only need to change if using different netCDF file
    final static double interval = 0.03;
    // precision that determines the error bound for lat and lon values.
    final static double prec = interval/2;
    
    // Array of all the possible locations at the same depth
    // If we just want 4 possible directions
    //final static int[] directions = { -1, 0, 0, -1, 0, 1, 1, 0,};
    //final static int numDirections = 4;
    
    // For 8 possible directions, includes diagonals
    final static int[] directions = {-1, -1, -1, 0, -1, 1, 0, -1,
            0, 1, 1, -1, 1, 0, 1, 1};
    final static int numDirections = 8;
    
    // for including depth, so 26 possible directions
    // final static int[] directions = {-1,-1,-1, 0,-1,-1, -1,-1,0, -1,0,-1, 
    //    -1,0,0, 0,-1,0, 0,0,-1, 1,1,1, 0,1,1, 1,1,0, 1,0,1, 1,0,0, 0,1,0, 0,0,1,
    //     1,-1,-1, -1,-1,1, -1,1,-1, -1,1,1, 1,-1,1, 1,1,-1, -1,1,1, 1,1,-1,
    //     1,-1,1, 1,-1,-1, -1,1,-1, -1,-1,1};
    // final static int[] numDirections = 26;
    
    // set true if text output for mathematica is needed
    // history is used to store mathematica string
    // NOTE: This makes the program run for much slower to record all paths
    static boolean mathematica = false;
    static String history = "";
    static String historyFile = "DFS_POC_COOK2.txt";
    
    // variables stored to be read across classes
    static int[] startIndex;
    static int[] destIndex;
    
    static double avgTempErr;
    static OceanGrid griddy;
    // name of file to show the grid coordinates
    static String gridFileName = "grid.kml";
    final static double[] boundaries = {33.459999084472656, 241.41000366210938, 33.459999084472656,
        241.44000244140625,33.459999084472656, 241.47000122070312,33.459999084472656, 241.5,33.43000030517578, 241.41000366210938,33.43000030517578, 241.44000244140625,33.43000030517578, 241.47000122070312,
33.43000030517578, 241.5,33.43000030517578, 241.52999877929688,33.43000030517578, 241.55999755859375,33.43000030517578, 241.58999633789062,
33.39999771118164, 241.5,33.39999771118164, 241.52999877929688,33.39999771118164, 241.55999755859375,33.39999771118164, 241.58999633789062,
33.39999771118164, 241.6199951171875,33.39999771118164, 241.64999389648438,33.369998931884766, 241.5,33.369998931884766, 241.52999877929688,
33.369998931884766, 241.55999755859375,33.369998931884766, 241.58999633789062,33.369998931884766, 241.6199951171875,33.369998931884766, 241.64999389648438,
33.369998931884766, 241.67999267578125,33.34000015258789, 241.5,33.34000015258789, 241.52999877929688,33.34000015258789, 241.55999755859375,
33.34000015258789, 241.58999633789062,33.34000015258789, 241.6199951171875,33.34000015258789, 241.64999389648438,33.34000015258789, 241.67999267578125,
33.34000015258789, 241.7100067138672,33.30999755859375, 241.52999877929688,33.30999755859375, 241.55999755859375,33.30999755859375, 241.58999633789062,
33.30999755859375, 241.6199951171875,33.30999755859375, 241.64999389648438,33.30999755859375, 241.67999267578125,33.30999755859375, 241.7100067138672,
33.279998779296875, 241.64999389648438,33.279998779296875, 241.67999267578125};
    
        
    /* 
     * Method: PathPlanner
     * 
     * Input: None
     * 
     * Output: ArrayList of OceanCells
     * 
     * Details: This is the primary method that reads the netCDF file 
     * and constructs the grid, then chooses the path planning
     *  algorithm and returns the path.
     */
    public static ArrayList<OceanCell> PathPlanner() {
        System.out.println("Starting path planning with " + SearchAlg + "...");

        // read the .nc file in so that the arrays hold
        // information
        OceanGrid.readFile(fileName, errFileName);

        // Make a 3-D array which holds all the data
        OceanGrid grid = new OceanGrid();
        grid = OceanGrid.averageDepths(grid);
        OceanGrid.validateCells(grid);
        writeMissionPlan.writeKMLGrid(gridFileName, grid);
        
        
        //for (int t = 0; t < grid.NTIME; ++t) {
            //for (int i = 0; i < grid.NLAT; ++i) {
              //  for (int j = 0; j < grid.NLON; ++j) {
                    //for (int d = 0; d < grid.NDEPTH; ++d) {
                        //if (grid.getCell(t, d, i, j).getTemp() < 0) {
                //            System.out.println(grid.getCell(0, 0, i, j));
                        //}
                        
                    //}
             //   }
           // } 
        //}
        
        
        // find start cell from the grid
        OceanCell start = null;
        if (useStart) {
            start = grid.getCell(hourStartIndex, 0, startIndex[1], startIndex[0]);
            // if we could not find the start coordinates
            if (start == null && useStart) {
                System.out.println("The chosen start coordinates are not within "
                        + "the boundaries!");
                System.exit(0);
             }
        }
        
        // find the dest cell in grid if necessary
        OceanCell dest = null;
        if (Planner.findDest) {
            dest = grid.getCell(hourEndIndex,0, Planner.destIndex[1], Planner.destIndex[0]);
        }
        
        /*
        // just for tracing average temp err for now
        avgTempErr = grid.averageTempErr();
        System.out.println("AVERAGE TEMP ERR IS: " + avgTempErr);
        */
        
        OceanPath path = null;
        switch (SearchAlg) {
            case "DFS":
                path = DFSPlanner.DFS(start, dest, grid, missionLength);
                break;
            case "AStar":
                path = AStarPlanner.AStar(start, grid, missionLength);
                break;
            case "Random":
                path = RandomPlanner.Random(start, grid, missionLength);
                break;
            case "TEST":
                path = PathTests.testPaths(start, grid, missionLength);
                break;
            case "TEST2":
                path = PathTests.testPaths2(start, grid, missionLength);
                break;
            default:
                System.out.println("Pick a search algorithm that exists!");
                break;
        }
        return path.path;
    }
    
    
    /*
     * Method: findStart
     * 
     * Input: OceanGrid to find a start cell
     * 
     * Output: OceanCell that is the optimal start cell
     * 
     * Details: TODO. Finds the optimal start cell for path planning.
     */
    
    public static OceanCell findStart(OceanGrid grid) {
        OceanCell bestCell = grid.getCell(hourStartIndex, 0, 0, 0);;
        for (int i = 0; i < grid.NLAT; ++i) {
            for (int j = 0 ; j < grid.NLON; ++j) {
                OceanCell cell = grid.getCell(hourStartIndex, 0, i, j);
                if (cell.getTempErr() > bestCell.getTempErr()) {
                    bestCell = cell;
                }
            }
        }
        return bestCell;
    }
    
    
    /*
     * Method: findNeighbors
     * 
     * Input: OceanCell, OceanGrid, and an int for the range of how far to look
     *  for nearby neighbors
     * 
     * Output: an ArrayList of OceanCells which are the neighbors of the 
     *  inputted OceanCell
     * 
     * Details: Finds all the neighboring cells within the range provided.
     *  Used for the heuristic for A* path planning.
     */
    public static ArrayList<OceanCell> findNeighbors(OceanPath currentPath, OceanGrid grid, int range) {
        OceanCell currentCell = currentPath.get(currentPath.size()-1);
        int t = currentCell.getTime();
        int z = currentCell.getDepth();
        int x = currentCell.getLat();
        int y = currentCell.getLon();         
            
        ArrayList<OceanCell> neighbors = new ArrayList<>();
        for (int dirx = -(range); dirx <= range; ++dirx) {
            for (int diry = -(range); diry <= range; ++diry) {
                int newx = x + dirx;
                int newy = y + diry;
                // If heading toward a feasible location
                if ((newy >= 0) && (newy < grid.getLonLength())
                        && (newx >= 0) && (newx < grid.getLatLength())) {
                    OceanCell neighbor = grid.getCell(t,z,newx,newy);
                    double timeTaken = AUV.travelTime(currentCell, neighbor);
                    // skip neighbors we cannot reach
                    if (timeTaken < 0) {
                        continue;
                    }
                    // check if the neighbor is in a different timestep
                    int time = (int)Math.floor(
                            (currentPath.timeElapsed + timeTaken)
                            / timeInterval);
                    // if there isn't enough time data, just use the latest one
                    if (time > Planner.hourEndIndex) {
                        // just going to use latest time data
                        time = Planner.hourEndIndex;
                    }
                    // if we moved onto the next timestep, we get the cell data
                    // associated with that space and time              
                    if (time > currentCell.getTime()) {
                        neighbor = grid.getCell(time, 
                                neighbor.getDepth(), neighbor.getLat(),
                                neighbor.getLon());
                    } 
                    // since location is within boundaries, add this neighbor
                    neighbors.add(neighbor);
                }  
            }
        }
        return neighbors;    
    }
    
    
    /* Method: recordInstance
     * 
     * Input: whether this instance is the last to be recorded
     * 
     * Output: records the grid at a particular instance
     * 
     * Details: Written by Sherman Lam. Edited a bit to work with my code.
     *  This method is used for debugging and tracing purposes in 
     *  Mathematica.
     */
    public static void recordInstance(ArrayList<OceanCell> visitedSet, boolean last, OceanGrid grid){
         int Ntime = grid.cellGrid.length;
         int Ndepth = grid.cellGrid[0].length;
         int Nrows = grid.cellGrid[0][0].length;
         int Ncols = grid.cellGrid[0][0][0].length;
         int pairsWritten = 0;

         Planner.history += "{";
         for(int t=0; t<Ntime; t++) {
             for(int d=0; d<Ndepth; d++) {
                 for(int row=0; row<Nrows; row++){
                         for(int col=0; col<Ncols; col++){
                             for (int i = 0; i<visitedSet.size(); i++) {
                                 if(visitedSet.get(i).equals(grid.cellGrid[t][d][row][col])){
                                     Planner.history += "{" + col + "," + row + "}";
                                     pairsWritten++;
                                     if(pairsWritten < visitedSet.size()){
                                             Planner.history += ",";
                                     }
                                 }
                             }
                        }    
                   }
              }
         }
         Planner.history += "}";
         if(!last){
                 Planner.history += ",";
         }
    }
	
    
    /* Method: recordHistory
     * 
     * Input: File to write to 
     * 
     * Output: writes the history of coordinates to the file as a list
     * 
     * Details: Written by Sherman Lam. Edited a bit to work with my code.
     *  This method is used for debugging and tracing purposes in 
     *  Mathematica.
     */
    public static void recordHistory(File file){
            PrintWriter outputStream = null;
            try{
                    String toWrite = "{" + Planner.history + "}";
                    outputStream = new PrintWriter(new FileWriter(file));
                    outputStream.write(toWrite);
            }
            catch(IOException e){
                    System.out.println("Could not write to file." + e.getMessage());
            }
            finally{
                    if(outputStream != null){
                            outputStream.close();
                    }
            }
    }
    

    public static void main(String args[]) {
        
        ArrayList<OceanCell> path = PathPlanner();       
        // if there is a path, write the mission file and kml file to view path
        if (path != null) {
            System.out.println(path.size() + " waypoints");
            writeMissionPlan.writeTextMission(outputFile, path);
            if (SearchAlg != "TEST") {
                writeMissionPlan.writeKMLMission(KMLFile, path);
            }
        }     
    }  
    
}
