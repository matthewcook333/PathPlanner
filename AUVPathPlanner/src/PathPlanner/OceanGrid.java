/*
 *
 */
package PathPlanner;

import java.io.IOException;
import java.util.List;
import ucar.ma2.Array;
import ucar.ma2.InvalidRangeException;
import ucar.nc2.NetcdfFile;
import ucar.nc2.Variable;
import ucar.nc2.dataset.CoordinateSystem;
import ucar.nc2.dataset.NetcdfDataset;
import ucar.nc2.dt.GridCoordSystem;
import ucar.nc2.dt.grid.GridCoordSys;

/**
 *
 * @author Matt Cook
 * 
 * Class: OceanGrid
 * 
 * Description: This class is used primarily as a 4-D array
 * used to hold objects of the class OceanCell. This class is used to read
 * in netCDF files in java arrays for easier handling and access.
 */
public class OceanGrid {
    
    // arrays to initially hold data from netCDF files
    static double[] timeArray;
    static double[] latArray;
    static double[] lonArray;
    static double[] depArray;
    static double[][][][] temp;
    static double[][][][] salin;
    static double[][][][] uCurrents;
    static double[][][][] vCurrents;
    static double[][][][] tempErr;
    static double[][][][] salinErr;
    static double[][][][] uCurrentsErr;
    static double[][][][] vCurrentsErr;
    
    // dimensions of the grid
    int NDEPTH = depArray.length;
    int NLAT = latArray.length;
    int NLON = lonArray.length;
    int NTIME = timeArray.length;
    
    //4-D array which is the grid to hold the OceanCells
    public OceanCell[][][][] cellGrid = null;
    
    
    // returns the length of this OceanGrid 
    public int getLatLength() {
        return latArray.length;
    }
    
    // returns the length of this OceanGrid 
    public int getLonLength() {
        return lonArray.length;
    }
    
    
    // Two argument constructor.
    // If used, need to manually fill in grid with specified values
    public OceanGrid(int lat, int lon) {
        NDEPTH = depArray.length;
        NLAT = lat;
        NLON = lon;
        NTIME = timeArray.length;
        cellGrid = new OceanCell[NTIME][NDEPTH][NLAT][NLON];
    }
    
    
    /*
     * Constructor for the OceanGrid using the dimensions from the netCDF file.
     * Constructor is used after reading in the file, as this takes the 
     * data from the multiple arrays and puts it all into a 4-D 
     * array of OceanCells, each of which holds all the information provided.
     */
    public OceanGrid() {
        NDEPTH = depArray.length;
        NLAT = latArray.length;
        NLON = lonArray.length;
        NTIME = timeArray.length;
        cellGrid = new OceanCell[NTIME][NDEPTH][NLAT][NLON];

        for (int t = 0; t < NTIME; t++) {
            for (int dep = 0; dep < NDEPTH; dep++) {
                for (int lat = 0; lat < NLAT; lat++) {
                    for (int lon = 0; lon < NLON; lon++) {
                        OceanCell currentCell = new OceanCell(t, lat, lon, dep,
                            timeArray[t], latArray[lat], lonArray[lon],
                            depArray[dep], temp[t][dep][lat][lon],
                            salin[t][dep][lat][lon], uCurrents[t][dep][lat][lon],
                            vCurrents[t][dep][lat][lon], tempErr[t][dep][lat][lon],
                            salinErr[t][dep][lat][lon], uCurrentsErr[t][dep][lat][lon],
                            vCurrentsErr[t][dep][lat][lon]);
                        cellGrid[t][dep][lat][lon] = currentCell;
                    }
                }
            }
        }
    }
    
    /*
     * getPortion
     * 
     * inputs: boundary coordinates of latitude and longitude.
     * 
     * output: This function outputs an OceanGrid of smaller dimensions
     * given the provided boundaries.
     */
    /*
     * CURRENTLY NOT USING THIS METHOD ANYMORE
    public OceanGrid getPortion(double lowLat, double highLat,
            double lowLon, double highLon) {
        
        int[] start = this.findIndex(lowLat, lowLon);
        int[] end = this.findIndex(highLat, highLon);
        int latSize = Math.abs(end[0] - start[0]) + 1;
        int lonSize = Math.abs(end[1] - start[1]) + 1;
       
        //System.out.println(start[0] + ", " + start[1] + ", " + end[0] + ", " + end[1] );
        //NOT NECESSARY IF USING FINDINDEX FOR START AND END
        //double latDiff = Math.abs(highLat - lowLat);
        //double lonDiff = Math.abs(highLon - lowLon);
        //int latSize = (int)Math.round(latDiff/PathPlanner.interval) + 1;
        //int lonSize = (int)Math.round(lonDiff/PathPlanner.interval) + 1; 
        
        NDEPTH = depArray.length;
        OceanGrid newOceanGrid = new OceanGrid();
        newOceanGrid.cellGrid = new OceanCell[Planner.NTIME][NDEPTH][latSize][lonSize];
        newOceanGrid.NLAT = latSize;
        newOceanGrid.NLON = lonSize;
        newOceanGrid.NDEPTH = this.NDEPTH;
        newOceanGrid.NTIME = this.NTIME;
         
        int x = start[0];
        int y = start[1];
        for (int t = 0; t < Planner.NTIME; t++) {
            for (int dep = 0; dep < NDEPTH; dep++) {
                for (int lat = 0; lat < latSize; lat++) {
                    for (int lon = 0; lon < lonSize; lon++) {
                        newOceanGrid.cellGrid[t][dep][lat][lon] = this.getCell(t,dep,x,y);
                        newOceanGrid.cellGrid[t][dep][lat][lon].setIndex(t, dep, lat, lon);
                        ++y;
                    }
                    y = start[1];
                    ++x;
                }
                y = start[1];
                x = start[0];
            }
            y = start[1];
            x = start[0];
        }
        return newOceanGrid;
    }
   */
    
    /*
     * getCell
     * 
     * input: four integers, which represent the indices of an OceanCell within
     * an OceanGrid. The four inputs are of time, depth, latitude, and 
     * longitude.
     * 
     * output: returns the OceanCell at those indicies, or prints a statement
     * stating that the cell is outside the OceanGrid's dimensions, and returns
     * null.
     */
    // access OceanCell in grid
    public OceanCell getCell(int time, int depth, int lat, int lon) {
        if (lat >= NLAT || lon >= NLON || depth >= NDEPTH || time >= NTIME) {
            System.out.println("Accessing cell [" + time + ", " + depth +
                    ", " + lat + ", " + lon + "] is outside of the dimensions"
                    + " of the grid!");
            return null;    
        }
        return this.cellGrid[time][depth][lat][lon];
    }
    
    /*
     * findIndex
     * 
     * input: latitude and longitude coordinates, both doubles
     * 
     * output: an integer array of length two that correspond to the indices
     * of those latitude and longitude coordinates, in that order,
     * within the OceanGrid.
     */
    /*
     * CURRENTLY NOT USING METHOD ANYMORE
    public int[] findIndex(double lat, double lon) {
        // array that is lat,lon
        int[] index = new int[2];   
        boolean foundIndex = false;
        outerloop:
        for (int x = 0; x < NLAT; x++) {
            for (int y = 0; y < NLON; y++) {
                index[0] = x;
                index[1] = y; 
                
                //System.out.println(cellGrid[0][0][x][y].getLatValue() + ", "
                      // + cellGrid[0][0][x][y].getLonValue());
                
                
                if (cellGrid[0][0][x][y].getLatValue() >= (lat - Planner.prec)
                      && cellGrid[0][0][x][y].getLatValue() <= (lat + Planner.prec)
                      && cellGrid[0][0][x][y].getLonValue() >= (lon - Planner.prec)
                      && cellGrid[0][0][x][y].getLonValue() <= (lon + Planner.prec)) {
                    foundIndex = true;
                    break outerloop;        
                }
            }
        }
        if (!foundIndex) {
            System.out.println("The coordinates " + lat + ", " + lon + 
                    " could not be found in given area!");
            return null;
        }
        return index;  
    }
    */
    
    /*
     * readFile
     * 
     * input: two Strings which are the files to be read in. The first file
     *  is the forecast file which has the mean data. The second file is the 
     *  ensemble mode data which gives the uncertainty data of each variable.
     * 
     * output: void
     * 
     * description: This function takes in two netCDF file (file ending in .nc)
     * and reads in the variables latitude, longitude, depth, temperature,
     * and salinity, into multiple primitive arrays. Also reads the uncertainty
     * in the variables into primitive arrays. These arrays can then be
     * used by the OceanGrid constructor to create OceanCells within the grid.
     */
    public static void readFile(String fileName, String errFileName) {
       
        // Open the file.
        NetcdfFile dataFile = null;
        NetcdfDataset dataSet;
        NetcdfFile errFile = null;
        try {
            // Open the netCDF file
            dataFile = NetcdfDataset.openFile(fileName, null);
            errFile = NetcdfDataset.openFile(errFileName, null);
            dataSet = NetcdfDataset.acquireDataset(fileName, null);
            
            //create grid coordinate system to determine search boundaries
            List<CoordinateSystem> csList = dataSet.getCoordinateSystems();
            if (csList.get(0) == null) {
                System.out.println("There is no useable coordinate system in "
                        + "this file!");
                System.exit(-1);
            }
            System.out.println("The coordinate system used is [" 
                    + csList.get(0) + "]");
            CoordinateSystem cs = csList.get(0);
            GridCoordSystem gcs = new GridCoordSys(cs, null);
				
            //find the indices that the coord correspond to
            int[] lowBounds;
            int[] highBounds;
            //handles coord in lat, lon form
            lowBounds = gcs.findXYindexFromLatLonBounded(
                    Planner.LOWLAT, Planner.LOWLON, null);
            highBounds = gcs.findXYindexFromLatLonBounded(
                    Planner.HIGHLAT, Planner.HIGHLON, null);
            //While the file is open, look for the lat and lon Indices of
            // start coordinates
            Planner.startIndex = gcs.findXYindexFromLatLonBounded(
                    Planner.latStart, Planner.lonStart, null);
            // Adjust for index change with boundaries
            Planner.startIndex[1] -= lowBounds[1];
            Planner.startIndex[0] -= lowBounds[0];
            System.out.println("start index: " + Planner.startIndex[1] + ", " + Planner.startIndex[0]);
            
            if (Planner.findDest) {
                Planner.destIndex = gcs.findXYindexFromLatLonBounded(
                Planner.latDest, Planner.lonDest, null);
                // Adjust for index change with boundaries
                Planner.destIndex[1] -= lowBounds[1];
                Planner.destIndex[0] -= lowBounds[0];
                if (Planner.findDest) {
                    System.out.println("dest index: "+ Planner.destIndex[1] + ", " + Planner.destIndex[0]);
                }
            }

            System.out.println("------Boundary Indices-----");
            System.out.println("LON: " + lowBounds[0] + " to " + highBounds[0]
                    + " (" + (highBounds[0]-lowBounds[0]+1) + " columns)");
            System.out.println("LAT: " + lowBounds[1] + " to " + highBounds[1]
                    + " (" + (highBounds[1]-lowBounds[1]+1) + " rows)");
            System.out.println("---------------------------");
            
            int depthDim = Planner.depthDim;
            int timeDim = Planner.hourEndIndex - Planner.hourStartIndex + 1;
            int latDim = highBounds[1] - lowBounds[1] + 1;
            int lonDim = highBounds[0] - lowBounds[0] + 1;
            
            // Getting lat, lon, depth, and time data from file
            Variable timeVar = dataFile.findVariable("time");
            if (timeVar == null) {
                System.out.println("Cant find Variable time");
                return;
            }           
            Variable latVar = dataFile.findVariable("lat");
            if (latVar == null) {
                System.out.println("Cant find Variable Latitude");
                return;
            }       
            Variable lonVar = dataFile.findVariable("lon");
            if (lonVar == null) {
                System.out.println("Cant find Variable longitude");
                return;
            }
            Variable depVar = dataFile.findVariable("depth");
            if (depVar == null) {
                System.out.println("Cant find Variable depth");
                return;
            }

            Array latArr, lonArr, depArr, timeArr;
              
            int[] latShape = {latDim};
            int[] latOrigin = {lowBounds[1]};
            int[] lonShape = {lonDim};
            int[] lonOrigin = {lowBounds[0]};
            int[] depthShape = {depthDim};
            int[] depthOrigin = {0};
            int[] timeShape = {timeDim};
            int[] timeOrigin = {Planner.hourStartIndex};

            latArr = (Array) latVar.read(latOrigin, latShape);
            lonArr = (Array) lonVar.read(lonOrigin, lonShape);
            depArr = (Array) depVar.read(depthOrigin, depthShape);
            timeArr = (Array) timeVar.read(timeOrigin, timeShape);
            
            if (latArr.getElementType() == float.class) {
                float[] latFloatArray = (float[]) latArr.copyToNDJavaArray();
                float[] lonFloatArray = (float[]) lonArr.copyToNDJavaArray();
                float[] depFloatArray = (float[]) depArr.copyToNDJavaArray();
                float[] timeFloatArray = (float[]) timeArr.copyToNDJavaArray();
                latArray = new double[latDim];
                lonArray = new double[lonDim];
                depArray = new double[depthDim];
                timeArray = new double[timeFloatArray.length];
                for (int i = 0; i < latFloatArray.length; ++i) {
                    latArray[i] = (double)latFloatArray[i];
                }
                for (int i = 0; i < lonFloatArray.length; ++i) {
                   lonArray[i] = (double)lonFloatArray[i];
                }
                for (int i = 0; i < depFloatArray.length; ++i) {
                    depArray[i] = (double)depFloatArray[i];
                } 
                for (int i = 0; i < timeFloatArray.length; ++i) {
                    timeArray[i] = (double)timeFloatArray[i];
                }
            }
            else {
                latArray = (double[]) latArr.copyToNDJavaArray();
                lonArray = (double[]) lonArr.copyToNDJavaArray();
                depArray = (double[]) depArr.copyToNDJavaArray();
                timeArray = (double[]) timeArr.copyToNDJavaArray();
            }
                       
            // Get the temperature, salinity, and currents variables.
            Variable tempVar = dataFile.findVariable("temp");
            if (tempVar == null) {
                System.out.println("Cant find Variable Temperature");
                return;
            }

            Variable salinVar = dataFile.findVariable("salt");
            if (salinVar == null) {
                System.out.println("Cant find Variable Salinity");
                return;
            }

            Variable uVar = dataFile.findVariable("u");
            if (uVar == null) {
                System.out.println("Cant find Variable u(zonal currents)");
                return;
            }

            Variable vVar = dataFile.findVariable("v");
            if (vVar == null) {
                System.out.println("Cant find Variable v(meridional currents)");
                return;
            }
            
            // Get the uncertainty in temperature, salinity, 
            // and currents variables from ensemble data
            Variable tempVarErr = errFile.findVariable("temp");
            if (tempVarErr == null) {
                System.out.println("Cant find Variable TemperatureErr");
                return;
            }

            Variable salinVarErr = errFile.findVariable("salt");
            if (salinVarErr == null) {
                System.out.println("Cant find Variable SalinityErr");
                return;
            }

            Variable uVarErr = errFile.findVariable("u");
            if (uVarErr == null) {
                System.out.println("Cant find Variable uErr(zonal currents)");
                return;
            }

            Variable vVarErr = errFile.findVariable("v");
            if (vVarErr == null) {
                System.out.println("Cant find Variable vErr(meridional currents)");
                return;
            }
            
            
            System.out.println("Reading data....");
            
            int[] shape = {timeDim, depthDim, latDim, lonDim};
            int[] origin = {0, 0, lowBounds[0], lowBounds[1]};
            
            // read 3D array for that index
            Array tempArray, salinArray, uArray, vArray;
            Array tempErrArray, salinErrArray, uErrArray, vErrArray;

            tempArray = (Array) tempVar.read(origin, shape);
            salinArray = (Array) salinVar.read(origin, shape);
            uArray = (Array) uVar.read(origin, shape);
            vArray = (Array) vVar.read(origin, shape);

            tempErrArray = (Array) tempVarErr.read(origin, shape);
            salinErrArray = (Array) salinVarErr.read(origin, shape);
            uErrArray = (Array) uVarErr.read(origin, shape);
            vErrArray = (Array) vVarErr.read(origin, shape);

            if (tempArray.getElementType() == float.class) {
                float[][][][] tempFloat = (float[][][][]) tempArray.copyToNDJavaArray();
                float[][][][] salinFloat = (float[][][][]) salinArray.copyToNDJavaArray();
                float[][][][] uFloat = (float[][][][]) uArray.copyToNDJavaArray();
                float[][][][] vFloat = (float[][][][]) vArray.copyToNDJavaArray();
                temp = new double[timeDim][depthDim][latDim][lonDim];
                salin = new double[timeDim][depthDim][latDim][lonDim];
                uCurrents = new double[timeDim][depthDim][latDim][lonDim];
                vCurrents = new double[timeDim][depthDim][latDim][lonDim];
                for (int t = 0; t < timeDim; ++t) {
                    for (int i = 0; i < latDim; ++i) {
                        for (int j = 0; j < lonDim; ++j) {
                            for (int k = 0; k < depthDim; ++k) {
                                temp[t][k][i][j] = (double)tempFloat[t][k][i][j];
                                salin[t][k][i][j] = (double)salinFloat[t][k][i][j];
                                uCurrents[t][k][i][j] = (double)uFloat[t][k][i][j];
                                vCurrents[t][k][i][j] = (double)vFloat[t][k][i][j];  
                            }
                        }
                    }  
                }
            }
            else {
                temp = (double[][][][]) tempArray.copyToNDJavaArray();
                salin = (double[][][][]) salinArray.copyToNDJavaArray();
                uCurrents = (double[][][][]) uArray.copyToNDJavaArray();
                vCurrents = (double[][][][]) vArray.copyToNDJavaArray();
            }
            tempErr = (double[][][][]) tempErrArray.copyToNDJavaArray();
            salinErr = (double[][][][]) salinErrArray.copyToNDJavaArray();
            uCurrentsErr = (double[][][][]) uErrArray.copyToNDJavaArray();
            vCurrentsErr = (double[][][][]) vErrArray.copyToNDJavaArray();
        

       // The file is closed no matter what by putting inside a try/catch block.
        } catch (java.io.IOException | InvalidRangeException e) {
            e.printStackTrace();
            return;
        } finally {
            if (dataFile != null) {
                try {
                    dataFile.close();
                } catch (IOException ioe) {
                    ioe.printStackTrace();
                }
            }
            if (errFile != null) {
                try {
                    dataFile.close();
                } catch (IOException ioe) {
                    ioe.printStackTrace();
                }
            }
        }
        System.out.println("Successfully read file " + fileName + " and file "
                + errFileName);
    
    }
    
    /*
     * Method: ReduceResolution
     * 
     * Input:
     * 
     * Output:
     * 
     * Details: STILL NEED TO DO. Method used to get coarser resolution grids.
     */
    public OceanGrid ReduceResolution() {
        OceanGrid newGrid = new OceanGrid(this.NLAT/3, this.NLON/3);
        for (int t = 0; t < NTIME; ++t) {
            for (int d = 0; d < NDEPTH; ++d) {
                for (int i = 0; i < newGrid.NLAT; ++i) {
                    for (int j = 0; j < newGrid.NLON; ++j) {
                        OceanCell cell = getCell(t, d, (3*i)+1, (3*j)+1);
                        double averageTemp = averageData(cell, this);
                        newGrid.cellGrid[t][d][i][j] = cell;
                    }
                }
            }
        }
        return newGrid;    
    }
   
    /*
     * Method: averageData
     * 
     * Input: OceanCell that is the center cell, and the grid currently used
     * 
     * Output: double that is the output of the averaged data from the neighbors
     *  of a given cell
     * 
     * Details: TODO. HAVE NOT COMPLETED. Would be used in conjunction with
     *  ReduceResolution.
     */
    public static double averageData(OceanCell cell, OceanGrid grid) {
        
        double averageTemp = 0;
        double averageSalin = 0;
        double averageU = 0;
        double averageV = 0;
        int[] directions = {-1,-1, -1,0, -1,1, 0,-1, 0,0,
            0,1, 1,-1, 1,0, 1,1};
        int numDirections = 9;
        
        // Look at all the neighbors including self, 
        // and assuming all neighbors are valid
        for (int dir = 0; dir < numDirections; dir++) {
            // Get the indices for the neighbor with the array of possible
            // directions
            int dirx = directions[dir * 2];
            int diry = directions[dir * 2 + 1];
            // These are INDICES
            int newx = cell.getLat() + dirx;
            int newy = cell.getLon() + diry;
            int t = cell.getTime();
            // the 0 is for depth
            OceanCell neighbor = grid.getCell(t,0,newx,newy);
            // Do this for all data members
            averageTemp += neighbor.getTemp();
        }
        //averageTemp = averageTemp/numDirections;
        OceanCell averagedCell = new OceanCell(cell);
        //averagedCell.setTemp(averageTemp);
           
        return 0.0;
    }
}
