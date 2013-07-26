/*
 *
 */
package PathPlanner;

import static PathPlanner.AStarPlanner.addObjective;
import static PathPlanner.AStarPlanner.objectiveEstimate;
import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 *
 * @author Matt Cook
 * 
 * Class: OceanPath
 * 
 * Description: This class is used to hold paths used for path planning. It
 *  uses an ArrayList to hold the cells on the path, and has data members for 
 *  the time elapsed, max and min temp found on the path, and the current
 *  reward (objective) gathered which is the gScore, and fScore which is the sum
 *  of the gScore and the predicted reward remaining for the path.
 */
public class OceanPath {
    
    public ArrayList<OceanCell> path;
    
    // data members used for path planning
    public double timeElapsed;
    public double distance;
    public double maxTemp;
    public double minTemp;
    // gScore is the reward (objective) gathered so far
    public double gScore;
    // fScore is the sum of the gScore and the predicted reward remaining
    public double fScore;
    

    /*
    * Constructor of the OceanCell at time, lat, lon, depth, with the
    * information at that cell from the ROMS data
    */
    public OceanPath() {      
        this.path = new ArrayList<OceanCell>();
        this.timeElapsed = 0;
        this.distance = 0;
        this.maxTemp = -9999;
        this.minTemp = 9999;
        this.gScore = 0;
        this.fScore = 0;    
    }
    
    /*
     * Copy constructor for the OceanPath
     */
    public OceanPath(OceanPath orig) {
        this.path = new ArrayList<OceanCell>(orig.path);
        this.timeElapsed = orig.timeElapsed;
        this.maxTemp = orig.maxTemp;
        this.minTemp = orig.minTemp;
        this.gScore = orig.gScore;
        this.fScore = orig.fScore;
        this.distance = orig.distance;
    }
  
    /*
     * toString()
     * 
     * method called when a OceanCell is passed as an argument to
     * System.out.println
     */
    @Override
    public String toString() {
        DecimalFormat myFormat = new DecimalFormat("0.000");
        /* real one
            return "[" + latitude + "\u00B0, " + longitude + "\u00B0, " +
                      depth + " m], Temperature:" + temperature + 
                    " Score:" + gScore + ", Arrival Time"
                    + " since " + time + ": " + timeArrived + " secs";
            ONE USED FOR DEBUGGING
             return "Path length is " + path.size() + " cells, Time elapsed:" 
                + myFormat.format((timeElapsed/3600)) + " hours, Score:" + 
                myFormat.format(gScore) +
                ", Predicted remaining:" + myFormat.format((fScore-gScore));
                    */
        return "Path length is " + path.size() + " cells, Time elapsed:" 
                + myFormat.format((timeElapsed/3600)) + " hours, Score:" + 
                myFormat.format(gScore) +
                ", Distance:" + myFormat.format(distance) + " km.";
    }  
    
    /*
     * equals operator
     */
    public boolean equals(OceanPath rhs) {
        if (rhs == null && this == null) {
            return true;
        }
        if (rhs == null || this == null) {
            return false;
        }
        return (
            this.path == rhs.path &&
            this.distance == rhs.distance &&
            this.timeElapsed == rhs.timeElapsed &&
            this.maxTemp == rhs.maxTemp &&
            this.minTemp == rhs.minTemp &&
            this.gScore == rhs.gScore &&
            this.fScore == rhs.fScore
            );
    }    
    
    /*
     * Method: recordData
     * 
     * Input: Non-static method called upon a path, and input is an OceanGrid,
     *  and a double that is the max mission length
     * 
     * Output: Void
     * 
     * Details: Records the accurate time arrived, gScore, and fScore in each
     *  cell, so that the cells output accurate information for printing.
     */
    public void recordData(OceanGrid grid, double maxMissionTime) {
        maxTemp = minTemp = path.get(0).getTemp();
        for (int i = 1; i < path.size(); ++i) {
            OceanCell parent = path.get(i-1);
            OceanCell cell = path.get(i);
            double timeTraveled = AUV.travelTime(parent, cell);
            cell.timeArrived = parent.timeArrived + timeTraveled;
            ArrayList<OceanCell> subPath = new ArrayList<OceanCell>();
            for (int j = 0; j < i; ++j) {
                subPath.add(path.get(j));
            }
            OceanPath tempPath = new OceanPath(this);
            tempPath.path = subPath;
            cell.gScore = parent.gScore 
                            + addObjective(tempPath, cell);
            tempPath.path.add(path.get(i));
            cell.fScore = cell.gScore +  objectiveEstimate(tempPath, grid, maxMissionTime);
        }
    }
    
    /* 
     * Below are methods that can be called on OceanPath that are standard
     *  to the ArrayList library. These include: add, isEmpty, clear, get, 
     *  size, and contains.
     */
    public boolean add(OceanCell cell) {
        if (cell.getTemp() < minTemp) {
            minTemp = cell.getTemp();
        }
        if (cell.getTemp() > maxTemp) {
            maxTemp = cell.getTemp();
        }
        return path.add(cell);
    }
    
    public boolean isEmpty() {
        return path.isEmpty();
    }
    
    public void clear() {
        // NOTE: Also resets the data members
        timeElapsed = 0;
        minTemp = 9999;
        maxTemp = -9999;
        gScore = 0;
        fScore = 0;
        path.clear();
    }
    
    public OceanCell get(int i) {
        return path.get(i);
    }
    
    public int size() {
        return path.size();
    }
    
    public boolean contains(OceanCell cell) {
        // Checks whether the spatial cell is in the path (which means it 
        // could be from a different time)
        for (int i = 0; i < this.size(); ++i) {
            if (path.get(i).equals(cell)) {
                return true;
            }
        }
        return false;
    }    
}
