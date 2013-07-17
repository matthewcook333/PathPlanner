/*
 *
 */
package PathPlanner;

import static PathPlanner.AStarPlanner.addObjective;
import static PathPlanner.AStarPlanner.objectiveEstimate;
import java.util.ArrayList;

/**
 *
 * @author atthewco
 */
public class OceanPath {
    
    public ArrayList<OceanCell> path;
    
    // data members used for path planning
    public double latestTime;
    public double maxTemp;
    public double minTemp;
    public double gScore;
    public double fScore;

    /*
    * Constructor of the OceanCell at time, lat, lon, depth, with the
    * information at that cell from the ROMS data
    */
    public OceanPath() {      
        this.path = new ArrayList<>();
        this.latestTime = 0;
        this.maxTemp = -9999;
        this.minTemp = 9999;
        this.gScore = 0;
        this.fScore = 0;    
    }
    
    public OceanPath(OceanPath orig) {
        this.path = new ArrayList<>(orig.path);
        this.latestTime = orig.latestTime;
        this.maxTemp = orig.maxTemp;
        this.minTemp = orig.minTemp;
        this.gScore = orig.gScore;
        this.fScore = orig.fScore;     
    }
  
    /*
     * toString()
     * 
     * method called when a OceanCell is passed as an argument to
     * System.out.println
     */
    @Override
    public String toString() {
        /* real one
            return "[" + latitude + "\u00B0, " + longitude + "\u00B0, " +
                      depth + " m], Temperature:" + temperature + 
                    " Score:" + gScore + ", Arrival Time"
                    + " since " + time + ": " + timeArrived + " secs";
                    */
        return "Path length is " + path.size() + " cells, time elapsed:" + latestTime
                + " sec. Accumulated objective:" + gScore + ", predicted remaining:"
                + (fScore-gScore);
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
            this.latestTime == rhs.latestTime &&
            this.maxTemp == rhs.maxTemp &&
            this.minTemp == rhs.minTemp &&
            this.gScore == rhs.gScore &&
            this.fScore == rhs.fScore
            );
    }    
    
    public void recordData(OceanGrid grid) {
        maxTemp = minTemp = path.get(0).getTemp();
        for (int i = 1; i < path.size(); ++i) {
            OceanCell parent = path.get(i-1);
            OceanCell cell = path.get(i);
            double timeTraveled = AUV.travelTime(parent, cell);
            cell.timeArrived = parent.timeArrived + timeTraveled;
            ArrayList<OceanCell> subPath = new ArrayList<>();
            for (int j = 0; j < i; ++j) {
                subPath.add(path.get(j));
            }
            OceanPath tempPath = new OceanPath(this);
            tempPath.path = subPath;
            cell.gScore = parent.gScore 
                            + addObjective(tempPath, cell);
            tempPath.path.add(path.get(i));
            cell.fScore = cell.gScore +  objectiveEstimate(tempPath, grid);
        }
    }
    
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
        minTemp = 9999;
        maxTemp = -9999;
        path.clear();
    }
    
    public OceanCell get(int i) {
        return path.get(i);
    }
    
    public int size() {
        return path.size();
    }
    
    public boolean contains(OceanCell cell) {
        for (int i = 0; i < this.size(); ++i) {
            if (path.get(i).equals(cell)) {
                return true;
            }
        }
        return false;
    }
    
    
}
