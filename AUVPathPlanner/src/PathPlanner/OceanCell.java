/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import java.util.Stack;

/**
 *
 * @author atthewco
 */
public class OceanCell {

    private double time;
    private double latitude;
    private double longitude;
    private double depth;
    // Indices to for the position of cell in a grid
    private int timeIndex;
    private int latIndex;
    private int lonIndex;
    private int depIndex;
    // temperature and salinity data
    private double temperature;
    private double salinity;
    //Zonal Current
    private double u;
    //Meridional Current
    private double v;
    
    private double tempErr;
    private double salinErr;
    private double uErr;
    private double vErr;
    
    // data members used for path planning
    public double timeArrived;
    public double gScore;
    public double fScore;
    public Stack<Integer> treeDepth;

    /*
    * Constructor of the OceanCell at time, lat, lon, depth, with the
    * information at that cell from the ROMS data
    */
    public OceanCell(int tI, int latI, int lonI, int depI,
            double time, double latit, double longit, double dep,
            double temp, double salin, double zonalC, double meridC,
            double tempErr, double salinErr, double zonalCErr, double meridCErr) {
        
        this.timeIndex = tI;
        this.latIndex = latI;
        this.lonIndex = lonI;
        this.depIndex = depI;
        this.time = time;
        this.latitude = (double) latit;
        this.longitude = (double) longit;
        this.timeArrived = 0;
        this.depth = (double) dep;
        this.temperature = (double) temp;
        this.salinity = (double) salin;
        this.u = (double) zonalC;
        this.v = (double) meridC;
        this.tempErr = tempErr;
        this.salinErr = salinErr;
        this.uErr = zonalCErr;
        this.vErr = meridCErr;
         
        this.gScore = 0;
        this.fScore = 0;
        this.treeDepth = new Stack<>();
        
    }
    
    public OceanCell(OceanCell orig) {
        this.time = orig.time;
        this.timeIndex = orig.timeIndex;
        this.latIndex = orig.latIndex;
        this.lonIndex = orig.lonIndex;
        this.depIndex = orig.depIndex;
        this.latitude = orig.latitude;
        this.longitude = orig.longitude;
        this.timeArrived = orig.timeArrived;
        this.depth = orig.depth;
        this.temperature = orig.temperature;
        this.salinity = orig.salinity;
        this.u = orig.u;
        this.v = orig.v;
        this.tempErr = orig.tempErr;
        this.salinErr = orig.salinErr;
        this.uErr = orig.uErr;
        this.vErr = orig.vErr;
        this.gScore = orig.gScore;
        this.fScore = orig.fScore;
        this.treeDepth = orig.treeDepth;
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
        return "[" + latIndex + ", " + lonIndex + ", " +
                      depIndex + "], Temperature Err:" + tempErr + 
                    " Score:" + gScore + ", Arrival Time"
                    + " since " + timeIndex + ": " + timeArrived + " secs";
        
    }  
    
    public String printIndex() {
        /* real one
            return "[" + latitude + "\u00B0, " + longitude + "\u00B0, " +
                      depth + " m], Temperature:" + temperature + 
                    " Score:" + gScore + ", Arrival Time"
                    + " since " + time + ": " + timeArrived + " secs";
                    */
        return "[" + latIndex + ", " + lonIndex + ", " +
                      depIndex + "]";
        
    }  
    
    /*
     * equals operator
     */
    public boolean equals(OceanCell rhs) {
        if (rhs == null && this == null) {
            return true;
        }
        if (rhs == null || this == null) {
            return false;
        }
        return (
            this.latIndex == rhs.latIndex &&
            this.lonIndex == rhs.lonIndex &&
            this.depIndex == rhs.depIndex &&
            this.latitude == rhs.latitude &&
            this.longitude == rhs.longitude &&
            this.depth == rhs.depth
            );
    }
    
    /*
     * copyData method for copying over the public data members used for
     * path planning
     */
    public void copyData(OceanCell orig) {
        this.timeArrived = orig.timeArrived;
        this.gScore = orig.gScore;
        this.fScore = orig.fScore;
        this.treeDepth = orig.treeDepth;
    }
 
    
    
    /*
     * getTime()
     * 
     * method that returns index of the time.
     */
    public int getTime() {
        return this.timeIndex;
    }  
    
     /*
     * getLat()
     * 
     * method that returns index of the latitude.
     */
    public int getLat() {
        return this.latIndex;
    }  
    
    /*
     * getLon()
     * 
     * method that returns index of the longitude.
     */
    public int getLon() {
        return this.lonIndex;
    }
    
    /*
     * getDepth()
     * 
     * method that returns index of the depth.
     */
    public int getDepth() {
        return this.depIndex;
    }
    
    /*
     * getTimeValue()
     * 
     * method that returns value of the time.
     */
    public double getTimeValue() {
        return this.time;
    }  
    
    /*
     * getLatValue()
     * 
     * method that returns the value of the latitude.
     */
    public double getLatValue() {
        return this.latitude;
    }  
    
    /*
     * getLonValue()
     * 
     * method that returns value of the longitude.
     */
    public double getLonValue() {
        return this.longitude;
    }
    
    /*
     * getDepthValue()
     * 
     * method that returns value of the depth.
     */
    public double getDepthValue() {
        return this.depth;
    }
    
    
    /*
     * getTemp()
     * 
     * method that returns value of the temperature
     */
    public double getTemp() {
        return this.temperature;
    }
    
    /*
     * getSalin()
     * 
     * method that returns value of the salinity.
     */
    public double getSalin() {
        return this.salinity;
    }
    
    /*
     * getU()
     * 
     * method that returns the zonal current.
     */
    public double getU() {
        return this.u;
    }
 
    /*
     * getV()
     * 
     * method that returns the meridional current.
     */
    public double getV() {
        return this.v;
    }
    
        /*
     * getTempErr()
     * 
     * method that returns value of the error in temperature
     */
    public double getTempErr() {
        return this.tempErr;
    }
    
    /*
     * getSalinErr()
     * 
     * method that returns value of the error in salinity.
     */
    public double getSalinErr() {
        return this.salinErr;
    }
    
    /*
     * getUErr()
     * 
     * method that returns the error in zonal current.
     */
    public double getUErr() {
        return this.uErr;
    }
 
    /*
     * getVErr()
     * 
     * method that returns the error in meridional current.
     */
    public double getVErr() {
        return this.vErr;
    }
}


