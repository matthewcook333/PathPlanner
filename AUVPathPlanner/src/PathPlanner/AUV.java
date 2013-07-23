/*
 * 
 */
package PathPlanner;


/**
 *
 * @author Matt Cook
 * 
 * Class: AUV
 * 
 * Description: Used for methods to calculate distance between lat and lon
 *  coordinates, as well as for calculating the time it takes for the AUV
 *  to travel between points given AUV propulsion and speed and direction of the
 *  ocean currents.
 */
public class AUV {
    
    /*
     * Method: travelTime
     * 
     * Input: OceanCell traveling from, OceanCell traveling to.
     * 
     * Output: double that is the time it takes to travel between the cells in 
     *  seconds.
     * 
     * Details: Code based on MATLAB glider code written by David R. Thompson.
     *  Uses the ocean currents information based upon the cell traveling from,
     *  and AUV propulsion specified in the Planner. Returns -1 if it is not
     *  possible to travel between cells due to opposing ocean currents speeds
     *  being stronger than AUV propulsion.
     */
    public static double travelTime(OceanCell currentCell, OceanCell neighbor) {
        
        double latStart = currentCell.getLatValue();
        double lonStart = currentCell.getLonValue();
        double latEnd = neighbor.getLatValue();
        double lonEnd = neighbor.getLonValue();
        double xCurrent = currentCell.getU();
        double yCurrent = currentCell.getV();
        double propulsion = Planner.propulsion;
        
        double dist = distance(latStart, lonStart, latEnd, lonEnd, 'K');
        // horiz and vert distances to determine direction vectors
        double xDist = distance(latStart, lonStart, latEnd, lonStart, 'K');
        double yDist = distance(latStart, lonStart, latStart, lonEnd, 'K');
        
        if ((neighbor.getLat() - currentCell.getLat()) < 0) {
            yDist = -yDist;
        }
        if ((neighbor.getLon() - currentCell.getLon()) < 0) {
            xDist = -xDist;
        }
        
        // case where we are stationary
        if (dist == 0) {
            double currentMag = 
                    Math.sqrt(Math.pow(xCurrent, 2) + Math.pow(yCurrent, 2));
            if (currentMag > propulsion) {
                // cannot stay stationary if currents are too strong
                return -1; 
            }
            else {
                return currentMag;
            }
        }
        
        // desired direction to travel in
        double yDirection = yDist/dist;
        double xDirection = xDist/dist;
        
        // direction that is orthogonal to the desired direction
        double yOrthogonal = xDist/dist;
        double xOrthogonal = -yDist/dist;
        
        // calculate the current orthogonal to our direction
        double orthoCurrentMag = ((xCurrent * xOrthogonal) +
                (yCurrent * yOrthogonal));
        
        // cannot overcome strength of the currents to go in this
        // direction
        if (orthoCurrentMag >= propulsion) {
            System.out.println("currents too strong! " + orthoCurrentMag);
            System.out.println("xortho: " + xOrthogonal + ", yOrtho: " + yOrthogonal);;
            return -1;
        }
        
        // calculate the magnitude of propulsion taking into account
        // the opposing (orthogonal) currents
        double propulsionMag = Math.sqrt(Math.pow(propulsion, 2) -
                Math.pow(orthoCurrentMag, 2));
        
        // calculate the magnitude of the current that helps the AUV
        // in direction of travel
        double currentMag = ((xCurrent * xDirection) + 
                (yCurrent * yDirection));
        
        // the total velocity is the sum of the propulsion and helpful
        // currents
        double velocity = propulsionMag + currentMag;
        
        if (velocity <= 0) {
            return -1;
        }
        // multiply by 1000 to have dist in m
        double time = (dist*1000)/velocity;
        return time;    
    }
        
 
    
    /*
     * Method: distance
     * 
     * Input: 4 doubles, for latitude and longitude of the two coordinates.
     *  char for the unit of distance.
     * 
     * Output: double which is the distance between the two coordinates, in the
     *  units specified.
     * 
     * Details: Code for longitude latitude distance formulas taken from
     *  http://www.dzone.com/snippets/distance-calculation-using-3
     *  Use the char 'K' to get distance in kilometers, and 'N' for nautical
     *  miles.
     */
    public static double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
      double theta = lon1 - lon2;
      double dist = Math.sin(deg2rad(lat1)) * Math.sin(deg2rad(lat2)) + Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) * Math.cos(deg2rad(theta));
      dist = Math.acos(dist);
      dist = rad2deg(dist);
      dist = dist * 60 * 1.1515;
      if (unit == 'K') {
        dist = dist * 1.609344;
      } else if (unit == 'N') {
        dist = dist * 0.8684;
        }
      return (dist);
    }

    /*
     * deg2rad and rad2deg methods also provided from 
     *  http://www.dzone.com/snippets/distance-calculation-using-3
     *  and is used for the distance calculations.
     */
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /*::  This function converts decimal degrees to radians             :*/
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    private static double deg2rad(double deg) {
      return (deg * Math.PI / 180.0);
    }

    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    /*::  This function converts radians to decimal degrees             :*/
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    private static double rad2deg(double rad) {
      return (rad * 180.0 / Math.PI);
    }
    
}
