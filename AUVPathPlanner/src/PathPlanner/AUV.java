/*
 * 
 */
package PathPlanner;


/**
 *
 * @author atthewco
 */
public class AUV {
    
    public static double travelTime(OceanCell currentCell, OceanCell neighbor) {
        
        double latStart = currentCell.getLatValue();
        double lonStart = currentCell.getLonValue();
        double latEnd = neighbor.getLatValue();
        double lonEnd = neighbor.getLonValue();
        double xCurrent = currentCell.getU();
        double yCurrent = currentCell.getV();
        double propulsion = Planner.propulsion;
        
        double dist = distance(latStart, lonStart, latEnd, lonEnd, 'K');
        double xDist = distance(latStart, lonStart, latEnd, lonStart, 'K');
        double yDist = distance(latStart, lonStart, latStart, lonEnd, 'K');
        
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
                (yCurrent + yOrthogonal));
        
        // cannot overcome strength of the currents to go in this
        // direction
        if (orthoCurrentMag >= propulsion) {
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
        
        
        
        //System.out.println("Start: " + latStart + ", " + lonStart + 
        //        " End: " + latEnd + ", " + lonEnd + " dist: " + dist + " time: " + dist/(velocity*1000));
        
        
        
        // multiply by 1000 to have dist in m
        double time = (dist*1000)/velocity;
        // print time for debugging
        //System.out.println(time);
        return time;    
    }
        
 
    
    /*
     * Code for longitude latitude distance formulas taken from
     * http://www.dzone.com/snippets/distance-calculation-using-3
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
