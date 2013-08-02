/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import static PathPlanner.Planner.hourStartIndex;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/**
 *
 * @author atthewco
 */
public class PathTests {
        /*
     * Method: testPaths
     * 
     * Input: OceanCell for the start, OceanGrid, and a double for the max
     *  mission length
     * 
     * Output: an OceanPath which holds the cells of the best path found
     *  from the testing.
     */
    public static OceanPath testPaths(OceanCell start, OceanGrid grid, double missionLength) {
        OceanPath currentPath;
        int numTrials = 100;
        System.out.println("RANDOM PATH PLANNING");
        System.out.println("----------------------------------");
        System.out.println("Generating " + numTrials + " Random Paths");
        OceanPath bestPath = RandomPlanner.Random(start, grid, missionLength);
        // index to represent which algorithm found the best path, -1 is random.
        // positive numbers correspond to weighting for A*
        double bestPathIndex = -1;
        Boolean optimizedStart = false;
        for (int i = 0; i < numTrials; ++i) {
            currentPath = RandomPlanner.Random(start, grid, missionLength);
            if (currentPath.fScore > bestPath.fScore) {
                bestPath = currentPath;
            }
        }
        System.out.println("-----------------------------------");
        System.out.println("BEST RANDOM PATH WITH CHOSEN START");
        System.out.println(bestPath);
        System.out.println("BEST RANDOM PATH WITH OPTIMAL START");
        OceanCell optimalStart = Planner.findStart(grid);
        System.out.println("Optimal Start is " + optimalStart.toString());
        OceanPath tempBest = RandomPlanner.Random(optimalStart, grid, missionLength);
        for (int i = 0; i < numTrials; ++i) {
            currentPath = RandomPlanner.Random(optimalStart, grid, missionLength);
            if (currentPath.fScore > tempBest.fScore) {
                optimizedStart = true;
                tempBest = currentPath;
            }
        }
        System.out.println(tempBest);
        if (tempBest.fScore > bestPath.fScore) {
            bestPath = tempBest;
        }
        System.out.println("-----------------------------------");
        System.out.println("A* PATH PLANNING");
        System.out.println("-----------------------------------");
        Planner.weighting = 0;
        while (Planner.weighting <= 3) {
            currentPath = AStarPlanner.AStar(start, grid, missionLength);
            System.out.println("Weighting:" + Planner.weighting
                    + ", " + currentPath);
            if (currentPath.fScore > bestPath.fScore) {
                bestPath = currentPath;
                bestPathIndex = Planner.weighting;
                optimizedStart = false;
            }
            writeMissionPlan.writeKMLMission(("test" + Planner.weighting + ".kml"),
                    currentPath);
            Planner.weighting += 0.5;
        }
        System.out.println("A* WITH OPTIMAL START");
        Planner.weighting = 0;
        while (Planner.weighting <= 3) {
            currentPath = AStarPlanner.AStar(optimalStart, grid, missionLength);
            System.out.println("Weighting:" + Planner.weighting
                    + ", " + currentPath);
            if (currentPath.fScore > bestPath.fScore) {
                bestPath = currentPath;
                bestPathIndex = Planner.weighting;
                optimizedStart = true;
            }
            writeMissionPlan.writeKMLMission(("test" + Planner.weighting + "OptStart.kml"),
                    currentPath);
            Planner.weighting += 0.5;
        }
        System.out.println("------------------------------------");
        if (bestPathIndex == -1) {
            System.out.println("BEST PATH FOUND WITH RANDOM:");
        }
        else {
            System.out.println("BEST PATH FOUND WITH WEIGHTING " + bestPathIndex 
                    + ": ");
        }
        if (optimizedStart) {
            System.out.println("FOUND BEST PATH WITH OPTIMIZED START");
        }
        else {
            System.out.println("FOUND BEST PATH WITH CHOSEN START");
        }
        System.out.println(bestPath);
        return bestPath;
    }
    
        /*
     * Method: testPaths2
     * 
     * Input: OceanCell for the start, OceanGrid, and a double for the max
     *  mission length
     * 
     * Output: an OceanPath which holds the cells of the best path found
     *  from the testing alternative 2.
     */
    public static OceanPath testPaths2(OceanCell start, OceanGrid grid, double missionLength) {
        try {
            BufferedWriter out = 
                    new BufferedWriter(new FileWriter(Planner.testFile));
            OceanPath currentPath;
            OceanPath bestPath = null;
            OceanCell bestStart = null;
            ArrayList<Double> randomScores = new ArrayList<Double>();
            ArrayList<Double> greedyScores = new ArrayList<Double>();
            ArrayList<Double> scores = new ArrayList<Double>();
            int randomCount = 0;
            int greedyCount = 0;
            int count = 0;
            double randomGscore = 0;
            double greedyGscore = 0;
            double gScore = 0;
            int invalidCount = 0;
            // index to represent which algorithm found the best path, -1 is random.
            // positive numbers correspond to Planner.weighting for A*
            double bestPathIndex = -1;         
            for (int x = 0; x < grid.NLAT; x=x+4) {
                for (int y = 0; y < grid.NLON; y=y+4) {
            //for (int x = 1; x < 2; ++x) {
            //    for (int y = 1; y < 11; ++y) {;
                    OceanCell startCell = grid.getCell(Planner.hourStartIndex, 0, x, y);
                    if (!startCell.validCell) {
                        invalidCount++;
                        System.out.println("invalid start #:" + invalidCount);
                        continue;
                    }
                    int numTrials = 100;
                    System.out.println("START CELL: " + startCell);
                    out.write("START CELL: " + startCell);
                    out.newLine();
                    //System.out.println("START PATH PLANNING");
                    //System.out.println("----------------------------------");
                    //System.out.println("Generating " + numTrials + " Random Paths");
                    currentPath = RandomPlanner.Random(startCell, grid, missionLength);
                    randomCount++;
                    randomGscore += currentPath.gScore;
                    randomScores.add(currentPath.gScore);
                    if (bestPath == null) {
                        bestPath = currentPath;
                    }
                    if (currentPath.gScore > bestPath.gScore) {
                        bestPathIndex = -1;
                        bestPath = currentPath;
                        bestStart = startCell;
                    }
                    
                    System.out.println("Random: " + currentPath);
                    out.write("Random: " + currentPath);
                    out.newLine();
                    Planner.weighting = 0;
                    while (Planner.weighting <= 1) {
                        currentPath = AStarPlanner.AStar(startCell, grid, missionLength); 
                        if (currentPath == null) {
                            break;
                        }
                        //writeMissionPlan.writeKMLMission("test" + x + "-" + y + ".kml", currentPath);                        
                        System.out.println("Weighting:" + Planner.weighting 
                                + ", " + currentPath);
                        out.write("Weighting:" + Planner.weighting
                                + ", " + currentPath);
                        out.newLine();
                        if (currentPath.fScore > bestPath.fScore) {
                            bestPath = currentPath;
                            bestPathIndex = Planner.weighting;
                            bestStart = startCell;
                        }
                        if (Planner.weighting == 0) {
                             greedyCount++;
                             greedyGscore += currentPath.gScore;
                             greedyScores.add(currentPath.gScore);
                        }
                        if (Planner.weighting > 0) {
                            count++;
                            gScore += currentPath.gScore;
                            scores.add(currentPath.gScore);
                        }
                        Planner.weighting += 1;
                    }
                    // in order to increment by 4 correctly
                    if (y == 0) {
                        --y;
                    }
                }
                // in order to increment by 4 correctly
                if (x == 0) {
                   --x;
                }
            }

            System.out.println("------------------------------------");
            out.write("------------------------------------");
            out.newLine();
            if (bestPathIndex == -1) {
                System.out.println("BEST PATH FOUND WITH RANDOM:");
                out.write("BEST PATH FOUND WITH RANDOM:");
                out.newLine();
            }
            else {
                System.out.println("BEST PATH FOUND WITH WEIGHTING " + bestPathIndex 
                        + ": ");
                out.write("BEST PATH FOUND WITH WEIGHTING " + bestPathIndex 
                        + ": ");
                out.newLine();
            }
            System.out.println(bestPath);
            out.write(bestPath.toString());
            out.newLine();
            System.out.println("WITH BEST START: " + bestStart);
            out.write("WITH BEST START: " + bestStart);
            out.newLine();
            System.out.println("----------------------------------------");
            System.out.println("AVERAGE STATISTICS");
            System.out.println("-----------------------------------------");
            double randomMean = (randomGscore/randomCount);
            double greedyMean = (greedyGscore/greedyCount);
            double mean = (gScore/count);        
            System.out.println("Average Score of Random in " + randomCount + " trials: " + randomMean);
            System.out.println("Average Score of Greedy in " + greedyCount + " trials: " + greedyMean );
            System.out.println("Average Score of A* in " + count + " trials: " + mean);   
            out.write("----------------------------------------");
            out.newLine();
            out.write("AVERAGE STATISTICS");
            out.newLine();
            out.write("-----------------------------------------");
            out.newLine();
            out.write("Average Score of Random in " + randomCount + " trials: " + randomMean);
            out.newLine();
            out.write("Average Score of Greedy in " + greedyCount + " trials: " + greedyMean);
            out.newLine();
            out.write("Average Score of A* in " + count + " trials: " + mean);
            out.newLine();
            double randomVar = 0;
            double greedyVar = 0;
            double var = 0;
            out.write("random scores = {");
            while (!randomScores.isEmpty()) {
                double random = randomScores.remove(randomScores.size()-1);
                if (!randomScores.isEmpty()) {
                    out.write(random + ", ");
                }
                else {
                    out.write(random + "}");
                }
                randomVar += Math.pow((random - randomMean), 2);
            }
            out.newLine();
            out.write("greedy scores = {");
            while (!greedyScores.isEmpty()) {
                double greedy = greedyScores.remove(greedyScores.size()-1);
                if (!greedyScores.isEmpty()) {
                    out.write(greedy + ", ");
                }
                else {
                    out.write(greedy + "}");
                }
                greedyVar += Math.pow((greedy - greedyMean), 2);
            }
            out.newLine();
            out.write("astar scores = {");            
            while (!scores.isEmpty() ) {
                double astar = scores.remove(scores.size()-1);
                if (!scores.isEmpty()) {
                    out.write(astar + ", ");
                }
                else {
                    out.write(astar + "}");
                }
                var += Math.pow((astar - mean), 2);
            }
            out.newLine();
            randomVar = randomVar / (randomCount-1);
            greedyVar = greedyVar / (greedyCount-1);
            var = var / (count-1);
            out.write("Variance in Random: " + randomVar);
            out.newLine();
            out.write("Variance in Greedy: " + greedyVar);
            out.newLine();
            out.write("Variance in A*: " + var);
            out.newLine();
            // to look at without opening output file
            System.out.println("Variance in Random: " + randomVar);
            System.out.println("Variance in Greedy: " + greedyVar);
            System.out.println("Variance in A*: " + var);
            
            out.write("------------------------------------------------");
            out.newLine();
            out.write("Grid Boundaries: ");
            out.newLine();
            out.write("LOWLON: " + Planner.LOWLON + ", HIGHLON: " + Planner.HIGHLON);
            out.newLine();
            out.write("LOWLAT: " + Planner.LOWLAT + ", HIGHLAT: " + Planner.HIGHLAT);
            out.newLine();
            out.write("Source: " + Planner.fileName + ", " + Planner.errFileName);
            out.newLine();
            out.write("Max Mission Length: " + Planner.missionLength + " secs");
            out.newLine();
            out.write("AUV Propulsion: " + Planner.propulsion + " m/s");
            out.newLine();
            out.write("Grid Dimensions: " + grid.NTIME + " time, " + 
                    grid.NDEPTH + " depth, " + grid.NLAT +
                    " latitude, " + grid.NLON + " longitude.");
            out.newLine();
            out.close();
            writeMissionPlan.writeKMLMission((Planner.testFile + ".kml"),
                  bestPath); 
            return bestPath;
                                 
        } catch (IOException e) {
            System.out.println("Failed to create test file!");
            return null;
        }
    }
    
}
