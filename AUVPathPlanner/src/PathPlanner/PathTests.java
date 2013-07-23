/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import static PathPlanner.Planner.hourStartIndex;

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
                    currentPath.path);
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
                    currentPath.path);
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
        OceanPath currentPath;
        OceanPath bestPath = null;
        OceanCell bestStart = null;
        int randomCount = 0;
        int greedyCount = 0;
        int count = 0;
        double randomGscore = 0;
        double greedyGscore = 0;
        double gScore = 0;
        // index to represent which algorithm found the best path, -1 is random.
        // positive numbers correspond to Planner.weighting for A*
        double bestPathIndex = -1;         
        for (int x = 0; x < grid.NLAT; ++x) {
            for (int y = 0 ; y < grid.NLON; ++y) {
                OceanCell startCell = grid.getCell(Planner.hourStartIndex, 0, x, y);
                int numTrials = 100;
                System.out.println("START CELL: " + startCell);
                System.out.println("START PATH PLANNING");
                System.out.println("----------------------------------");
                System.out.println("Generating " + numTrials + " Random Paths");
                bestPath = RandomPlanner.Random(startCell, grid, missionLength);
                randomCount++;
                randomGscore += bestPath.gScore;
                bestStart = startCell;
                for (int i = 0; i < numTrials; ++i) {
                    currentPath = RandomPlanner.Random(startCell, grid, missionLength);
                    randomCount++;
                    randomGscore += currentPath.gScore;
                    if (currentPath.fScore > bestPath.fScore) {
                        bestPath = currentPath;
                        bestStart = startCell;
                    }
                }
                System.out.println("Best Random: " + bestPath);
                Planner.weighting = 0;
                while (Planner.weighting <= 1) {
                    currentPath = AStarPlanner.AStar(startCell, grid, missionLength);
                    System.out.println("Weighting:" + Planner.weighting
                            + ", " + currentPath);
                    if (currentPath.fScore > bestPath.fScore) {
                        bestPath = currentPath;
                        bestPathIndex = Planner.weighting;
                        bestStart = startCell;
                    }
                    if (Planner.weighting == 0) {
                         greedyCount++;
                         greedyGscore += currentPath.gScore;
                    }
                    if (Planner.weighting > 0) {
                        count++;
                        gScore += currentPath.gScore;
                    }
                    writeMissionPlan.writeKMLMission(("test" + Planner.weighting + ".kml"),
                            currentPath.path);
                    Planner.weighting += 1;
                }
            }
        }
       
        System.out.println("------------------------------------");
        if (bestPathIndex == -1) {
            System.out.println("BEST PATH FOUND WITH RANDOM:");
        }
        else {
            System.out.println("BEST PATH FOUND WITH WEIGHTING " + bestPathIndex 
                    + ": ");
        }
        System.out.println(bestPath);
        System.out.println("WITH BEST START: " + bestStart);
        System.out.println("----------------------------------------");
        System.out.println("AVERAGE STATISTICS");
        System.out.println("-----------------------------------------");
        System.out.println("Average Score of Random in " + randomCount + " trials: " + (randomGscore/randomCount));
        System.out.println("Average Score of Greedy in " + greedyCount + " trials: " + (greedyGscore/greedyCount));
        System.out.println("Average Score of A* in " + count + " trials: " + (gScore/count));
        return bestPath;
    }
    
}
