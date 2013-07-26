/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;

/**
 *
 * @author atthewco
 */
public class writeMissionPlan {
    
    public static void writeTextMission(String fileName,
            ArrayList<OceanCell> path) {
        Writer out = null;
        try {
            out = new BufferedWriter(new FileWriter(fileName));
            out.write("MISSION FILE VERSION V1.1");
            out.write('\n');
            out.write("2X ;WP Number; Latitude; Longitude; Distance(m); "
                    + "Heading; Depth(ft) (D:depth from surface, "
                    + "H: Height from bottom, U:undulate); P:Parking(mins) "
                    + "VC:Video Camera S:Speed(kn) SS:Sidescan MB:Multibeam");
            out.write('\n');
            out.write("FILES");
            out.write('\n');
            
            // SERVER INFORMATION. MUST CHANGE!
            out.write("SOME SERVER AND COORDINATES STUFF");
            out.write('\n');
            out.write("SOMETHING MORE ABOUT THE SERVER AND COORDINATES");
            // END OF WRITING SERVER STUFF
            
            // Mission Waypoints section
            out.write("MISSION NAME");
            out.write('\n');
            // Assumes fileName wil include .txt at the end
            out.write(fileName);
            out.write('\n');
            out.write("START");
            out.write('\n');
            int numWP = path.size();
            for (int i = 0; i < numWP; ++i) {
                OceanCell wp = path.get(i);
                double dist = 0.0;
                if (i != 0) {
                    dist = AUV.distance(wp.getLatValue(), wp.getLonValue(), 
                            path.get(i-1).getLatValue(),
                            path.get(i-1).getLonValue(), 'K');
                    dist *= 1000; // convert to meters
                }
                out.write((i+1) + "; " + wp.getLatValue() + "; "
                        + wp.getLonValue() + "; "
                        + dist + "; "
                        // MUST FIXED HEADING (ANGLE?)
                        + "0.0 FIX" + "; "
                        + " D" + wp.getDepthValue()
                        + " P0 VC1,0,0,1000,0,VC2,0,0,1000,0 S2; 0;-1");
                out.write('\n');                                    
            }
            out.write("END");
            out.write('\n');
            
            // Buoy data section
            out.write("// Buoy Data: Lat; Long; CycleTime; "
                    + "RingRadious; ShowRing; "
                    + "PingData1; PingData2; ...; PingData99;");
            out.write('\n');
            out.write("START BUOY DATA");
            // If there is buoy data, write in here.
            out.write('\n');
            out.write("END BUOY DATA");
            out.write('\n');
            
            // Workspace info section
            out.write("START WORKSPACE");
            out.write('\n');
            // Boundary coordinates
            out.write("MINX=" + Planner.LOWLAT);
            out.write('\n');
            out.write("MAXX=" + Planner.HIGHLAT);
            out.write('\n');
            out.write("MINY=" + Planner.LOWLON);
            out.write('\n');
            out.write("MAXY=" + Planner.HIGHLON);
            out.write('\n');
            out.write("CENTERX=" + 
                    (Planner.HIGHLAT + Planner.LOWLAT)/2);
            out.write('\n');
            out.write("CENTERY=" + 
                    (Planner.HIGHLON + Planner.LOWLON)/2);
            out.write('\n');
            out.write("REFWP=0; 0; 0;  0.0; 0.0;  D0 P0 VC1,0,0,"
                    + "1000,0,VC2,0,0,1000,0 S2; 0;-1");
            out.write('\n');
            out.write("REFBUOY=0; 0; 10; 50; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;"
                    + " 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; ");
            out.write('\n');
            out.write("END WORKSPACE");
            out.write('\n');
            
            // waypoint timing section
            out.write("WP TIMING");
            out.write('\n');
            double totalTime = 0;
            double totalDist = 0;
            for (int i = 0; i < numWP; ++i) {
                OceanCell wp = path.get(i);
                double dist = 0;
                if (i != 0) {
                    dist = AUV.distance(wp.getLatValue(), wp.getLonValue(), 
                            path.get(i-1).getLatValue(),
                            path.get(i-1).getLonValue(), 'K'); 
                    dist *= 1000;
                }
                totalDist += dist;
                totalTime = wp.timeArrived; // total time is timeArrived of
                                            // last waypoint
                out.write("WP" + (i+1) + ";Time=" + wp.timeArrived
                        + ";Dist=" + totalDist);
                out.write('\n');
            }
            SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss");
            String time = df.format(totalTime);
            out.write("Total Time = " + time 
                    + ";Total Distance = " + totalDist);
            out.write('\n');
            out.write("END WP TIMING");
            out.write('\n');
            
            // sonar layers section
            out.write("START SONAR LAYERS");
            out.write('\n');          
            out.write("END SONAR LAYERS");
            out.write('\n');                  
            
            // lawnmowers section
            out.write("START LAWNMOWERS");
            out.write('\n');          
            out.write("END LAWNMOWERS");
            out.write('\n');               
            
            // notes section
            out.write("START NOTES");
            out.write('\n');          
            out.write("END NOTES");
            out.write('\n');
            
            // vehicle section
            out.write("START VEHICLE");
            out.write('\n');
            out.write("UVC=WPRadius=5");
            out.write('\n');          
            out.write("UVC=ParkPercentRadius=100");
            out.write('\n');  
            out.write("UVC=DelayMissionStart=0");
            out.write('\n');          
            out.write("UVC=MaxPitchAngle=25");
            out.write('\n'); 
            // MIGHT WANT TO ENABLE SALINITY CORRECTION
            out.write("UVC=EnableSalinityCorrection=False");
            out.write('\n');          
            out.write("UVC=SalinityValue=0");
            out.write('\n');  
            out.write("DVL=EnableControlADCPUp=False");
            out.write('\n');          
            out.write("DVL=TurnOnADCPUp=False");
            out.write('\n');  
            out.write("DVL=Setup.adpsetup[1].cellSize(m)=0");
            out.write('\n');          
            out.write("DVL=Setup.adpsetup[1].cellCount=0");
            out.write('\n');  
            out.write("DVL=EnableControlADCPDown=False");
            out.write('\n');          
            out.write("DVL=TurnOnADCPDown=False");
            out.write('\n');  
            out.write("DVL=Setup.adpsetup[0].cellSize(m)=0");
            out.write('\n');          
            out.write("DVL=Setup.adpsetup[0].cellCount=0");
            out.write('\n');  
            out.write("DVL=DwnCellNumToDisplay=0");
            out.write('\n');          
            out.write("DVL=EnableIceTrack=False");
            out.write('\n');  
            out.write("DVL=setup.dvlsetup.outputSon42=False");
            out.write('\n');          
            out.write("MB=TiltAnglePort20=False");
            out.write('\n');  
            out.write("MB=TiltAngleStarboard20=False");
            out.write('\n');          
            out.write("MB=TiltAngleCentered=False");
            out.write('\n');  
            out.write("UVC=TowFltRules=True");
            out.write('\n');          
            out.write("UVC=TowFltDelTme=10");
            out.write('\n');  
            // THIS CHANGES! FIGURE OUT WHAT IT DOES TO TOWFLOAT
            out.write("UVC=TowFltMisTme=1.48");
            out.write('\n');          
            out.write("UVC=TowFltUndTme=0");
            out.write('\n');  
            out.write("UVC=TowFltWrkDep=4");
            out.write('\n');          
            out.write("UVC=TowFltMaxDep=0");
            out.write('\n');  
            out.write("UVC=TowFltPowLosTme=30");
            out.write('\n');          
            out.write("UVC=EnableAllSafetyRules=True");
            out.write('\n');  
            out.write("UVC=MINDTB=True;10;");
            out.write('\n');          
            out.write("UVC=AbrtDive=True;40;");
            out.write('\n');  
            out.write("UVC=AbrtDiveNoLock=False;100;");
            out.write('\n');          
            out.write("UVC=OverPitch=True;35;40;");
            out.write('\n');  
            out.write("UVC=RunsLnger=True;2;");
            out.write('\n');          
            out.write("UVC=LeakDet=True;20;");
            out.write('\n');  
            out.write("UVC=ForwProg=True;60;");
            out.write('\n');          
            out.write("UVC=MaxDepth=True;5;");
            out.write('\n');  
            out.write("UVC=UpwdProg=True;40;");
            out.write('\n');          
            out.write("UVC=MinPower=True;15;");
            out.write('\n');  
            out.write("UVC=IrdPower=False;10;");
            out.write('\n');          
            out.write("UVC=RevMot=True;4;5;");
            out.write('\n');  
            out.write("UVC=DeplTwFlt=True;");
            out.write('\n');          
            out.write("UVC=StrtSRP=True;");
            out.write('\n');  
            out.write("UVC=LnchTwFltSRP=True;");
            out.write('\n');          
            out.write("END VEHICLE");
            out.write('\n');
            
            out.flush();             
        } catch (IOException e) {
            System.out.println("Failed to create mission file!");
        } finally {
            try {
                out.close();
             } catch (Exception exp) {
             }  
        }
    }
    
public static void writeKMLMission(String fileName,
            OceanPath currentPath) {
     ArrayList<OceanCell> path = currentPath.path;
     Writer out = null;
     try {    
         out = new BufferedWriter(new FileWriter(fileName));
         out.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
         out.write('\n');
         out.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
         out.write('\n');
         out.write("  <Document>");
         out.write('\n');
         out.write("    <name>" + fileName + "</name>");
         out.write('\n');
         out.write("    <Style id=\"yellowLineGreenPoly\">");
         out.write('\n');
         out.write("      <LineStyle>");
         out.write('\n');
         
         out.write("        <color>50B4DC14</color>");
         out.write('\n');
         out.write("        <width>8</width>");
         out.write('\n');
         out.write("      </LineStyle>");
         out.write('\n');
         out.write('\n');
         out.write("    </Style>");
         out.write('\n');
         out.write("    <Placemark>");
         out.write('\n');
         out.write("      <name>Boundaries</name>");
         out.write('\n');
         out.write("      <description> LOWLON: " + (360-Planner.LOWLON) +
                 "\u00B0,\n LOWLAT: " + Planner.LOWLAT + "\u00B0,\n HIGHLON: " + 
                 (360-Planner.HIGHLON) + "\u00B0,\n HIGHLAT: " + Planner.HIGHLAT +
                 "\u00B0 </description>");
         out.write('\n');
         out.write("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
         out.write('\n');
         out.write("      <LineString>");
         out.write('\n');     
         out.write("        <coordinates> ");  
         out.write("-" + (360-Planner.LOWLON) + ","
                 + Planner.LOWLAT+ "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.write('\n');                                                             
         out.write("        </coordinates>");
         out.write('\n');
         out.write("      </LineString>");
         out.write('\n');
         out.write("    </Placemark>");
         out.write('\n');          
         
         
         out.write("    <Style id=\"yellowLineGreenPoly\">");
         out.write('\n');
         out.write("      <LineStyle>");
         out.write('\n');
         
         out.write("        <color>6414F000</color>");
         out.write('\n');
         out.write("        <width>8</width>");
         out.write('\n');
         out.write("      </LineStyle>");
         out.write('\n');
         out.write("      <PolyStyle>");
         out.write('\n');
         out.write("        <color>7f00ff00</color>");
         out.write('\n');
         out.write("      </PolyStyle>");
         out.write('\n');
         out.write("    </Style>");
         out.write('\n');
         out.write("    <Placemark>");
         out.write('\n');
         out.write("      <name>Mission Path</name>");
         out.write('\n');
         out.write("      <description>" + currentPath + "</description>");
         out.write('\n');
         out.write("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
         out.write('\n');
         out.write("      <LineString>");
         out.write('\n');               
         out.write("        <extrude>1</extrude>");
         out.write('\n');
         out.write("        <tessellate>1</tessellate>");
         out.write('\n');
         out.write("        <altitudeMode>absolute</altitudeMode>");
         out.write('\n');
         out.write("        <coordinates> ");  
         
         OceanCell wp = path.get(0);
         out.write("-" + (360-wp.getLonValue()) + ","
                 + wp.getLatValue()+ "," + wp.getDepthValue());
         out.write('\n');                                
         int numWP = path.size();
         for (int i = 1; i < numWP; ++i) {
             wp = path.get(i);
             out.write("          -" + (360-wp.getLonValue()) + ","
                     + wp.getLatValue() + "," + wp.getDepthValue());
             out.write('\n');                                
         }
         out.write("        </coordinates>");
         out.write('\n');
         out.write("      </LineString>");
         out.write('\n');
         out.write("    </Placemark>");
         out.write('\n');                     
         for (int i = 0; i < numWP; ++i) {
             wp = path.get(i);
             out.write("    <Placemark>");
             out.write('\n');
             out.write("      <name>" + (i+1) + "</name>");
             out.write('\n');
             out.write("      <description>" + wp.printData() + "</description>");
             out.write('\n');
             out.write("      <Point>");
             out.write('\n');               
             out.write("        <coordinates> "); 
             out.write("-" + (360-wp.getLonValue()) + ","
                     + wp.getLatValue() + "," + wp.getDepthValue());
             out.write('\n');
             out.write("        </coordinates>");
             out.write('\n');
             out.write("      </Point>");
             out.write('\n');
             out.write("    </Placemark>");
             out.write('\n');   
         }
         out.write("  </Document>");
         out.write('\n');
         out.write("</kml>");
         out.write('\n');  
         out.flush();
        } catch (IOException e) {
            System.out.println("Failed to create KML Path file!");
        } finally {
            try {
                out.close();
             } catch (Exception exp) {
             }  
        }
    }

public static void writeKMLGrid(String fileName, OceanGrid grid) {
    Writer out = null;
    try {
         out = new BufferedWriter(new FileWriter(fileName));  
         out.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
         out.write('\n');
         out.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
         out.write('\n');
         out.write("  <Document>");
         out.write('\n');
         out.write("    <name>" + fileName + "</name>");
         out.write('\n');
         out.write("    <Style id=\"yellowLineGreenPoly\">");
         out.write('\n');
         out.write("      <LineStyle>");
         out.write('\n');
         
         out.write("        <color>50B4DC14</color>");
         out.write('\n');
         out.write("        <width>8</width>");
         out.write('\n');
         out.write("      </LineStyle>");
         out.write('\n');
         out.write('\n');
         out.write("    </Style>");
         out.write('\n');
         out.write("    <Placemark>");
         out.write('\n');
         out.write("      <name>Boundaries</name>");
         out.write('\n');
         out.write("      <description> LOWLON: " + (360-Planner.LOWLON) +
                 "\u00B0,\n LOWLAT: " + Planner.LOWLAT + "\u00B0,\n HIGHLON: " + 
                 (360-Planner.HIGHLON) + "\u00B0,\n HIGHLAT: " + Planner.HIGHLAT +
                 "\u00B0 </description>");
         out.write('\n');
         out.write("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
         out.write('\n');
         out.write("      <LineString>");
         out.write('\n');     
         out.write("        <coordinates> ");  
         out.write("-" + (360-Planner.LOWLON) + ","
                 + Planner.LOWLAT+ "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.write('\n');                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.write('\n');                                                             
         out.write("        </coordinates>");
         out.write('\n');
         out.write("      </LineString>");
         out.write('\n');
         out.write("    </Placemark>");
         out.write('\n');          
         out.write('\n');
         for (int i = 0; i < grid.NLAT; ++i) {
                for (int j = 0; j < grid.NLON; ++j) {
                    OceanCell wp = grid.getCell(0, 0, i, j);
                    out.write("    <Placemark>");
                    out.write('\n');
                    out.write("      <name>" + wp.validCell + "</name>");
                    out.write('\n');
                    out.write("      <description>" + wp.printData() + "</description>");
                    out.write('\n');
                    out.write("      <Point>");
                    out.write('\n');               
                    out.write("        <coordinates> "); 
                    out.write("-" + (360-wp.getLonValue()) + ","
                            + wp.getLatValue() + "," + wp.getDepthValue());
                    out.write('\n');
                    out.write("        </coordinates>");
                    out.write('\n');
                    out.write("      </Point>");
                    out.write('\n');
                    out.write("    </Placemark>");
                    out.write('\n');          
                }
         }  
         out.write("  </Document>");
         out.write('\n');
         out.write("</kml>");
         out.write('\n');
         out.flush();      
        } catch (IOException e) {
            System.out.println("Failed to create KML Grid file!");
        }  finally {
            try {
                out.close();
             } catch (Exception exp) {
             }  
        }           
    }    
}
