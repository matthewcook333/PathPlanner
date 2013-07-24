/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package PathPlanner;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;

/**
 *
 * @author atthewco
 */
public class writeMissionPlan {
    
    public static void writeTextMission(String fileName,
            ArrayList<OceanCell> path) {
        try {
            BufferedWriter out = 
                    new BufferedWriter(new FileWriter(fileName));
            out.write("MISSION FILE VERSION V1.1");
            out.newLine();
            out.write("2X ;WP Number; Latitude; Longitude; Distance(m); "
                    + "Heading; Depth(ft) (D:depth from surface, "
                    + "H: Height from bottom, U:undulate); P:Parking(mins) "
                    + "VC:Video Camera S:Speed(kn) SS:Sidescan MB:Multibeam");
            out.newLine();
            out.write("FILES");
            out.newLine();
            
            // SERVER INFORMATION. MUST CHANGE!
            out.write("SOME SERVER AND COORDINATES STUFF");
            out.newLine();
            out.write("SOMETHING MORE ABOUT THE SERVER AND COORDINATES");
            // END OF WRITING SERVER STUFF
            
            // Mission Waypoints section
            out.write("MISSION NAME");
            out.newLine();
            // Assumes fileName wil include .txt at the end
            out.write(fileName);
            out.newLine();
            out.write("START");
            out.newLine();
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
                out.newLine();                                    
            }
            out.write("END");
            out.newLine();
            
            // Buoy data section
            out.write("// Buoy Data: Lat; Long; CycleTime; "
                    + "RingRadious; ShowRing; "
                    + "PingData1; PingData2; ...; PingData99;");
            out.newLine();
            out.write("START BUOY DATA");
            // If there is buoy data, write in here.
            out.newLine();
            out.write("END BUOY DATA");
            out.newLine();
            
            // Workspace info section
            out.write("START WORKSPACE");
            out.newLine();
            // Boundary coordinates
            out.write("MINX=" + Planner.LOWLAT);
            out.newLine();
            out.write("MAXX=" + Planner.HIGHLAT);
            out.newLine();
            out.write("MINY=" + Planner.LOWLON);
            out.newLine();
            out.write("MAXY=" + Planner.HIGHLON);
            out.newLine();
            out.write("CENTERX=" + 
                    (Planner.HIGHLAT + Planner.LOWLAT)/2);
            out.newLine();
            out.write("CENTERY=" + 
                    (Planner.HIGHLON + Planner.LOWLON)/2);
            out.newLine();
            out.write("REFWP=0; 0; 0;  0.0; 0.0;  D0 P0 VC1,0,0,"
                    + "1000,0,VC2,0,0,1000,0 S2; 0;-1");
            out.newLine();
            out.write("REFBUOY=0; 0; 10; 50; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;"
                    + " 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; "
                    + "0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; ");
            out.newLine();
            out.write("END WORKSPACE");
            out.newLine();
            
            // waypoint timing section
            out.write("WP TIMING");
            out.newLine();
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
                out.newLine();
            }
            SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss");
            String time = df.format(totalTime);
            out.write("Total Time = " + time 
                    + ";Total Distance = " + totalDist);
            out.newLine();
            out.write("END WP TIMING");
            out.newLine();
            
            // sonar layers section
            out.write("START SONAR LAYERS");
            out.newLine();          
            out.write("END SONAR LAYERS");
            out.newLine();                  
            
            // lawnmowers section
            out.write("START LAWNMOWERS");
            out.newLine();          
            out.write("END LAWNMOWERS");
            out.newLine();               
            
            // notes section
            out.write("START NOTES");
            out.newLine();          
            out.write("END NOTES");
            out.newLine();
            
            // vehicle section
            out.write("START VEHICLE");
            out.newLine();
            out.write("UVC=WPRadius=5");
            out.newLine();          
            out.write("UVC=ParkPercentRadius=100");
            out.newLine();  
            out.write("UVC=DelayMissionStart=0");
            out.newLine();          
            out.write("UVC=MaxPitchAngle=25");
            out.newLine(); 
            // MIGHT WANT TO ENABLE SALINITY CORRECTION
            out.write("UVC=EnableSalinityCorrection=False");
            out.newLine();          
            out.write("UVC=SalinityValue=0");
            out.newLine();  
            out.write("DVL=EnableControlADCPUp=False");
            out.newLine();          
            out.write("DVL=TurnOnADCPUp=False");
            out.newLine();  
            out.write("DVL=Setup.adpsetup[1].cellSize(m)=0");
            out.newLine();          
            out.write("DVL=Setup.adpsetup[1].cellCount=0");
            out.newLine();  
            out.write("DVL=EnableControlADCPDown=False");
            out.newLine();          
            out.write("DVL=TurnOnADCPDown=False");
            out.newLine();  
            out.write("DVL=Setup.adpsetup[0].cellSize(m)=0");
            out.newLine();          
            out.write("DVL=Setup.adpsetup[0].cellCount=0");
            out.newLine();  
            out.write("DVL=DwnCellNumToDisplay=0");
            out.newLine();          
            out.write("DVL=EnableIceTrack=False");
            out.newLine();  
            out.write("DVL=setup.dvlsetup.outputSon42=False");
            out.newLine();          
            out.write("MB=TiltAnglePort20=False");
            out.newLine();  
            out.write("MB=TiltAngleStarboard20=False");
            out.newLine();          
            out.write("MB=TiltAngleCentered=False");
            out.newLine();  
            out.write("UVC=TowFltRules=True");
            out.newLine();          
            out.write("UVC=TowFltDelTme=10");
            out.newLine();  
            // THIS CHANGES! FIGURE OUT WHAT IT DOES TO TOWFLOAT
            out.write("UVC=TowFltMisTme=1.48");
            out.newLine();          
            out.write("UVC=TowFltUndTme=0");
            out.newLine();  
            out.write("UVC=TowFltWrkDep=4");
            out.newLine();          
            out.write("UVC=TowFltMaxDep=0");
            out.newLine();  
            out.write("UVC=TowFltPowLosTme=30");
            out.newLine();          
            out.write("UVC=EnableAllSafetyRules=True");
            out.newLine();  
            out.write("UVC=MINDTB=True;10;");
            out.newLine();          
            out.write("UVC=AbrtDive=True;40;");
            out.newLine();  
            out.write("UVC=AbrtDiveNoLock=False;100;");
            out.newLine();          
            out.write("UVC=OverPitch=True;35;40;");
            out.newLine();  
            out.write("UVC=RunsLnger=True;2;");
            out.newLine();          
            out.write("UVC=LeakDet=True;20;");
            out.newLine();  
            out.write("UVC=ForwProg=True;60;");
            out.newLine();          
            out.write("UVC=MaxDepth=True;5;");
            out.newLine();  
            out.write("UVC=UpwdProg=True;40;");
            out.newLine();          
            out.write("UVC=MinPower=True;15;");
            out.newLine();  
            out.write("UVC=IrdPower=False;10;");
            out.newLine();          
            out.write("UVC=RevMot=True;4;5;");
            out.newLine();  
            out.write("UVC=DeplTwFlt=True;");
            out.newLine();          
            out.write("UVC=StrtSRP=True;");
            out.newLine();  
            out.write("UVC=LnchTwFltSRP=True;");
            out.newLine();          
            out.write("END VEHICLE");
            out.newLine();
            
            out.close();             
        } catch (IOException e) {
            System.out.println("Failed to create mission file!");
        }
    }
    
    public static void writeKMLMission(String fileName,
            ArrayList<OceanCell> path) {
 try {
     try (BufferedWriter out = new BufferedWriter(new FileWriter(fileName))) {
         out.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
         out.newLine();
         out.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
         out.newLine();
         out.write("  <Document>");
         out.newLine();
         out.write("    <name>" + fileName + "</name>");
         out.newLine();
         out.write("    <Style id=\"yellowLineGreenPoly\">");
         out.newLine();
         out.write("      <LineStyle>");
         out.newLine();
         
         out.write("        <color>50B4DC14</color>");
         out.newLine();
         out.write("        <width>8</width>");
         out.newLine();
         out.write("      </LineStyle>");
         out.newLine();
         out.newLine();
         out.write("    </Style>");
         out.newLine();
         out.write("    <Placemark>");
         out.newLine();
         out.write("      <name>Boundaries</name>");
         out.newLine();
         out.write("      <description> LOWLON: " + (360-Planner.LOWLON) +
                 "\u00B0,\n LOWLAT: " + Planner.LOWLAT + "\u00B0,\n HIGHLON: " + 
                 (360-Planner.HIGHLON) + "\u00B0,\n HIGHLAT: " + Planner.HIGHLAT +
                 "\u00B0 </description>");
         out.newLine();
         out.write("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
         out.newLine();
         out.write("      <LineString>");
         out.newLine();     
         out.write("        <coordinates> ");  
         out.write("-" + (360-Planner.LOWLON) + ","
                 + Planner.LOWLAT+ "," + 0);
         out.newLine();                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.newLine();                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.HIGHLAT + "," + 0);
         out.newLine();                                
         out.write("          -" + (360-Planner.HIGHLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.newLine();                                
         out.write("          -" + (360-Planner.LOWLON) + ","
                + Planner.LOWLAT + "," + 0);
         out.newLine();                                                             
         out.write("        </coordinates>");
         out.newLine();
         out.write("      </LineString>");
         out.newLine();
         out.write("    </Placemark>");
         out.newLine();          
         
         
         out.write("    <Style id=\"yellowLineGreenPoly\">");
         out.newLine();
         out.write("      <LineStyle>");
         out.newLine();
         
         out.write("        <color>6414F000</color>");
         out.newLine();
         out.write("        <width>8</width>");
         out.newLine();
         out.write("      </LineStyle>");
         out.newLine();
         out.write("      <PolyStyle>");
         out.newLine();
         out.write("        <color>7f00ff00</color>");
         out.newLine();
         out.write("      </PolyStyle>");
         out.newLine();
         out.write("    </Style>");
         out.newLine();
         out.write("    <Placemark>");
         out.newLine();
         out.write("      <name>Mission Path</name>");
         out.newLine();
         out.write("      <description>AUV Path</description>");
         out.newLine();
         out.write("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
         out.newLine();
         out.write("      <LineString>");
         out.newLine();               
         out.write("        <extrude>1</extrude>");
         out.newLine();
         out.write("        <tessellate>1</tessellate>");
         out.newLine();
         out.write("        <altitudeMode>absolute</altitudeMode>");
         out.newLine();
         out.write("        <coordinates> ");  
         
         OceanCell wp = path.get(0);
         out.write("-" + (360-wp.getLonValue()) + ","
                 + wp.getLatValue()+ "," + wp.getDepthValue());
         out.newLine();                                
         int numWP = path.size();
         for (int i = 1; i < numWP; ++i) {
             wp = path.get(i);
             out.write("          -" + (360-wp.getLonValue()) + ","
                     + wp.getLatValue() + "," + wp.getDepthValue());
             out.newLine();                                
         }
         out.write("        </coordinates>");
         out.newLine();
         out.write("      </LineString>");
         out.newLine();
         out.write("    </Placemark>");
         out.newLine();  
            
         for (int i = 0; i < Planner.griddy.NLAT; ++i) {
                for (int j = 0; j < Planner.griddy.NLON; ++j) {
                    wp = Planner.griddy.getCell(0, 0, i, j);
                    out.write("    <Placemark>");
                    out.newLine();
                    out.write("      <name>" + "</name>");
                    out.newLine();
                    out.write("      <description>" + wp + "</description>");
                    out.newLine();
                    out.write("      <Point>");
                    out.newLine();               
                    out.write("        <coordinates> "); 
                    out.write("-" + (360-wp.getLonValue()) + ","
                            + wp.getLatValue() + "," + wp.getDepthValue());
                    out.newLine();
                    out.write("        </coordinates>");
                    out.newLine();
                    out.write("      </Point>");
                    out.newLine();
                    out.write("    </Placemark>");
                    out.newLine();          
                }
         }
         /*                             
         for (int i = 0; i < numWP; ++i) {
             wp = path.get(i);
             out.write("    <Placemark>");
             out.newLine();
             out.write("      <name>Waypoint " + (i+1) + "</name>");
             out.newLine();
             out.write("      <description>" + wp + "</description>");
             out.newLine();
             out.write("      <Point>");
             out.newLine();               
             out.write("        <coordinates> "); 
             out.write("-" + (360-wp.getLonValue()) + ","
                     + wp.getLatValue() + "," + wp.getDepthValue());
             out.newLine();
             out.write("        </coordinates>");
             out.newLine();
             out.write("      </Point>");
             out.newLine();
             out.write("    </Placemark>");
             out.newLine();   
         }
         */
         
         out.write("  </Document>");
         out.newLine();
         out.write("</kml>");
         out.newLine();
     }             
        } catch (IOException e) {
            System.out.println("Failed to create KML Path file!");
        }             
    }   
}
