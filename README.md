PathPlanner
===========
Author: Matt Cook

This project is used for AUV Path Planning using information provided by the Jet Propulsion Laboratory's Regional Ocean
Modeling System (ROMS). Multiple path planning algorithms are used in this project, and this README outlines how to run 
the program and use the output from this project. For more information about the individual classes, read the comments
within the files.

To run the program:
The only file that should be changed to run the program is the Planner.java, as that is where the global variables are
held that are changed to fit the parameters of the search desired. Most likely, the most important parameters to be 
changed are:
- The netCDF files to be read
- Start Location
- Boundaries of the path planner (to search on a smaller grid)
- max time length of the mission
- name of output files that record the path
The other parameters that can be changed are also explained in the Planner.java file.

The main method is in Planner.java, and by running the program will output two files to the local directory of
the project.
The first one of these files is a text file, which is the VectorMap mission file. VectorMap is the software
used for planning waypoints for the Iver2 AUV to travel. This text file can be loaded directed into the Iver2
for deployment.
The second file is a KML file. KML files are used to display in Google Earth or NASA World Wind. When opened in
Google Earth, this file displays the path planning boundaries, the path, and information about each waypoint along the path.




