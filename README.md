Tool to map eye tracking raw data to a reference map including multiple ARTags.
It gets ".avi"" eye tracking video file and eye tracking raw data in ".txt". 
Then calculates x and y positions of eye gaze in a reference map. 


input format of eye gaze raw data: 

"Time of Day ","Participant","Point of Regard Binocular X [px]","Point of Regard Binocular Y [px]","Video Time"
"15:49:54:566","john",931.7,933.5,"00:00:00:083"


As the output it creates a ".txt"" file including the input eyegaze raw data and calculated x,y positions on the reference map and frame number of each instance. 

output format:

"Time of Day [hh:mm:ss:ms]","Participant","BinocularX[px]","BinocularY[px]","VideoTime[hh:mm:ss:ms]",0value,xpos,ypos,framecount
"15:34:09:485","ali",927.4,846.1,"00:01:10:080",0,354.632,-153.861,0:1:10:0

In order to detect the reference map position in the video it uses ARTag positions which are printed to the reference map. ArTag positions, rotations and numbers should be added to a configuration file and should be set in main.qml 

    loadMultiMarkersConfigFile("paper_map","/home/arzu/arzu_papernew.data")

In the arzu_papernew.data we used 12 different 4x4 ARTags (type of AR_MATRIX_CODE_4x4_BCH_13_9_3). 

Map object should also be set in main.qml

    loadMesh("/home/arzu/paper_mesh.obj","paper_map")

In paper_mesh.obj we have 300 mm to 500 mm plane area.


Note:

The project is a modified version of # EyeTracking-3DScene2AOIHits: 

A tool for converting fixations into AOI hits defined in a 3D Scene.

Requires

    -OpenCV 2.4
    -ARToolkit5
