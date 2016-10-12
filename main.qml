import QtQuick 2.0
import QtQuick.Controls 1.4
import SynchLoop 1.0
Item {
    visible: true
    width: 1280
    height: 960
    signal quit()
    focus: true

    Connections{
       target: synchLoop
       onDone:quit()
    }

    SynchLoop{
        id:synchLoop
        objectName: "synchLoop"

        function start(){
            setWrite_subtititles(enable_subs)//Always before load video
            setInteractive(interactive)
            setFileVideoFileName(video_file,skip_intervals)
            loadGazeData(gaze_data_file)
            loadMultiMarkersConfigFile("default","/home/chili/board_configuration.data")
            loadMultiMarkersConfigFile("tablet","/home/chili/QTProjects/EyeTrackingExtractAOIHits/tablet_tags.data")
            loadMesh("/home/chili/QTProjects/EyeTrackingExtractAOIHits/tablet_mesh.obj","tablet")
            run();
        }

        onSuggestionChanged: suggestion_field.text=suggestion
        onCurrent_frameChanged: {
            image_current_frame.source=""
            image_current_frame.source="file:///home/chili/QTProjects/build-EyeTrackingExtractAOIHits-Desktop_Qt_5_6_0_GCC_64bit-Debug/current_frame.png"
        }
    }

    Image {
        id:image_current_frame
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit

    }


    Row{
        anchors.top: parent.top
        anchors.margins: 10
        Text{
            text:"Processing video:"+video_file
        }
        Text{
            text:"Gaze Data:"+gaze_data_file
        }
    }

    Timer{
        interval: 5000
        running: true
        repeat: false
        onTriggered: synchLoop.start()
    }

    Row{
        anchors.bottom: parent.bottom
        anchors.margins: 20
        spacing: 10
        TextField{
            id:suggestion_field
            focus: true
            width: 200
            onAccepted: {
                aoi_field.text=suggestion_field.text
            }
        }
        Rectangle{
            width: 200
            height: suggestion_field.height
            Text{
                id:aoi_field
                anchors.fill: parent
                MouseArea{
                    anchors.fill: parent
                    onClicked: {
                        synchLoop.set_User_Value(aoi_field.text);
                    }
                }
            }
        }

    }


}
