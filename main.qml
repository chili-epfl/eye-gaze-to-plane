import QtQuick 2.0
import QtQuick.Controls 1.4
import SynchLoop 1.0
Item {
    visible: true
    width: 980
    height: 660
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
            loadMultiMarkersConfigFile("paper_map","/home/arzu/arzu_papernew.data")
            loadMesh("/home/arzu/paper_mesh.obj","paper_map")
            run();
        }

        onSuggestionChanged: suggestion_field.text=suggestion
        onCurrent_frameChanged: {
            image_current_frame.source=""
            image_current_frame.source=current_image_path
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
