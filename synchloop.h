#ifndef SYNCHLOOP_H
#define SYNCHLOOP_H

#include <QObject>
#include <AR/ar.h>
#include <opencv.hpp>
#include <QFile>
#include <QTextStream>
#include <QQuickImageProvider>
extern "C"{
#include <AR/ar.h>
#include <AR/config.h>
#include <AR/arFilterTransMat.h>
#include <AR/arMulti.h>
#include <AR/video.h>

}
#include <QTime>
#include <QMatrix4x4>
#include <Qt3DCore/Qt3DCore>

struct AR3DMultiPatternObject {
    ARMultiMarkerInfoT* marker_info;
    /*In Opengl coordinate system*/
    QMatrix4x4 pose;
    /*....*/
    ARFilterTransMatInfo* ftmi;
    bool visible;
    inline AR3DMultiPatternObject& operator=(const AR3DMultiPatternObject& o) {
        marker_info=o.marker_info;
        pose=o.pose;
        visible=o.visible;
        ftmi=o.ftmi;
        return *this;
    }
};


class SynchLoop : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString suggestion READ suggestion NOTIFY suggestionChanged)
public:
    explicit SynchLoop(QObject *parent = 0);

    /*Load a mesh with format .obj. The mesh objects are defined as obj objects (no groups) and triagular faces.
     *transform_name is the key to link the pose of the mesh to the pose of MultiMarker configuration*/
    Q_INVOKABLE void loadMesh(QString file_path, QString transform_name);

    /*Load a MultiMarker configuration indentified by the key "transform_name". The format of the file is
     *the native one from Artoolkit (http://artoolkit.org/documentation/doku.php?id=3_Marker_Training:marker_multi) */
    Q_INVOKABLE void loadMultiMarkersConfigFile(QString transform_name, QString file_path);

    /*Enable the generation of subtitles for the video. The subtitles contain the AOIs*/
    Q_INVOKABLE void setWrite_subtititles(bool write_subtitles){
        this->write_subtitles=write_subtitles;
    }

    /*Load Gaze data as csv with comma as separetor. The format is Time_of_day,Event_Category,P_x,P_y,Video_Time_ms*/
    Q_INVOKABLE void loadGazeData(QString file_name);

    Q_INVOKABLE void run();

    /*Open a video file. Skip_intervals is a QString (as QVariant) defining ranges of the video to be skipped.
     * Each interval is separeted by a semi-colon and it has format hh:mm:ss-hh:mm:ss*/
    Q_INVOKABLE void setFileVideoFileName(QString video_file_name, QString skip_intervals=QString());

    /*Set modality: interactive or non-interactive.
     * In interactive mode the detected AOI is suggested to the user through the UI, and the detection loop waits until the
     * user confirms. */
    Q_INVOKABLE void setInteractive(bool interactive){ this->interactive=interactive;}
    Q_INVOKABLE void set_User_Value(QString user_value){ this->user_value=user_value;wait_condition.wakeAll();}
    QString suggestion(){return m_suggestion;}

signals:
    void current_frameChanged();
    void suggestionChanged();
    void done();
public slots:

private:

    bool interactive=true;

    bool write_subtitles;

    QFile subtitle_file;
    QTextStream subtitles_stream;

    int subtitles_counter;

    int fps=24;

    QSize cameraSize;

    QString video_file_name,gaze_data_file_name;

    cv::VideoCapture video_capture;
    QList<int> skip_frames;

    int frame_counter;
    int tot_frame;

    ARParamLT* ar_param;
    ARHandle* ar_handle;
    AR3DHandle* ar_3d_handle;
    ARPattHandle* ar_patt_handle;
    AR_MATRIX_CODE_TYPE matrix_code_type=AR_MATRIX_CODE_3x3;


    QMatrix4x4 projectionMatrix;
    QVector4D distortionCoeff;

    Qt3DCore::QCamera camera;

    QHash<QString,AR3DMultiPatternObject*> ar_multimarker_objects;

    QHash<QString,QString> mesh_name2pose_name;
    QHash<QString, QVector<QVector3D> > mesh_vertices_objects;
    QHash<QString, QVector< QVector <int> > > mesh_faces_objects;


    QMultiHash<int, QPair<QVector2D,QString>> fixations;
    QMultiHash<int, QString> fixationsAOI;

    void setupCameraParameters();
    void setupMarkerParameters();
    bool checkIntersectionRay_Triangle(const Qt3DCore::QRay3D ray, const QVector<QVector3D> triangle, qreal &tnear);
    void run_private();

    QString select_on_mesh(QVector2D mouseXYNormalized, qreal &tnear);

    QImage m_current_frame;
    QMutex frame_mutex;
    QWaitCondition wait_condition;
    QString user_value;
    QString m_suggestion;
};

#endif // SYNCHLOOP_H
