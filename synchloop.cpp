#include "synchloop.h"
#include <QDebug>
#include <QQuaternion>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <QtConcurrent>

#define DIST_THRESHOLD 30
const char separator=',';

/*Utility function to skip comments and new lines in the config file*/
QString readLine(QTextStream* inputStream, char separator=separator){
    QString line;
    bool isValid=false;
    do{
        line=inputStream->readLine();
        if(line.isEmpty())
            continue;
        if(line.contains("#"))
            line=line.split("#").at(0);
        bool allEmpty=true;
        Q_FOREACH(QString part, line.split(separator)){
            if(!part.isEmpty()){
                allEmpty=false;
                break;
            }
        }
        if(allEmpty)
            continue;
        isValid=true;
    }while(!isValid);
    return line;
}

SynchLoop::SynchLoop(QObject *parent) : QObject(parent)
{
    ar_param=NULL;
    ar_handle=NULL;
    ar_3d_handle=NULL;
    ar_patt_handle=NULL;
    frame_counter=0;

    /*Configuration for the mobile eyetracker SMI*/
    cameraSize=QSize(1280,960);
    projectionMatrix=QMatrix4x4(1.1326430126128812e+03, 0., 6.3950000000000000e+02,0,
                                0.   ,     1.1326430126128812e+03, 4.7950000000000000e+02, 0,
                                0., 0., 1,0.,
                                0,0,0,1);
    distortionCoeff=QVector4D();


    camera.setProjectionType(Qt3DCore::QCameraLens::FrustumProjection);
    camera.setNearPlane(0.1);
    camera.setFarPlane(10000);
    camera.setTop( 0.1*(projectionMatrix(1,2)/(projectionMatrix(0,0))));
    camera.setBottom(-0.1*(projectionMatrix(1,2)/(projectionMatrix(0,0))));
    camera.setLeft(-0.1*(projectionMatrix(0,2)/(projectionMatrix(1,1))));
    camera.setRight(0.1*(projectionMatrix(0,2)/(projectionMatrix(1,1))));
    camera.setPosition(QVector3D(0,0,0));
    camera.setUpVector(QVector3D(0,1,0));
    camera.setViewCenter(QVector3D(0,0,-1));


    write_subtitles=false;
    subtitles_counter=0;

    setupCameraParameters();
    setupMarkerParameters();

}

void SynchLoop::setFileVideoFileName(QString video_file_name,QString skip_intervals)
{
    this->video_file_name=video_file_name;
    //video_capture.open(1);
    video_capture.open(video_file_name.toStdString());
    tot_frame=video_capture.get(CV_CAP_PROP_FRAME_COUNT);
    if(!video_capture.isOpened()){
        qFatal("Cannot open video file");
    }
    if(write_subtitles){
        subtitle_file.setFileName(video_file_name+".srt");
        subtitle_file.open(QFile::WriteOnly | QFile::Truncate);
        if(subtitle_file.isOpen()){
            subtitles_stream.setDevice(&subtitle_file);
        }
        else{
            qDebug()<<"Can't make subtitles";
            write_subtitles=false;
        }
    }

    if(!skip_intervals.isEmpty()){
        QStringList intervals=skip_intervals.split(";");
        Q_FOREACH(QString interval, intervals){
            QString start=interval.split("-")[0];
            QString end=interval.split("-")[1];

            QStringList start_parts=start.split(":");
            QStringList end_parts=end.split(":");

            int start_frame= (start_parts[0].toInt()*3600+start_parts[1].toInt()*60+start_parts[3].toInt())*fps;
            int end_frame= (end_parts[0].toInt()*3600+end_parts[1].toInt()*60+end_parts[3].toInt())*fps;

            for(int i=start_frame;i<=end_frame;i++){
                skip_frames.append(i);
            }
        }
    }

}

void SynchLoop::loadGazeData(QString file_name){
    gaze_data_file_name=file_name;
    int fixationGroup=0;
    QString event_type;
    QFile file(file_name);
    if(!file.open(QFile::ReadOnly)){
        qDebug()<<"Can't open gaze file";
        return;
    }
    QTextStream stream(&file);
    //Read line and split
    QString line;
    while(!stream.atEnd()){
        line=readLine(&stream);
        QStringList parts=line.split("\t");

        if(parts.size()!=5){
            qDebug()<<"Invalid file format for gaze data";
            return;
        }
        if(parts[2].contains("nan",Qt::CaseInsensitive) ||
                parts[3].contains("nan",Qt::CaseInsensitive) ||
                parts[2].toDouble()<0 ||
                parts[3].toDouble()<0)
           continue;
        if(parts[1]!=event_type && parts[1]=="Visual Intake")
            fixationGroup++;
        event_type=parts[1];

        QVector2D pos(parts[2].toDouble(),parts[3].toDouble());

        QString timestamp=parts[4];
        QStringList timestamp_parts=timestamp.split(':');
        int frame;

        frame=timestamp_parts[0].toInt()*3600*fps+
                    timestamp_parts[1].toInt()*60*fps+
                    timestamp_parts[2].toInt()*fps+
                    round(timestamp_parts[3].toInt()*(fps/1000.0));

        QPair<QVector2D,QString> fixation;
        fixation.first=pos;
        fixation.second=line+","+QString::number(fixationGroup);
        fixations.insert(frame,fixation);
    }
    file.close();
}


void SynchLoop::run(){
      QtConcurrent::run(this,&SynchLoop::run_private);
}

void SynchLoop::run_private(){

    cv::Mat frame;

    AR2VideoBufferT* ar_buffer;
    ar_buffer=(AR2VideoBufferT*)malloc(sizeof(AR2VideoBufferT));

    int marker_num;

    ARMarkerInfo* marker_info;
    QMatrix3x3 rotMat;
    qreal err;

    QQuaternion openglAlignment=QQuaternion::fromAxisAndAngle(1,0,0,180);

    m_current_frame= QImage(cameraSize.width(),cameraSize.height(),QImage::Format_Grayscale8);

    cv::Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
    cv::BackgroundSubtractorMOG2 pMOG2; //MOG2 Background subtractor

    pMOG2 = cv::BackgroundSubtractorMOG2();
    QString latest_AOI;
    QVector2D prev_gaze(-100,-100);

    while(video_capture.read(frame)){
        if(skip_frames.contains(frame_counter)){
            frame_counter++;
            continue;
        }

        cv::Mat gray_img;
        cv::cvtColor(frame, gray_img, CV_BGR2GRAY,1);

        ar_buffer->buff=gray_img.data;
        ar_buffer->bufPlaneCount=0;
        ar_buffer->bufPlanes=NULL;
        ar_buffer->fillFlag=1;
        ar_buffer->time_sec=(ARUint32)QDateTime::currentMSecsSinceEpoch()/1000;
        ar_buffer->time_usec=(ARUint32)QDateTime::currentMSecsSinceEpoch()-ar_buffer->time_sec*1000;
        //ar_buffer->buffLuma=ar_buffer->buff;

        if (arDetectMarker(ar_handle, ar_buffer->buff) < 0) {
            qDebug()<<"Error in arDetectMarker";
            continue;
        }
        marker_num=arGetMarkerNum(ar_handle);
        /*Detection*/
        if(marker_num>0){
            marker_info=arGetMarker(ar_handle);
            Q_FOREACH(QString id, ar_multimarker_objects.keys()){
                AR3DMultiPatternObject* o=ar_multimarker_objects[id];
                err=arGetTransMatMultiSquareRobust(ar_3d_handle,marker_info,marker_num,o->marker_info);
                if(err<0)
                    o->visible=false;
                else{
                    /*Filter*/
                    arFilterTransMat(o->ftmi,o->marker_info->trans,!o->visible);
                    /*...*/
                    o->visible=true;
                    for(int i=0;i<3;i++)
                        for(int j=0;j<3;j++)
                            rotMat(i,j)=o->marker_info->trans[i][j];

                    QQuaternion alignedRotation=openglAlignment*QQuaternion::fromRotationMatrix(rotMat);
                    rotMat=alignedRotation.toRotationMatrix();

                    for(int i=0;i<3;i++)
                        for(int j=0;j<3;j++)
                            o->pose(i,j)=rotMat(i,j);
                    o->pose(0,3)=o->marker_info->trans[0][3];
                    o->pose(1,3)=-o->marker_info->trans[1][3];
                    o->pose(2,3)=-o->marker_info->trans[2][3];
                }
            }
        }
        /*Intersection Test*/
        QString hitAOI;

        if(fixations.contains(frame_counter)){
            pMOG2.operator ()(gray_img,fgMaskMOG2);
            int nonZero=cv::countNonZero(fgMaskMOG2);
            Q_FOREACH(auto fixation,fixations.values(frame_counter)){
                QVector2D gaze_point=fixation.first;
                //If the movement in the image is low or the gaze didn't move much we use the latest AOI
                //arzu changed
                if(false){
                //if(nonZero<0.3*cameraSize.width()*cameraSize.height() && prev_gaze.distanceToPoint(gaze_point)<20){
                    hitAOI=latest_AOI;
                }
                else{
                    QVector2D gaze_point_norm=QVector2D((2.0f * gaze_point.x()) / cameraSize.width() - 1.0f, 1.0f - (2.0f * gaze_point.y()) / cameraSize.height());
                    qreal dist_mesh;
                    hitAOI=select_on_mesh(gaze_point_norm,dist_mesh);
                    prev_gaze=gaze_point;
                    if(interactive){
                        frame_mutex.lock();
                        cv::circle(frame,cv::Point2f(gaze_point.x(),gaze_point.y()),10,cv::Scalar(0,0,255),-1);
                        cv::imwrite("current_frame.png",frame);
                        emit current_frameChanged();
                        m_suggestion=hitAOI;
                        emit suggestionChanged();
                        wait_condition.wait(&frame_mutex);
                        frame_mutex.unlock();
                        hitAOI=user_value;
                    }
                }
                fixationsAOI.insert(frame_counter,hitAOI);
                latest_AOI=hitAOI;
            }
        }
        if(write_subtitles){
            if(frame_counter%fps==0){
                subtitles_counter++;
                double seconds =(double)frame_counter/fps;
                double int_seconds;
                double ms=modf(seconds,&int_seconds);

                int hh= int_seconds/3600;
                int mm= (int_seconds-hh*3600)/60;
                int s= (int_seconds-hh*3600-mm*60);

                subtitles_stream <<"\n\n" << subtitles_counter << "\n";
                subtitles_stream<<QString::number(hh)<<":"<<QString::number(mm)<<":"<<QString::number(s)<<",000"
                               <<" --> "<<QString::number(hh)<<":"<<QString::number(mm)<<":"<<QString::number(s+1)<<",000\n";
            }
            if(!hitAOI.isEmpty())
                subtitles_stream<<hitAOI<<";";
        }
        frame_counter++;
    }
    video_capture.release();

    QFileInfo fixation_file_info(gaze_data_file_name);
    QString fixation_AOI_file_path=fixation_file_info.absolutePath()+"/"+fixation_file_info.baseName()+"_AOI.txt";
    QFile fixation_with_AOI(fixation_AOI_file_path);
    fixation_with_AOI.open(QFile::WriteOnly| QFile::Truncate);
    QTextStream file_with_AOI_stream(&fixation_with_AOI);

    auto timestamps=fixationsAOI.keys().toSet().toList();
    qSort(timestamps.begin(), timestamps.end());
    Q_FOREACH(auto timestamp, timestamps){
        int i=1;
        /* Remeber that the  function values()
         * returns a list of all the values associated with the key, from the most recently inserted to the least recently inserted.
         * Hence the order between fixationsAOI and fixation is inverted*/
        auto fixations_at_timestamp=fixations.values(timestamp);
        Q_FOREACH(auto AOI,fixationsAOI.values(timestamp)){
            int hh=timestamp/(fps*3600);
            int mm=(timestamp-hh*3600*fps)/(60*fps);
            int ss=(timestamp-hh*3600*fps - mm*60*fps)/fps;
            int fms=timestamp-hh*3600*fps - mm*60*fps - ss*fps;

            if(AOI.isEmpty()){
                AOI="0,0";
            }
            file_with_AOI_stream<<fixations_at_timestamp[fixations_at_timestamp.size()-i].second<<","<<AOI<<","<<QString::number(hh)<<":"
                               <<QString::number(mm)<<":"<<QString::number(ss)<<":"<<fms<<"\n";
            i++;
        }
    }

    file_with_AOI_stream.flush();
    fixation_with_AOI.close();

    if(write_subtitles){
        subtitles_stream.flush();
        subtitle_file.close();

    }
    emit done();

}

void SynchLoop::setupCameraParameters()
{
    ARParam cparam;
    /*
        * The values for  dist_function_version  correspond to the following algorithms:
        version 1: The original ARToolKit lens model, with a single radial distortion factor, plus center of distortion.<br>
        version 2: Improved distortion model, introduced in ARToolKit v4.0. This algorithm adds a quadratic term to the radial distortion factor of the version 1 algorithm.<br>
        version 3: Improved distortion model with aspect ratio, introduced in ARToolKit v4.0. The addition of an aspect ratio to the version 2 algorithm allows for non-square pixels, as found e.g. in DV image streams.<br>
        version 4: OpenCV-based distortion model, introduced in ARToolKit v4.3. This differs from the standard OpenCV model by the addition of a scale factor, so that input values do not exceed the range [-1.0, 1.0] in either forward or inverse conversions.
        */

    cparam.xsize=cameraSize.width();
    cparam.ysize=cameraSize.height();
    cparam.dist_function_version=4;
    cparam.dist_factor[0]=distortionCoeff.x();
    cparam.dist_factor[1]=distortionCoeff.y();
    cparam.dist_factor[2]=distortionCoeff.z();
    cparam.dist_factor[3]=distortionCoeff.w();

    cparam.dist_factor[4]=projectionMatrix.operator ()(0,0);
    cparam.dist_factor[5]=projectionMatrix.operator ()(1,1);
    cparam.dist_factor[6]=projectionMatrix.operator ()(0,3);
    cparam.dist_factor[7]=projectionMatrix.operator ()(1,3);
    cparam.dist_factor[8]=1;

    int i,j;
    for(i=0;i<3;i++){
        for(j=0;j<3;j++)
            cparam.mat[i][j]=projectionMatrix.operator ()(i,j);
        cparam.mat[i][3]=0;
    }

    if ((ar_param = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        qWarning("Error in creating ar_param");
        return;
    }

    if ((ar_handle = arCreateHandle(ar_param)) == NULL) {
        qWarning("Error in creating ar_handle");
        return;

    }
    if (arSetPixelFormat(ar_handle, AR_PIXEL_FORMAT_MONO) < 0) {
        qWarning("Error in setting pixel format");
        return;
    }
    if (arSetDebugMode(ar_handle, AR_DEBUG_DISABLE) < 0) {
        qWarning("Error in setting debug mode");
        return;
    }

    if ((ar_3d_handle = ar3DCreateHandle(&cparam)) == NULL) {
        qWarning("Error in creating ar_3d_handle");
        return;
    }

}

void SynchLoop::setupMarkerParameters()
{
    if(ar_handle){
        arSetLabelingThreshMode(ar_handle, AR_LABELING_THRESH_MODE_MANUAL);
        arSetLabelingThresh(ar_handle,AR_DEFAULT_LABELING_THRESH);
        arSetPatternDetectionMode(ar_handle,AR_TEMPLATE_MATCHING_MONO_AND_MATRIX);
        arSetMatrixCodeType(ar_handle, matrix_code_type);
        if(ar_patt_handle==NULL){
            ar_patt_handle=arPattCreateHandle();
        }
        arPattAttach(ar_handle, ar_patt_handle);
    }
}

void SynchLoop::loadMultiMarkersConfigFile(QString config_name,QString file_path)
{
    if(!ar_handle){
        qWarning("AR Handle is null");
        return;
    }
    ARMultiMarkerInfoT* marker_info=arMultiReadConfigFile(file_path.toStdString().c_str(),ar_patt_handle);
    if(marker_info!=NULL){
        marker_info->min_submarker=0;
        AR3DMultiPatternObject* o=new AR3DMultiPatternObject;
        o->marker_info=marker_info;
        o->visible=false;
        o->ftmi=arFilterTransMatInit(25,15);
        ar_multimarker_objects[config_name]=o;
    }
    else{
        qWarning()<<"Cannot load multipatter file";
    }

}

void SynchLoop::loadMesh(QString file_path,QString transform_name){

    QFile file(file_path);
    if(file.open(QFile::ReadOnly)){
        QTextStream stream(&file);

        QString line;

        QString objectName;
        int face_offset;
        int verteces_count=0;
        while (!stream.atEnd()) {
            line = stream.readLine();
            if(line.at(0)==QChar('#')) continue;
            if(line.at(0)==QChar('o')){
                QStringList parts= line.split(" ");
                objectName=parts[1];
                mesh_name2pose_name[objectName]=transform_name;
                face_offset=verteces_count;
            }
            else if(line.at(0)==QChar('v')){
                QStringList parts= line.split(" ");
                mesh_vertices_objects[objectName].append(QVector3D(parts[1].toFloat(),parts[2].toFloat(),parts[3].toFloat()));
                verteces_count++;
            }
            else if(line.at(0)==QChar('f')){
                QStringList parts= line.split(" ");
                QVector<int> face;
                if(parts[1].toInt()<0 || parts[2].toInt()<0 || parts[3].toInt()<0)
                    qFatal("Obj is using a negative index");
                face.append(parts[1].toInt()-face_offset);
                face.append(parts[2].toInt()-face_offset);
                face.append(parts[3].toInt()-face_offset);
                mesh_faces_objects[objectName].append(face);
            }
        }
        file.close();
    }
    else
        qDebug("Can't open mesh file");


}


QString SynchLoop::select_on_mesh(QVector2D mouseXYNormalized,qreal &tnear){
    QVector4D ray_clip(mouseXYNormalized.x(),mouseXYNormalized.y(),-1,1);
    QVector4D ray_eye = camera.projectionMatrix().inverted().map(ray_clip);
    ray_eye.setZ(-1);ray_eye.setW(0);

    QVector4D ray_wor_4D = camera.viewMatrix().inverted().map(ray_eye);
    QVector3D ray_wor(ray_wor_4D.x(),ray_wor_4D.y(),ray_wor_4D.z());
    ray_wor.normalize();

    Qt3DCore::QRay3D ray;
    ray.setOrigin(QVector3D(0,0,0));
    ray.setDirection(ray_wor);

    qreal hitEntity_tnear=DBL_MAX;
    QString hitEntity;

    QVector<QVector3D> triangle(3,QVector3D());
    Q_FOREACH(QString AOI, mesh_faces_objects.keys()){
        if(!ar_multimarker_objects.contains(mesh_name2pose_name[AOI])) continue;
        if(!ar_multimarker_objects[mesh_name2pose_name[AOI]]->visible) continue;
        QMatrix4x4 pose_inv= ar_multimarker_objects[mesh_name2pose_name[AOI]]->pose.inverted();
        Qt3DCore::QRay3D ray_transform(pose_inv*ray.origin(),pose_inv.mapVector(ray.direction()));
        Q_FOREACH(QVector<int> triangle_face, mesh_faces_objects[AOI]){
            triangle[0]=mesh_vertices_objects[AOI][triangle_face[0]-1];
            triangle[1]=mesh_vertices_objects[AOI][triangle_face[1]-1];
            triangle[2]=mesh_vertices_objects[AOI][triangle_face[2]-1];
            qreal t_near;
            QString pos;
            if(checkIntersectionRay_Triangle(ray_transform,triangle,t_near,pos)){
                if(t_near<hitEntity_tnear){
                    hitEntity_tnear=t_near;
                    //hitEntity=AOI;
                    hitEntity=pos;
                }
            }
        }
    }

    tnear=hitEntity_tnear;
    return hitEntity;
}

/*Based on http://geomalgorithms.com/a06-_intersect-2.html#intersect3D_RayTriangle%28%29*/
bool SynchLoop::checkIntersectionRay_Triangle(const Qt3DCore::QRay3D ray, const QVector<QVector3D> triangle,qreal &tnear,QString& pos ){
    if(triangle.size()!=3){qWarning()<<"Triangle doesn't have 3 points!";return false;}

    QVector3D u=triangle[1]-triangle[0]; //precomputable
    QVector3D v=triangle[2]-triangle[0]; //precomputable

    QVector3D n= QVector3D::crossProduct(u,v);

    if(n.length()==0){
        qWarning()<<"Triangle is degenerate";return false;}

    QVector3D w0=ray.origin()-triangle[0];

    qreal a=-QVector3D::dotProduct(n,w0);
    qreal b=QVector3D::dotProduct(n,ray.direction());

    if(fabs(b)<0.00000001){
        if(a==0) {qWarning()<<"Ray lies in triangle plane";return false;}

        return false;
    }

    qreal r=a/b;

    if(r<0.0)
        return 0;

    QVector3D P=ray.origin()+r*ray.direction();

    qreal uu, uv, vv, wu, wv, D;

    uu = QVector3D::dotProduct(u,u);
    uv = QVector3D::dotProduct(u,v);
    vv = QVector3D::dotProduct(v,v);
    QVector3D w = P - triangle[0];
    wu = QVector3D::dotProduct(w,u);
    wv = QVector3D::dotProduct(w,v);
    D = uv * uv - uu * vv;

    qreal s, t;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)         // I is outside T
        return false;
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  // I is outside T
        return false;

    tnear=r;

    pos=QString::number(P.x())+","+QString::number(P.y());
    //fixationsAOI.insert(frame_counter,pos);
    
    //qDebug()<<P;

    return 1;                       // I is in T
}

