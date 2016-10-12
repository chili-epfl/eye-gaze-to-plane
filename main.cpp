#include <QApplication>
#include <QtQml>
#include <QQuickView>
#include "synchloop.h"
#include <QtQuick>
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QCommandLineParser parser;
    parser.setApplicationDescription("Eye Fixation Parser");
    parser.addHelpOption();

    QCommandLineOption videoFile("video", "Video file","Video file");
    parser.addOption(videoFile);

    QCommandLineOption fixationFile("fixation", "Fixation file","Fixation file");
    parser.addOption(fixationFile);

    QCommandLineOption interactive("interactive", "Interactive","Interactive");
    parser.addOption(interactive);

    QCommandLineOption subtitles("subs", "Generate subtitles");
    parser.addOption(subtitles);

    QCommandLineOption skip_intervals("skipIntervals", "Intervals to skip format hh::mm::ss-hh:mm:ss;hh::mm::ss-hh:mm:ss...","Intervals to skip");
    parser.addOption(skip_intervals);

    parser.process(app);

    if(!parser.isSet(videoFile) || !parser.isSet(fixationFile)){
        qDebug()<<"Missing options";
        return -1;
    }

    qmlRegisterType<SynchLoop>("SynchLoop",1,0,"SynchLoop");

    QQuickView view;

    view.resize(200, 200);
    view.setResizeMode(QQuickView::SizeViewToRootObject);
    view.setSource(QUrl("qrc:/main_synch_loop.qml"));

    view.show();

    view.rootContext()->setContextProperty("video_file",parser.value(videoFile));
    view.rootContext()->setContextProperty("gaze_data_file",parser.value(fixationFile));
    if(parser.isSet(skip_intervals))
        view.rootContext()->setContextProperty("skip_intervals",parser.value(skip_intervals));
    else
        view.rootContext()->setContextProperty("skip_intervals","");

    if(parser.isSet(subtitles))
        view.rootContext()->setContextProperty("enable_subs",true);
    else
        view.rootContext()->setContextProperty("enable_subs",false);
    if(parser.isSet(interactive))
        view.rootContext()->setContextProperty("interactive",true);
    else
        view.rootContext()->setContextProperty("interactive",false);

    QObject::connect(view.rootObject(), SIGNAL(quit()), qApp, SLOT(quit()));
    return app.exec();
}
