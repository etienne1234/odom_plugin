#ifndef odom_plugin__ODOMPLUGIN_H
#define odom_plugin__ODOMPLUGIN_H

#include "qwt_plot.h"
#include "qwt_plot_seriesitem.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_directpainter.h"
#include "qwt_symbol.h"
#include "qwt_plot_grid.h"
#include "qwt_plot_layout.h"
#include "qwt_symbol.h"
#include <QTimer>
#include <QApplication>
#include <QDesktopWidget>
#include <QSize>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <rqt_gui_cpp/plugin.h>

namespace odom_plugin {

class OdomPlugin : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:
    OdomPlugin();
    virtual ~OdomPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin(); 

protected:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

signals: 
    void pointReady(QPointF myPoint);

public slots:
    void drawPoint(QPointF myPoint);

private:
    QWidget *myWidget;
    QwtPlot *myPlot;
    QwtPlotCurve *myCurve;
    QwtPlotDirectPainter *directPainter;
    ros::Subscriber subscriber;
    //ros::NodeHandle nh;
    int x,y;
    bool firstTime;

};
}

#endif // odom_plugin__ODOMPLUGIN_H

