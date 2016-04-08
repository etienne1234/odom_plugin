#include "odom_plugin.h"
#include <ros/master.h>
#include <pluginlib/class_list_macros.h>

namespace odom_plugin {

class CurveData: public QwtArraySeriesData<QPointF>
{
public:
    CurveData()
    {
    }

     virtual QRectF boundingRect() const
     {
         if ( d_boundingRect.width() < 0.0 )
             d_boundingRect = qwtBoundingRect(*this);

         return d_boundingRect;
     }

    inline void append( const QPointF &point )
    {
        d_samples += point;
    }

    void clear()
    {
        d_samples.clear();
        d_samples.squeeze();
        d_boundingRect = QRectF( 0.0, 0.0, -1.0, -1.0 );
    }
};

OdomPlugin::OdomPlugin() : rqt_gui_cpp::Plugin()
{

    setObjectName("OdometryPlugin");
    firstTime = true;

}

void OdomPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

    myWidget = new QWidget();
    myPlot = new QwtPlot(myWidget);
    directPainter = new QwtPlotDirectPainter(myPlot);
    myCurve = new QwtPlotCurve();
    myCurve->setData( new CurveData() );
    myCurve->setSymbol(new QwtSymbol(QwtSymbol::Diamond));

    myCurve->attach(myPlot);
    x = y = 0;

    myPlot->setFrameStyle( QFrame::NoFrame );
    myPlot->setLineWidth( 0 );

    myPlot->plotLayout()->setAlignCanvasToScales( true );

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->setMajorPen( Qt::gray, 0, Qt::DotLine );
    grid->attach(myPlot);

    myPlot->setCanvasBackground( QColor( 29, 100, 141 ) ); // nice blue

    myPlot->setAxisScale( myPlot->xBottom, 0, 1000 );
    myPlot->setAxisScale( myPlot->yLeft, 0, 1000 );

    context.addWidget(myWidget);
    connect(this,SIGNAL(pointReady(QPointF)),this,SLOT(drawPoint(QPointF)));
    //ros::init(0,NULL,"odom_listener");
    subscriber = getNodeHandle().subscribe("odom",1000,&OdomPlugin::odomCallback,this);
    //ros::spin();

    //this->setGeometry(0,0,800,800);

}

OdomPlugin::~OdomPlugin()
{
    delete directPainter;
    delete myCurve;
    delete myPlot;
    delete myWidget;
}

void OdomPlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    if(firstTime)
    {

        x = msg->pose.pose.position.y;
        y = msg->pose.pose.position.y;
        firstTime = false;

    }
    else
    {

        if(x == msg->pose.pose.position.y && msg->pose.pose.position.y)
        {
            return;
        }
        else
        {
            x = msg->pose.pose.position.y;
            y = msg->pose.pose.position.y;
        }
    }

    QPointF myPoint(x,y);
    emit pointReady(myPoint);
        
}

void OdomPlugin::drawPoint(QPointF myPoint)
{


        CurveData *data = static_cast<CurveData *>( myCurve->data() );
        data->append( myPoint );
        x++;
        y++;

        const bool doClip = !myPlot->canvas()->testAttribute( Qt::WA_PaintOnScreen );
        if ( doClip )
        {
            const QwtScaleMap xMap = myPlot->canvasMap( myCurve->xAxis() );
            const QwtScaleMap yMap = myPlot->canvasMap( myCurve->yAxis() );

            QRegion clipRegion;

            const QSize symbolSize = myCurve->symbol()->size();
            QRect r( 0, 0, symbolSize.width() + 2, symbolSize.height() + 2 );

            const QPointF center =
                QwtScaleMap::transform( xMap, yMap, myPoint );
            r.moveCenter( center.toPoint() );
            clipRegion += r;

            directPainter->setClipRegion( clipRegion );
        }

        directPainter->drawSeries( myCurve,
        data->size() - 1, data->size() - 1 );
        myPlot->replot();
        myPlot->repaint();

}

void OdomPlugin::shutdownPlugin()
{

    subscriber.shutdown();

}

} // odom_plugin

PLUGINLIB_EXPORT_CLASS(odom_plugin::OdomPlugin,rqt_gui_cpp::Plugin)

//PLUGINLIB_DECLARE_CLASS(odom_plugin, OdomPlugin, odom_plugin::OdomPlugin,rqt_gui_cpp::Plugin)
