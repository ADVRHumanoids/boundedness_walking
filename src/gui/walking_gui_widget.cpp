#include "walking_gui_widget.h"

#include <QWidget>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>


namespace
{

    QWidget * LoadUiFile(QWidget * parent)
    {
        QUiLoader loader;

        QFile file(":/walking_gui_widget.ui");
        file.open(QFile::ReadOnly);

        QWidget *formWidget = loader.load(&file, parent);
        file.close();

        return formWidget;
        
        
    }

}

WalkingGui::WalkingGui(QWidget * parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);
    
    /* Create slider/spinbox synced */
    _vel_x_sync._slider = findChild<QSlider *>("sliderX");
    _vel_x_sync._spinbox = findChild<QDoubleSpinBox *>("spinBoxX");
    _vel_x_sync.set_range(-0.2, 0.2);
    _vel_x_sync._value = &_msg.twist.linear.x;
    _vel_x_sync.connect();
    
    auto on_vel_changed = [this](double v)
    {
        _msg.twist.linear.x = v;
        _pub.publish(_msg);
    };
    
    _vel_x_sync._f = on_vel_changed;
    
    /* Create publisher */
    ros::NodeHandle nh;
    _pub = nh.advertise<geometry_msgs::TwistStamped>("multidof_walking/velocity_reference", 1);
    
    /* Buttons */
    auto * button_start = findChild<QPushButton *>("buttonStart");
    auto * button_stop = findChild<QPushButton *>("buttonStop");
    
    connect(button_start, &QPushButton::clicked,
            this, &WalkingGui::on_start);
    
    connect(button_stop, &QPushButton::clicked,
            this, &WalkingGui::on_stop);
    
}

void WalkingGui::on_start(bool checked)
{
    std_srvs::SetBool srv;
    srv.request.data = true;
    
    ros::service::call("multidof_walking/start_stop", srv);
    
    printf("Start\n");
}

void WalkingGui::on_stop(bool checked)
{
    std_srvs::SetBool srv;
    srv.request.data = false;
    
    ros::service::call("multidof_walking/start_stop", srv);
    
    printf("Stop\n");
}


void WalkingGui::SliderSync::connect()
{
    QObject::connect(_slider, &QSlider::valueChanged,
                     this, &SliderSync::on_slider_value_changed);
    
    
    
    QObject::connect(_spinbox, &QDoubleSpinBox::editingFinished,
                     this, &SliderSync::on_spinbox_value_changed);
}

void WalkingGui::SliderSync::set_range(double min, double max)
{
    _min = min;
    _max = max;
    _spinbox->setRange(min, max);
    
    _spinbox->setValue(0.0);
    _slider->setValue(val_to_perc(0.0));
}

void WalkingGui::SliderSync::on_slider_value_changed(int perc)
{
    double value = perc_to_val(perc);
    _spinbox->setValue(value);
    
    _f(value);
}

void WalkingGui::SliderSync::on_spinbox_value_changed()
{
    double value = _spinbox->value();
    _slider->setValue(val_to_perc(value));
    
    _f(value);
}

double WalkingGui::SliderSync::perc_to_val(int perc)
{
    return _min + perc/100.0 * (_max - _min);
}

int WalkingGui::SliderSync::val_to_perc(double value)
{
    return (value - _min)/(_max - _min) * 100;
}


