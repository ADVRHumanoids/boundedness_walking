#include <QtUiTools/QtUiTools>
#include <QWidget>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

class WalkingGui : public QWidget
{
    
public:
    
    WalkingGui(QWidget * parent = nullptr);
    
private:
    
    struct SliderSync : public QObject
    {
        double _min, _max;
        QSlider * _slider;
        QDoubleSpinBox * _spinbox;
        std::function<void(double)> _f;
        
        void connect();
        void set_range(double min, double max);
        void on_slider_value_changed(int perc);
        void on_spinbox_value_changed();
        
        int val_to_perc(double value);
        double perc_to_val(int perc);
    };
    
    void on_start(bool checked);
    void on_stop(bool checked);
    
    SliderSync _vel_x_sync, _kp_sync, _kd_sync;
    
    geometry_msgs::TwistStamped _msg;
    ros::Publisher _pub, _kp_pub, _kd_pub;
    
};
