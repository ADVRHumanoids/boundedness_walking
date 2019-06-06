#include "walking_gui_widget.h"

#include <QApplication>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "walking_gui");
    
    QApplication app(argc, argv);
    WalkingGui gui;
    gui.show();
    return app.exec();
}


