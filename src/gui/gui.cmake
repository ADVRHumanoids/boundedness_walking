set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick  REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

add_library(walking_gui_widget SHARED 
                src/gui/walking_gui_widget.cpp
                src/gui/ui/ui.qrc)

target_link_libraries(walking_gui_widget PUBLIC 
                        Qt5::Widgets 
                        Qt5::UiTools
                        ${catkin_LIBRARIES}
                      )
                      
add_executable(walking_gui src/gui/walking_gui.cpp)
target_link_libraries(walking_gui PRIVATE walking_gui_widget)
