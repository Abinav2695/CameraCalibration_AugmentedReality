/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition QT GUI Main File
   Source file
*/

#include <QApplication>
#include <QLabel>
#include "mainwindow.h"
#include <iostream>
#include <map>
#include <string>
#include <vector>



int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    if(argc<2) //check if any arguments are passed 
    {
        std::cout << "Please input the Video Device ID!!!" << std::endl;
        std::cout << "Usage : " << argv[0] << " {Video Device ID}" <<std::endl;
        std::cout << "Example : " << argv[0] << " 0/1/2/3/..." <<std::endl;
        return 0;
    }
    int vid_src  = atoi(argv[1]);

    Window window(nullptr, vid_src);
    window.show();
    return app.exec();
}


