/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   QT HUI Application Header File
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QLabel>
#include <QTableWidget>
#include <QDir>

#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <QtWidgets/QGraphicsScene>
#include <QImage>
#include <QGraphicsPixmapItem>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Window : public QWidget
{
    Q_OBJECT

public:
    Window(QWidget *parent = nullptr, int vidSource = 0);
    ~Window();

private slots:
    void browse();  // Browse Function for CSV Files
    void browse2(); // Browse Function for Train Images Folder
    void animateFindClick();
    void saveFeature(); // Save Feature in live video feature
    void trainDatabase();  // Train Database Function call
    void startVidCapture(); // Start Live Video Feed with object recognition

private:
    // QStringList findFiles(const QStringList &files, const QString &text);
    // void showFiles(const QStringList &paths);
    QComboBox *createComboBox(const QString &text = QString());

    QComboBox *csvfileComboBox;
    QComboBox *textComboBox;
    QComboBox *traindirectoryComboBox;
    QLabel *filesFoundLabel;
    QLineEdit *objectLabel;
    QTableWidget *filesTable;
    QGraphicsScene *scene;
    QGraphicsView *view;


    cv::Mat frame;
    cv::Mat filtered_frame;
    QGraphicsPixmapItem pixmap;

    QString CSV_FILE_PATH;
    QString TRAINING_DATA_PATH;


    QPushButton *browseButton;
    QPushButton *browseButton2;
    QPushButton *saveFeatureButton;
    QPushButton *trainDatabaseButton;
    QPushButton *startButton;    
    

    bool play_video = false;

    int videoChannel = 0;
    cv::VideoCapture *cap = NULL;

    QDir currentDir;
};
#endif // MAINWINDOW_H