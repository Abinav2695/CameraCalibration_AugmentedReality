/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition QT GUI Functions
   Source file
*/

#include "mainwindow.h"
#include <QPushButton>
#include <QGridLayout>
#include <QLineEdit>
#include <QGridLayout>
#include <QShortcut>
#include <QHeaderView>
#include <QMessageBox>
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QFileDialog>
#include <QTimer>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include "rtor.h"


Window::Window(QWidget *parent, int vidSource) 
        : QWidget(parent)
{
    videoChannel = vidSource;
    QString PATH = QDir::toNativeSeparators(QDir::currentPath());
    std::string PROJECT_FOLDER_PATH = PATH.toStdString();
    PROJECT_FOLDER_PATH =  PROJECT_FOLDER_PATH.substr(0, PROJECT_FOLDER_PATH.rfind("/Assgn_3"));
    std::string csv_file_path = PROJECT_FOLDER_PATH + "/Assgn_3/csv_files/dataset.csv";
    std::string training_dataset_path = PROJECT_FOLDER_PATH + "/Assgn_3/training_datasets/";
    CSV_FILE_PATH = QString::fromStdString(csv_file_path);
    TRAINING_DATA_PATH = QString::fromStdString(training_dataset_path);



   
    setWindowTitle(tr("Real Time 2D Object Detection"));
    browseButton = new QPushButton(tr("&Browse..."), this);
    browseButton2 = new QPushButton(tr("&Browse..."), this);
    saveFeatureButton = new QPushButton(tr("&Save Feature..."), this);
    trainDatabaseButton = new QPushButton(tr("&Train Database..."), this);
    startButton = new QPushButton(tr("&Start"), this);    

    objectLabel = new QLineEdit(tr(""), this);
    objectLabel->setPlaceholderText(tr("Enter Object Label Here"));

    connect(browseButton, &QAbstractButton::clicked, this, &Window::browse);
    connect(browseButton2, &QAbstractButton::clicked, this, &Window::browse2);
    connect(startButton, &QAbstractButton::clicked, this, &Window::startVidCapture);
    connect(saveFeatureButton, &QAbstractButton::clicked, this, &Window::saveFeature);
    connect(trainDatabaseButton, &QAbstractButton::clicked, this, &Window::trainDatabase);

    csvfileComboBox = createComboBox(CSV_FILE_PATH);
    connect(csvfileComboBox->lineEdit(), &QLineEdit::returnPressed,
            this, &Window::animateFindClick);
    
    traindirectoryComboBox = createComboBox(TRAINING_DATA_PATH);
    connect(traindirectoryComboBox->lineEdit(), &QLineEdit::returnPressed,
            this, &Window::animateFindClick);

    filesFoundLabel = new QLabel;

    scene = new QGraphicsScene(this);
    scene->setSceneRect(QRectF(0,0,1000,1000));
    view = new QGraphicsView(scene);
    scene->addItem(&pixmap);
    // createFilesTable();

    QGridLayout *mainLayout = new QGridLayout(this);
    mainLayout->addWidget(new QLabel(tr("CSV File:")), 0, 0,1 ,1);
    mainLayout->addWidget(csvfileComboBox, 0, 1, 1, 5); //, 1, 2);
    mainLayout->addWidget(browseButton, 0, 6);
    // mainLayout->addWidget(new QLabel(tr("Containing text:")), 1, 0);
    // mainLayout->addWidget(textComboBox, 1, 1, 1, 2);
    mainLayout->addWidget(new QLabel(tr("Train Database:")), 2, 0, 1, 1);
    mainLayout->addWidget(traindirectoryComboBox, 2, 1, 1, 5);
    mainLayout->addWidget(browseButton2, 2, 6,1,1);
    mainLayout->addWidget(view, 3, 0, 1, 6);

    mainLayout->addWidget(new QLabel(tr("Object Label : ")), 4, 0, 1 ,1);
    mainLayout->addWidget(objectLabel, 4, 1, 1, 1);
    mainLayout->addWidget(saveFeatureButton, 4, 2, 1, 1);
    mainLayout->addWidget(trainDatabaseButton, 4, 3, 1, 1);
    mainLayout->addWidget(startButton, 4, 6);


    connect(new QShortcut(QKeySequence::Quit, this), &QShortcut::activated,
        qApp, &QApplication::quit);

}

Window::~Window(){
    
    startButton->click();
    QTimer *timer = new QTimer(this);
    QEventLoop loop;
    timer->setInterval(1000);

     // Connect the timer's timeout signal to the QEventLoop's quit() slot
    QObject::connect(timer, &QTimer::timeout, &loop, &QEventLoop::quit);

    timer->start();
    loop.exit();
    if(cap != NULL){
        cap->release();
    }
}


void Window::browse()
{
    QString directory =
        QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Find Files"), QDir::currentPath(),tr("CSV Files (*.csv)")));

    if (!directory.isEmpty()) {
        if (csvfileComboBox->findText(directory) == -1)
            csvfileComboBox->addItem(directory);
        csvfileComboBox->setCurrentIndex(csvfileComboBox->findText(directory));
    }
    CSV_FILE_PATH = csvfileComboBox->currentText();
}

void Window::browse2()
{
    QString directory =
        QDir::toNativeSeparators(QFileDialog::getExistingDirectory(this, tr("Find Files"), QDir::currentPath()));

    if (!directory.isEmpty()) {
        if (traindirectoryComboBox->findText(directory) == -1)
            traindirectoryComboBox->addItem(directory);
        traindirectoryComboBox->setCurrentIndex(traindirectoryComboBox->findText(directory));
    }
    TRAINING_DATA_PATH = traindirectoryComboBox->currentText();
}

QComboBox *Window::createComboBox(const QString &text)
{
      QComboBox *comboBox = new QComboBox;
      comboBox->setEditable(true);
      comboBox->addItem(text);
      comboBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      return comboBox;
}


void Window::animateFindClick()
{   
    
    startButton->animateClick();
}


void Window::trainDatabase(){

    std::string label =  objectLabel->text().toStdString();
    if(label == ""){
        QMessageBox::critical(this,
                                  "Object Label Error",
                                  "Make sure you entered a valid object label");
        return;
    }
    std::string csv_file = CSV_FILE_PATH.toStdString();
    std::string folder = TRAINING_DATA_PATH.toStdString() + '/';
    std::cout << folder << "  " << label << std::endl;
    train_folder((char *)folder.c_str(), (char *)csv_file.c_str(), (char *)label.c_str());
    objectLabel->setText(tr(""));
}

void Window::saveFeature(){
    
    if(play_video){
        std::string label =  objectLabel->text().toStdString();
        if(label == ""){
            QMessageBox::critical(this,
                                    "Object Label Error",
                                    "Make sure you entered a valid object label");
            return;
        }
        std::string csv_file = CSV_FILE_PATH.toStdString();
        save_new_feature(frame, (char *)csv_file.c_str(), (char *)label.c_str());
        objectLabel->setText(tr(""));
        QMessageBox::critical(this,
                                    "Saved",
                                    "Feature Set has been added to database");
    } else{
        QMessageBox::critical(this, "Video Capture Error", "Please Start Video Capture First!!");
    }
    
}


void Window::startVidCapture(){

    if(!play_video){
        
        cap = new cv::VideoCapture(videoChannel);

        //Check if the camera is accessible else exit program
        if (!cap->isOpened()) 
        {
            std::cerr << "ERROR: Cannot open video capture device" << std::endl;
            return;
        }

        // Get the size of the input image from the camera
        cv::Size refS( (int) cap->get(cv::CAP_PROP_FRAME_WIDTH ),
                        (int) cap->get(cv::CAP_PROP_FRAME_HEIGHT));

        std::cout << "INFO: Real Time Object 2-D Recongnition" << std::endl;

        startButton->setText("Stop");
        play_video = true;
        // Video Loop
        while(play_video)
        {
            cap->read(frame); //get new frame
            if (frame.empty())
            {
                std::cout << "WARNING: Frame is empty" << std::endl;
                continue;
            }

            std::vector< std::vector<float>> components;
            std::vector< std::vector<float>> feature_set;

            darkenImage(frame, filtered_frame);
            threshold(filtered_frame, filtered_frame , kThreshOtsu);
            GTMorph(filtered_frame, 2, kClosingMode);
            GTMorph(filtered_frame, 2, kOpeningMode);
            findConnectedComponents(filtered_frame, components);

            // if(components.size()>0) {
            //     std::vector<float> feature_set;
            //     extract_feature(frame, feature_set);
            //     std::string csv_file = CSV_FILE_PATH.toStdString();
            //     std::string Final_match = feature_match((char *)csv_file.c_str(), feature_set, KNearestNeighbour, 3);
            //     std::cout << "Got a match :: " << Final_match << std::endl;
            //     cv::rectangle(frame, cv::Rect(components[0][0], components[0][1], components[0][2], components[0][3]), cv::Scalar(50, 255, 50), 2);
            //     // cv2.putText(image, 'Fedex', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            //     cv::putText(frame, Final_match, cv::Point(components[0][0], components[0][1] - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, (50, 255, 50), 2);
            // }

            if(components.size()>0) {
                std::string csv_file = CSV_FILE_PATH.toStdString();
                compute_features(filtered_frame,components,feature_set);
                for(int i=0; i<feature_set.size(); i++){
                                   
                    std::string Final_match = feature_match((char *)csv_file.c_str(), feature_set[i], KNearestNeighbour, 3);
                    std::cout << "Got a match :: " << Final_match << std::endl;
                    cv::rectangle(frame, cv::Rect(components[i][0], components[i][1], components[i][2], components[i][3]), cv::Scalar(50, 255, 50), 2);
                    cv::putText(frame, Final_match, cv::Point(components[i][0], components[i][1] - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, (50, 255, 50), 2);
                }
            }
            QImage qimg(frame.data,
                            frame.cols,
                            frame.rows,
                            frame.step,
                            QImage::Format_RGB888);
            pixmap.setPixmap( QPixmap::fromImage(qimg.rgbSwapped()));
            view->fitInView(&pixmap, Qt::KeepAspectRatio);

            qApp->processEvents();
        }
        cap->release();
        startButton->setText("Start");
    } else{
        play_video = false;
        std::cout << "Stop Pressed " << std::endl;
    }
}
