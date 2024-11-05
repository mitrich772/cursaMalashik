#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QPen>
#define SIZE 500
void MainWindow::drawPoint(const Point& point,QColor color) {
    // Отрисовка точки
    scene->addEllipse(point.coords[0] - 2, -point.coords[1] - 2, 4, 4, QPen(color), QBrush(color)); // Смещение по Y для корректного отображения
}

void MainWindow::drawAxes() {
    // Отрисовка осей
    scene->addLine(-250, 0, 250, 0, QPen(Qt::black, 2)); // Ось X
    scene->addLine(0, -250, 0, 250, QPen(Qt::black, 2)); // Ось Y
}
void MainWindow::printStrelka(Point p1,Point p2,QColor color)
{
    float x1,y1,x2,y2;

    x1 = p1.coords[0];
    y1 = p1.coords[1];
    x2 = p2.coords[0];
    y2 = p2.coords[1];
    float x, y;
    float f1x2 , f1y2;
    float lons, ugol;

    const float ostr = 0.25;        // острота стрелки

    scene->addLine(x1, y1, x2, y2, QPen(color, 2));

    x = x2 - x1;
    y = y2 - y1;

    lons = sqrt(x*x + y*y) / 7;     // длина лепестков % от длины стрелки
    ugol = atan2(y, x);             // угол наклона линии

    //lons = 12;

    f1x2 = x2 - lons * cos(ugol - ostr);
    f1y2 = y2 - lons * sin(ugol - ostr);

    scene->addLine(x2, y2, f1x2, f1y2, QPen(color, 2));

    f1x2 = x2 - lons * cos(ugol + ostr);
    f1y2 = y2 - lons * sin(ugol + ostr);

    scene->addLine(x2, y2, f1x2, f1y2, QPen(color, 2));
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    scene->addRect(0,0,SIZE,SIZE,QPen(Qt::yellow, 3));
    scene->addRect(-SIZE,0,SIZE,SIZE,QPen(Qt::yellow, 3));
    scene->addRect(-SIZE,-SIZE,SIZE,SIZE,QPen(Qt::yellow, 3));
    scene->addRect(0,-SIZE,SIZE,SIZE,QPen(Qt::yellow, 3));
}

MainWindow::~MainWindow()
{
    delete ui;
}

