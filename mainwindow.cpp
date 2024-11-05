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

