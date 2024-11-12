#include "mainwindow.h" // 4 variant
#include<cmath>
#include <QApplication>
#include <QPen>
#include <iostream>
#include <vector>
#include <functional>
#include <stdexcept>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
int main(int argc, char* argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
