#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <vector>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>


using namespace std;

struct Point {
    vector<double> coords;

    Point(initializer_list<double> coords_list) : coords(coords_list) {}
    Point(const vector<double>& coords_vec) : coords(coords_vec) {}

    Point operator+(const Point& other) const { // Сложение векторов
        Point result = *this;
        for (size_t i = 0; i < coords.size(); ++i) {
            result.coords[i] += other.coords[i];
        }
        return result;
    }

    Point operator*(double scalar) const { // Умножение на скаляр
        Point result = *this;
        for (double& val : result.coords) {
            val *= scalar;
        }
        return result;
    }
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void addSeriesToChart(QtCharts::QLineSeries* series);
    QtCharts::QLineSeries *makeSeries(vector<Point> points);
    void on_pushButtonCalculate_clicked();
    void on_pushButtonClear_clicked();
    void calculateAll(double h_value,double initialAlpha);
    void clear();

    QtCharts::QLineSeries *series1;
    QtCharts::QLineSeries *series2;
    QtCharts::QChart *chart;
private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;

};
#endif // MAINWINDOW_H
