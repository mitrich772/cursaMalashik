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
    Point(const vector<double>& coords_vec = {0, 0})  : coords(coords_vec) {}

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
    QtCharts::QLineSeries *makeSeries(vector<Point> points);
    void addSeriesToChart(QtCharts::QLineSeries* series);
    void on_pushButtonCalculate_clicked();
    void on_pushButtonClear_clicked();
    void on_horizontalSlide_moved();
    void on_verticalSlide_moved();
    void on_chartContainerClicked(QPoint &pos);
    void calculateAll(double h_value,double initialAlpha, double stepReduce);
    void clear();
    void setupAxes();

    QtCharts::QLineSeries *series1;
    QtCharts::QLineSeries *series2;
    QtCharts::QChart *chart;
    Point startPoint1;
    Point startPoint2;
    double h_value;
    double alpha_value;
    double stepReduce;
    double x;
    double y;
    Ui::MainWindow *ui;
    inline static QPoint coords = QPoint(0,0);
private:
    QGraphicsScene *scene;

};
#endif // MAINWINDOW_H
