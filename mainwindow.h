#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <vector>

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

    void drawPoint(const Point& point,QColor color);
    void drawAxes();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
};
#endif // MAINWINDOW_H
