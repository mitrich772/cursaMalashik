#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include<cmath>
#include <QPen>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <iostream>
#include <QVBoxLayout>
#define SIZE 500
#define MULT 2
#define MULT2 2
#define INIT_ALPHA 100
#define H 1e-8
#define TOL 1e-8
#define ITERATIONS 10000
#define N 2
using namespace std;

class MathFunction {
public:

    function<double(const vector<double>&)> func;

    MathFunction(function<double(const vector<double>&)> f) : func(f){}

    double evaluate(const vector<double> args) const {
        return func(args);
    }
    double evaluate(const Point args) const {
        return func(args.coords);
    }
};

static vector<Point> points;

MathFunction firstMathFunc(
    [](const std::vector<double>& args) -> double {
        if (args.size() != 2) {
            throw std::invalid_argument("Function requires exactly 2 arguments.");
        }
        return (args[0] - 3) * (args[0] - 3) + 50 * (args[1] - 5) * (args[1] - 5);
    }
    );

MathFunction secondMathFunc(
    [](const std::vector<double>& args) -> double {
        double x = args[0];
        double y = args[1];
        if (args.size() != 2) {
            throw std::invalid_argument("Function requires exactly 2 arguments.");
        }
        return (-1.0 / ((x - 3) * (x - 3) * (y - 4) * (y - 4) + 10 * (x - 5) * (x - 5) + (y - 4) * (y - 4) + 1));
    }
    );

double lineSearch(const MathFunction& func, const Point& point, const Point& direction, double initialAlpha = INIT_ALPHA,double stepReduce = 0.5 ,double tol = 1e-6) {
    double alpha = initialAlpha;
    double stepSizeReduction = stepReduce;
    double minAlpha = tol;
    double bestAlpha = 0.0;
    double bestValue = func.evaluate(point.coords);

    while (alpha > minAlpha) {
        Point testPoint = point + (direction * alpha);
        double testValue = func.evaluate(testPoint.coords);

        if (testValue < bestValue) {
            bestValue = testValue;
            bestAlpha = alpha;
        } else {
            alpha *= stepSizeReduction;
        }
    }
    return bestAlpha;
}


double computePartialDerivative(const MathFunction& func, const std::vector<double>& coords, unsigned int index, double h) {
    std::vector<double> pointPlusH = coords;
    std::vector<double> pointMinusH = coords;

    pointPlusH[index] += h;
    pointMinusH[index] -= h;

    return (func.evaluate(pointPlusH) - func.evaluate(pointMinusH)) / (2 * h);
}


Point computeGradient(const MathFunction& func, const Point& point, double h = H) {
    std::vector<double> grad_coords(point.coords.size());

    for (unsigned int i = 0; i < point.coords.size(); ++i) {
        grad_coords[i] = computePartialDerivative(func, point.coords, i, h);
    }

    return Point(grad_coords);
}


double findBeta(const Point& grad, const Point& gradNew, size_t iteration) {
    return (iteration % (N + 1) == 0) ? 0 :
               (gradNew.coords[0] * gradNew.coords[0] + gradNew.coords[1] * gradNew.coords[1]) /
                   (grad.coords[0] * grad.coords[0] + grad.coords[1] * grad.coords[1]);
}


Point minWithConjugateGradient(const MathFunction& func, Point point, double h = H, double initialAlpha = INIT_ALPHA,double stepReduce = 0.5 ,int maxIter = ITERATIONS) {
    Point grad = computeGradient(func, point);
    Point d = grad * -1;
    double beta, gradNorm;

    for (size_t k = 0; k < maxIter; k++) {
        if (k == maxIter - 1) {
            std::cout << "MaxIter reached" << std::endl;
        }

        double alpha = lineSearch(func, point, d, initialAlpha, stepReduce);
        Point pointNew = point + (d * alpha);

        Point gradNew = computeGradient(func, pointNew);
        gradNorm = sqrt(gradNew.coords[0] * gradNew.coords[0] + gradNew.coords[1] * gradNew.coords[1]);

        if (gradNorm < h) {
            return pointNew;
        }

        beta = findBeta(grad, gradNew, k);
        d = (gradNew * -1) + (d * beta);
        points.push_back(point);
        point = pointNew;
        grad = gradNew;
    }

    return point;
}

QtCharts::QLineSeries* MainWindow::makeSeries(vector<Point> points = {}){
    QtCharts::QLineSeries *series = new QtCharts::QLineSeries();
    for(Point point : points){
        series->append(point.coords[0],point.coords[1]);
    }
    return series;
}

void MainWindow::calculateAll(double h_value = H,double initialAlpha = INIT_ALPHA, double stepReduce = 0.5){
    QtCharts::QLineSeries *series;

    cout << "Starting minimization of the first function..." << endl;
    points.clear();
    Point startPoint1{-100, -10};


    Point minPoint1 = minWithConjugateGradient(firstMathFunc, startPoint1, h_value, initialAlpha,stepReduce);
    if (!points.empty()) {
        cout << "First function minimized." << endl;


        series = makeSeries(points);
        series1 = series;
        addSeriesToChart(series);



        double resultFirst = firstMathFunc.evaluate(points.back().coords);

        cout << "Minimum of the first function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultFirst << endl;
    } else {
        cout << "Error: No points found during minimization of the first function." << endl;
    }
    chart->createDefaultAxes();
    // Вт
    cout << "\nStarting minimization of the second function..." << endl;
    points.clear();
    Point startPoint2{-15,-5};

    Point minPoint2 = minWithConjugateGradient(secondMathFunc, startPoint2, h_value, initialAlpha,stepReduce);
    if (!points.empty()) {


        series = makeSeries(points);
        series2 = series;
        addSeriesToChart(series);

        cout << initialAlpha <<endl;
        cout << "Second function minimized." << endl;
        double resultSecond = secondMathFunc.evaluate(points.back().coords);
        cout << "Minimum of the second function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultSecond << endl;
    } else {
        cout << "Error: No points found during minimization of the second function." << endl;
    }
    chart->createDefaultAxes();
}

void MainWindow::addSeriesToChart(QtCharts::QLineSeries* series){
    chart->addSeries(series);
}

void MainWindow::clear(){
    chart->removeAllSeries();
    chart->addSeries(makeSeries());
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    scene = new QGraphicsScene();
    chart = new QtCharts::QChart();
    chart->legend()->hide();
    chart->setTitle("график");

    QtCharts::QChartView *chartView = new QtCharts::QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QVBoxLayout *layout = new QVBoxLayout(ui->chartContainer);
    ui->chartContainer->setLayout(layout);
    ui->chartContainer->layout()->addWidget(chartView);
    connect(ui->pushButtonCalculate, &QPushButton::clicked, this, &MainWindow::on_pushButtonCalculate_clicked); //подключение кнопки
    connect(ui->pushButtonClear, &QPushButton::clicked, this, &MainWindow::on_pushButtonClear_clicked);
}
void MainWindow::on_pushButtonCalculate_clicked(){
    // Получаем значение из spinBoxNumber
    double h_value = ui->doubleSpinBox_h->value();
    double alpha_value = ui->doubleSpinBox_Alpha->value();
    double stepReduce = ui->doubleSpinBox_AlphaRedution->value();
    cout<<"accur:="<< h_value << endl;
    cout<<"alpha:="<< alpha_value << endl;
    if(h_value == 0 || alpha_value == 0){
        calculateAll();
    }else{
        calculateAll(h_value,alpha_value,stepReduce);
    }
}

void MainWindow::on_pushButtonClear_clicked(){
    clear();
    chart->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}

