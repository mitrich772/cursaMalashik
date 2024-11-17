#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include<cmath>
#include <QPen>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <iostream>
#include <QVBoxLayout>
#include<QWidget>
#include"customwidget.h"
#define SIZE 500
#define MULT 2
#define MULT2 2
#define INIT_ALPHA 100
#define H 1e-8
#define TOL 1e-8
#define ITERATIONS 100000
#define N 2
using namespace std;


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
void MainWindow::saveResultsToFile(const QString& fileName, const Point& result, MathFunction func, int iterations) {
    QString path = "C:/Users/dima/Documents/cursaMalashik/" + fileName; // Путь к файлу
    QFile file(path);
    if (file.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        out << "Result: (" << result.coords[0] << ", " << result.coords[1] << ")" << ":= "<< func.evaluate(result) << "\n";
        out << "Iterations: " << iterations << "\n";
        file.close();
        qDebug() << "Результаты успешно сохранены в файл:" << path;
    } else {
        qDebug() << "Ошибка при открытии файла:" << path;
    }
}

// Функция создания файла
void MainWindow::createFile(const QString &fileName) {
    QString directory = "C:/Users/dima/Documents/cursaMalashik/";
    QDir dir(directory);

    // Создаем папку, если она не существует
    if (!dir.exists()) {
        dir.mkpath(directory);
    }

    QFile file(dir.filePath(fileName));
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        file.close();
        qDebug() << "Файл создан: " << dir.filePath(fileName);
    } else {
        qDebug() << "Ошибка при создании файла: " << file.errorString();
    }
}
void MainWindow::clearFile(const QString &filePath) {
    QFile file(filePath);

    // Открываем файл в режиме записи (WriteOnly), чтобы очистить его содержимое
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        file.resize(0); // Убедимся, что файл действительно очищен
        file.close();
        qDebug() << "Файл успешно очищен:" << filePath;
    } else {
        qDebug() << "Не удалось открыть файл для очистки:" << file.errorString();
    }
}

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
double FindMin(const MathFunction& f, const Point& start, const Point& direction, double step = 0.1, double tol = 1e-4) {
    double alpha = 0.0;
    double bestAlpha = 0.0;
    double fx = f.evaluate(start);
    double bestFx = fx;

    while (true) {
        alpha += step;
        Point x = start + direction * alpha;
        double newFx = f.evaluate(x);

        // Если новое значение меньше, обновляем лучшую точку
        if (newFx < bestFx) {
            bestFx = newFx;
            bestAlpha = alpha;
        } else {
            // Если значение увеличивается, уменьшаем шаг и пробуем снова
            step *= 0.5;
            if (step < tol) break;
            alpha -= step;  // Возвращаемся назад
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


Point MainWindow::minWithConjugateGradient(const MathFunction& func, Point point, double h = H, double initialAlpha = INIT_ALPHA,double stepReduce = 0.5 ,int maxIter = ITERATIONS) {
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
            cout << "Iterations:=" << k << endl;
            //clearFile("result.txt");
            saveResultsToFile("result.txt", pointNew, func, k);
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

void MainWindow::calculateAll(double h_value = H,double initialAlpha = INIT_ALPHA, double stepReduce = 0.5){ // Дальше бога нет
    QtCharts::QLineSeries *series;
    clearFile("result.txt");
    cout << "Starting minimization of the first function..." << endl;
    points.clear();


    Point minPoint1 = minWithConjugateGradient(firstMathFunc, startPoint1, h_value, initialAlpha,stepReduce);
    if (!points.empty()) {
        cout << "First function minimized." << endl;


        series = makeSeries(points);
        series1 = series;
        addSeriesToChart(series);



        double resultFirst = firstMathFunc.evaluate(minPoint1);

        cout << "Minimum of the first function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultFirst << endl;
    } else {
        cout << "Error: No points found during minimization of the first function." << endl;
        makeSeries();
    }
    chart->createDefaultAxes();
    // Вт
    cout << "\nStarting minimization of the second function..." << endl;
    points.clear();
    //startPoint2{-15,-5};

    Point minPoint2 = minWithConjugateGradient(secondMathFunc, startPoint2, h_value, initialAlpha,stepReduce);
    if (!points.empty()) {


        series = makeSeries(points);
        series2 = series;
        addSeriesToChart(series);

        cout << "Second function minimized." << endl;
        double resultSecond = secondMathFunc.evaluate(minPoint2);
        cout << "Minimum of the second function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultSecond << endl;
    } else {
        cout << "Error: No points found during minimization of the second function." << endl;
        series2 = makeSeries();
    }
    chart->createDefaultAxes();
    setupAxes();
}

void MainWindow::addSeriesToChart(QtCharts::QLineSeries* series){
    chart->addSeries(series);
}

void MainWindow::clear() {
    for (auto series : chart->series()) {
        // Преобразуем текущую серию в QLineSeries, чтобы очистить точки
        auto lineSeries = qobject_cast<QtCharts::QLineSeries*>(series);
        if (lineSeries) {
            lineSeries->clear(); // Удаляем все точки из серии
        }
    }
    chart->update(); // Обновляем график
}

void MainWindow::setupAxes() {
    // Создаем оси

    QtCharts::QValueAxis *axisX = new QtCharts::QValueAxis();
    QtCharts::QValueAxis *axisY = new QtCharts::QValueAxis();

    // Устанавливаем фиксированные диапазоны для осей
    axisX->setRange(-50, 50);  // Пример: ось X от -10 до 10
    axisY->setRange(-50, 50);  // Пример: ось Y от -10 до 10

    QList<QtCharts::QAbstractAxis*> oldAxes = chart->axes();
    for (QtCharts::QAbstractAxis* axis : oldAxes) {
        chart->removeAxis(axis);
    }
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);

    // Привязываем серии к осям
    if (series1->count() != 0) {
        series1->attachAxis(axisX);
        series1->attachAxis(axisY);
    }
    if (series2->count() != 0) {
        series2->attachAxis(axisX);
        series2->attachAxis(axisY);
    }
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    createFile("result.txt");
    // Создаём новый CustomWidget
    CustomWidget *customContainer = new CustomWidget(this);
    customContainer->setGeometry(ui->chartContainer->geometry());

    // Переносим layout из старого chartContainer
//    QLayout *layout = ui->chartContainer->layout();
//    ui->chartContainer->setLayout(nullptr); // Отвязываем layout от старого контейнера
//    customContainer->setLayout(layout);

    // Заменяем старый chartContainer на новый
    delete ui->chartContainer;
    ui->chartContainer = customContainer;

    // Подключаем сигнал для кликов мыши


    h_value = ui->doubleSpinBox_h->value();
    alpha_value = ui->doubleSpinBox_Alpha->value();
    stepReduce = ui->doubleSpinBox_AlphaRedution->value();

    scene = new QGraphicsScene();
    chart = new QtCharts::QChart();
    chart->legend()->hide();


    QtCharts::QChartView *chartView = new QtCharts::QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QVBoxLayout *layout = new QVBoxLayout(ui->chartContainer);
    ui->chartContainer->setLayout(layout);
    ui->chartContainer->layout()->addWidget(chartView);

    connect(ui->pushButtonCalculate, &QPushButton::clicked, this, &MainWindow::on_pushButtonCalculate_clicked); //подключение кнопки
    connect(ui->pushButtonClear, &QPushButton::clicked, this, &MainWindow::on_pushButtonClear_clicked);
    connect(ui->horizontalSlider, &QSlider::sliderMoved, this, &MainWindow::on_horizontalSlide_moved);
    connect(ui->verticalSlider, &QSlider::sliderMoved, this, &MainWindow::on_verticalSlide_moved);
    connect(customContainer, &CustomWidget::mouseClicked, this, &MainWindow::on_chartContainerClicked);


}

void MainWindow::on_chartContainerClicked(QPoint &pos) {
    if(chart->axes().length() != 0){
        MainWindow::setupAxes();
    }
    pos -= QPoint(71,45); //777|697

    x = (pos.rx() / 7.77) - 50;
    y = -((pos.ry() / 6.97) - 50);
    cout<<x<<"|"<<y<<endl;
    startPoint1 = Point{x,y};
    startPoint2 = Point{x,y};

    ui->doubleSpinBox_Y->setValue(y);
    ui->doubleSpinBox_X->setValue(x);

    clear();
    calculateAll(h_value,alpha_value,stepReduce);
}
void MainWindow::on_pushButtonCalculate_clicked(){
    h_value = ui->doubleSpinBox_h->value();
    alpha_value = ui->doubleSpinBox_Alpha->value();
    stepReduce = ui->doubleSpinBox_AlphaRedution->value();
    x = ui->horizontalSlider->value();
    y = ui->verticalSlider->value();

    startPoint1 = Point{x,y};
    startPoint2 = Point{x,y};

    calculateAll(h_value,alpha_value,stepReduce);

}

void MainWindow::on_pushButtonClear_clicked(){
    clear();
}
void MainWindow::on_horizontalSlide_moved(){
    clear();
    chart->update();

    x = ui->horizontalSlider->value();
    //y = ui->verticalSlider->value();

    startPoint1 = Point{x,y};
    startPoint2 = Point{x,y};

    ui->doubleSpinBox_X->setValue(x);
    calculateAll(h_value,alpha_value,stepReduce);
}
void MainWindow::on_verticalSlide_moved(){
    clear();
    chart->update();

    //x = ui->horizontalSlider->value();
    y = ui->verticalSlider->value();

    startPoint1 = Point{x,y};
    startPoint2 = Point{x,y};

    ui->doubleSpinBox_Y->setValue(y);
    calculateAll(h_value,alpha_value,stepReduce);
}

MainWindow::~MainWindow()
{
    delete ui;
}

