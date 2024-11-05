#include "mainwindow.h" // 4 variant

#include <QApplication>
#include <QPen>
#include <iostream>
#include <vector>
#include <functional>
#include <stdexcept>
using namespace std;

class MathFunction {
public:

    function<double(const vector<double>&)> func;

    MathFunction(function<double(const vector<double>&)> f) : func(f){}

    double evaluate(const vector<double>& args) const {
        return func(args);
    }
};
//struct Point {
//    vector<double> coords;

//    Point(initializer_list<double> coords_list) : coords(coords_list) {}
//    Point(const vector<double>& coords_vec) : coords(coords_vec) {}

//    Point operator+(const Point& other) const { // сложение векторов
//        Point result = *this;
//        for (size_t i = 0; i < coords.size(); ++i) {
//            result.coords[i] += other.coords[i];
//        }
//        return result;
//    }

//    Point operator*(double scalar) const { // умнож на скаляр
//        Point result = *this;
//        for (double& val : result.coords) {
//            val *= scalar;
//        }
//        return result;
//    }
//};
static vector<Point> points;

class conjugateGradient{ //x0 -> R n-ое (x1,x2,x3)
public:

    static MathFunction firstMathFunc;
    static MathFunction secondMathFunc;


    static Point computeGradient(const MathFunction& func, const Point point, double h = 1e-6) {
        vector<double> grad_coords(point.coords.size());

        for (unsigned int i = 0; i < point.coords.size(); ++i) {
            grad_coords[i] = computePartialDerivative(func, point.coords, i, h);
        }

        return Point(grad_coords);
    }

    static Point minWithConjugateGradient(const MathFunction func, Point point,double h = 1e-6, int maxIter = 1000) {
        Point grad = computeGradient(func, point); // Градиент теперь Point
        Point d = grad;
        double beta;

        swapCoords(d); // Начальное направление

        for (int k = 0; k < maxIter; ++k) {
            // Поиск шага
            double alpha = 0.001;  // Пример фиксированного шага
            Point pointNew = point + (d * alpha);

            // Вычисляем новый градиент и проверяем условие остановки
            Point gradNew = computeGradient(func, pointNew);
            double gradNorm = sqrt(gradNew.coords[0] * gradNew.coords[0] + gradNew.coords[1] * gradNew.coords[1]);

            if (gradNorm < h) {
                return pointNew;
            }

            // Коэффициент Бета
            beta = findBeta(grad,gradNew);

            // Обновляем направление с использованием перегруженных операторов
            d = (gradNew * -1) + (d * beta);
            points.push_back(point);
            // Обновляем точку и градиент
            point = pointNew;
            grad = gradNew;
        }
        return point; // Возвращаем последнюю точку
    }


private:

    static double computePartialDerivative(const MathFunction func, const vector<double>& coords, unsigned int index, double h) {
        vector<double> pointPlusH = coords;
        vector<double> pointMinusH = coords;

        pointPlusH[index] += h;
        pointMinusH[index] -= h;

        return (func.evaluate(pointPlusH) - func.evaluate(pointMinusH)) / (2 * h);
    }

    static double findBeta(Point grad, Point gradNew){
        return (gradNew.coords[0] * gradNew.coords[0] + gradNew.coords[1] * gradNew.coords[1]) /
               (grad.coords[0] * grad.coords[0] + grad.coords[1] * grad.coords[1]);
    }

    static void swapCoords(Point d) {
        for (double& v : d.coords) {
            v = -v;
        }
    }
};



MathFunction conjugateGradient::firstMathFunc = MathFunction(
    [](const vector<double>& args) -> double {
        if (args.size() != 2) {
            throw invalid_argument("Function requires exactly 2 arguments.");
        }
        return (args[0] - 3) * (args[0] - 3) + 50 * (args[1] - 5) * (args[1] - 5);
        }
    ); // реализация 1й
MathFunction conjugateGradient::secondMathFunc = MathFunction(
    [](const vector<double>& args) -> double {
        double x = args[0];
        double y = args[1];
        if (args.size() != 2) {
            throw invalid_argument("Function requires exactly 2 arguments.");
        }
        return (-1*((x-3)*(x-3)*(y-4)*(y-4)) + 10*((x-5)*(x-5)) + (y-4)*(y-4) + 1);
        }
    ); // реализация 2й

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);
    MainWindow w;
    conjugateGradient::minWithConjugateGradient(conjugateGradient::firstMathFunc,Point{12,24});
    for(Point p : points){
        w.drawPoint(p*10,Qt::blue);
    }
    points.clear();
    conjugateGradient::minWithConjugateGradient(conjugateGradient::secondMathFunc,Point{12,24});
    for(Point p : points){
        w.drawPoint(p*10,Qt::red);
    }

    w.show();
    return a.exec();
}
