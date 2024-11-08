#include "mainwindow.h" // 4 variant

#include <QApplication>
#include <QPen>
#include <iostream>
#include <vector>
#include <functional>
#include <stdexcept>
#define MULT 2
#define MULT2 20
#define INIT_ALPHA 0.1 // Снижен начальный шаг
#define H 1e-8        // Точность для вычисления градиента
#define TOL 1e-8      // Порог для остановки минимизации
#define ITERATIONS 100000
using namespace std;

class MathFunction {
public:

    function<double(const vector<double>&)> func;

    MathFunction(function<double(const vector<double>&)> f) : func(f){}

    double evaluate(const vector<double> args) const {
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


    static Point computeGradient(const MathFunction& func, const Point point, double h = H) {
        vector<double> grad_coords(point.coords.size());

        for (unsigned int i = 0; i < point.coords.size(); ++i) {
            grad_coords[i] = computePartialDerivative(func, point.coords, i, h);
        }

        return Point(grad_coords);
    }

    static double lineSearch(const MathFunction& func, const Point& point, const Point& direction, double tol = 1e-6, double initialAlpha = INIT_ALPHA) {
        double alpha = initialAlpha;
        double stepSizeReduction = 0.5; // Коэффициент уменьшения шага
        double minAlpha = tol; // Минимальный шаг, при котором остановится поиск
        double bestAlpha = 0.0;
        double bestValue = func.evaluate(point.coords); // Начальное значение функции

        while (alpha > minAlpha) {
            Point testPoint = point + (direction * alpha); // Новая точка в направлении direction с шагом alpha
            double testValue = func.evaluate(testPoint.coords); // Вычисляем значение функции в новой точке

            if (testValue < bestValue) {
                bestValue = testValue;
                bestAlpha = alpha; // Обновляем лучший найденный шаг
            } else {
                // Уменьшаем шаг, так как текущий не улучшает значение функции
                alpha *= stepSizeReduction;
            }
        }

        return bestAlpha; // Возвращаем лучший найденный шаг
    }

    static Point minWithConjugateGradient(const MathFunction func, Point point,double h = H, int maxIter = ITERATIONS) {
        Point grad = computeGradient(func, point); // Градиент теперь Point
        Point d = grad;
        double beta, gradNorm;

        swapCoords(d); // Начальное направление

        for (int k = 0; k < maxIter; k++) {
            // Поиск шага
            double alpha = lineSearch(func, point, d);  // шаг
            Point pointNew = point + (d * alpha);

            // Вычисляем новый градиент и проверяем условие остановки
            Point gradNew = computeGradient(func, pointNew);
            gradNorm = sqrt(gradNew.coords[0] * gradNew.coords[0] + gradNew.coords[1] * gradNew.coords[1]);

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

    static void swapCoords(Point& d) {
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
        return (-1 / ((x-3)*(x-3)*(y-4)*(y-4) + 10*(x-5)*(x-5) + (y-4)*(y-4) + 1));
        }
    ); // реализация 2й

int main(int argc, char* argv[]) {
    QApplication a(argc, argv);
    MainWindow w;

    // Первое минимизирование
    cout << "Starting minimization of the first function..." << endl;
    points.clear();
    Point startPoint1{-150, -100};

    // Выполнение минимизации для первой функции
    Point minPoint1 = conjugateGradient::minWithConjugateGradient(conjugateGradient::firstMathFunc, startPoint1);
    if (!points.empty()) {
        cout << "First function minimized." << endl;

        // Вывод результатов и визуализация
        for(size_t i = 0; i < points.size() - 1; ++i) {
            w.printStrelka(points[i] * MULT, points[i + 1] * MULT, Qt::blue);
        }

        // Итоговый результат минимизации первой функции
        double resultFirst = conjugateGradient::firstMathFunc.evaluate(points.back().coords);
        cout << "Minimum of the first function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultFirst << endl;
    } else {
        cout << "Error: No points found during minimization of the first function." << endl;
    }

    // Второе минимизирование
    cout << "\nStarting minimization of the second function..." << endl;
    points.clear();
    Point startPoint2{0,0};

    // Выполнение минимизации для второй функции
    Point minPoint2 = conjugateGradient::minWithConjugateGradient(conjugateGradient::secondMathFunc, startPoint2);
    if (!points.empty()) {
        cout << "Second function minimized." << endl;
        // Вывод результатов и визуализация
        for (size_t i = 0; i < points.size() - 1; ++i) {
            w.printStrelka(points[i] * MULT2, points[i + 1] * MULT2, Qt::red);
        }

        // Итоговый результат минимизации второй функции
        double resultSecond = conjugateGradient::secondMathFunc.evaluate(points.back().coords);
        cout << "Minimum of the second function at point ("
             << points.back().coords[0] << ", "
             << points.back().coords[1] << ") := "
             << resultSecond << endl;
    } else {
        cout << "Error: No points found during minimization of the second function." << endl;
    }

    // Показ окна
    w.show();
    return a.exec();
}
