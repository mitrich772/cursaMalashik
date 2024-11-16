#ifndef CUSTOMWIDGET_H
#define CUSTOMWIDGET_H

#include <QWidget>
#include <QMouseEvent>
#include <QDebug>

class CustomWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CustomWidget(QWidget *parent = nullptr);
protected:
    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            QPoint coords = event->pos();
            emit mouseClicked(coords);  // Генерируем сигнал
        }
    }
signals:
    void mouseClicked(QPoint &coords);  // Сигнал для передачи координат
};

#endif // CUSTOMWIDGET_H
