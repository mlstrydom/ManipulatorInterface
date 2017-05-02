#ifndef SCREENGRAPHICS_H
#define SCREENGRAPHICS_H

#include <QMainWindow>

namespace Ui {
class screenGraphics;
}

class screenGraphics : public QMainWindow
{
    Q_OBJECT

public:
    explicit screenGraphics(QWidget *parent = 0);
    ~screenGraphics();

private:
    Ui::screenGraphics *ui;
};

#endif // SCREENGRAPHICS_H
