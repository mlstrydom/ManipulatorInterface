#include "screengraphics.h"
#include "ui_screengraphics.h"

screenGraphics::screenGraphics(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::screenGraphics)
{
    ui->setupUi(this);
}

screenGraphics::~screenGraphics()
{
    delete ui;
}
