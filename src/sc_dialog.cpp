#include "../include/launchgui_yj/sc_dialog.hpp"
#include "ui_sc_dialog.h"
#include <QtGui/QApplication>
#include <QLabel>

extern QImage qt_image_screenshot;

Sc_Dialog::Sc_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Sc_Dialog)
{
    ui->setupUi(this);
    //QObject::connect(&qnode, SIGNAL(statusUpdated_sc()), this, SLOT(updateState_sc()));
}

Sc_Dialog::~Sc_Dialog()
{
    delete ui;
}

void Sc_Dialog::show_screenshot() {
    ui->label_sc->setPixmap(QPixmap::fromImage(qt_image_screenshot));
    ui->label_sc->resize(ui->label_sc->pixmap()->size());
}
