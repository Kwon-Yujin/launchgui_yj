#ifndef SC_DIALOG_H
#define SC_DIALOG_H

#include <QDialog>

namespace Ui {
    class Sc_Dialog;
}

class Sc_Dialog : public QDialog
{
    Q_OBJECT
public:
    explicit Sc_Dialog(QWidget *parent = nullptr);
    ~Sc_Dialog();
    void show_screenshot();

public Q_SLOTS:
    //void updateState_sc();//add

private:
    //launchgui::QNode q_node;
    Ui::Sc_Dialog *ui;
};

#endif // SC_DIALOG_H
