#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;//создается без parent, ну то есть врехеуровневое окно
    w.show();

    return a.exec();
}
