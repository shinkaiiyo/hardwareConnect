#include <QCoreApplication>
#include "lhardwareconnect.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    LhardwareConnect *connect = new LhardwareConnect();
    connect->createConnection();
    return a.exec();
}
