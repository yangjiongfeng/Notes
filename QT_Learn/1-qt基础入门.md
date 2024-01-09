参考链接：[Qt学习之Qt基础入门(上)_qt入门-CSDN博客](https://blog.csdn.net/RongLin02/article/details/120595809?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-120595809-blog-125241375.235%5Ev39%5Epc_relevant_3m_sort_dl_base4&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-120595809-blog-125241375.235%5Ev39%5Epc_relevant_3m_sort_dl_base4&utm_relevant_index=3)
## 4.2.1. .pro文件
pro文件相当于一个配置清单
QT       += core gui   Qt包含的模块

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets  //大于4版本以上 包含 widget模块

TARGET = MyTest  //目标   生成的.exe程序的名称
TEMPLATE = app       	  //模板   应用程序模板  Application


SOURCES += \                //源文件
        main.cpp \
        mainwindow.cpp

HEADERS += \                //头文件
        mainwindow.h

FORMS += \                    //ui文件界面
        mainwindow.ui

## 4.2.2. main.cpp

    QApplication a(argc, argv);    //应用程序对象，有且仅有一个
    MainWindow w;    //实例化窗口对象
    w.show();    //调用show函数 显示窗口

    return a.exec();    //让应用程序对象进入消息循环机制中，代码阻塞到当前行

#### 4.2.3. mainwindow.cpp

这个就是用来书写逻辑代码的主要地方，和`mainwindow.h`是一起的

#### 4.2.4. mainwindow.ui

ui文件，可视化界面拖动，可以做界面布局，很方便
