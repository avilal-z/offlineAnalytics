#if defined (_MSC_VER)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "segmentviewer.h"

int main(int argc, char* argv[])
{
	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication a(argc, argv);

	// a.setStyle(new DarkStyle);

	segmentViewer w;
	w.show();
	// w.setWindowState(Qt::WindowMaximized);
	return a.exec();
}