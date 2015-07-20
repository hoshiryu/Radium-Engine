#include <Core/CoreMacros.hpp>

#if defined (OS_WINDOWS)
    #include <GL/glew.h>
#endif

#include <iostream>
#include <QApplication>

#include <MainApplication/Gui/MainWindow.hpp>



int main(int argc, char** argv)
{
    std::cerr<<"*** Radium Engine Main App  ***"<<std::endl;
#if defined (CORE_DEBUG)
    std::cerr<<"(Debug Build)"<<std::endl;
 #else
    std::cerr<<"(Release Build)"<<std::endl;
#endif
    std::cerr<<"Floating point format : ";
#if defined(CORE_USE_DOUBLE)
    std::cerr<<"double precision"<<std::endl;
#else
    std::cerr<<"single precision"<<std::endl;
#endif

#if defined (OS_WINDOWS)
    glewExperimental = GL_TRUE;
    glewInit();
#endif

    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setVersion(4, 4);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(0);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(16);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);

    Ra::Gui::MainWindow w;
    w.show();
    return app.exec();
}
