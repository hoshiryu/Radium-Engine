#include <QApplication>
#include <QTimer>

#include <QOpenGLContext>

#include <GuiBase/Viewer/Viewer.hpp>
#include <GuiBase/TimerData/FrameTimerData.hpp>

#include <Engine/Managers/EntityManager/EntityManager.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderConfigFactory.hpp>

#include <minimalradium.hpp>
/* This file contains a minimal radium/qt application which shows the
classic "Spinning Cube" demo. */
#include <minimalapp.hpp>

extern std::string EXE_PATH;

int main(int argc, char* argv[])
{
    EXE_PATH = QCoreApplication::applicationDirPath().toStdString() + "/";

    // Create default format for Qt.
    QSurfaceFormat format;
    format.setVersion( 4, 4 );
    format.setProfile( QSurfaceFormat::CoreProfile );
    format.setDepthBufferSize( 24 );
    format.setStencilBufferSize( 8 );
    //format.setSamples( 16 );
    format.setSwapBehavior( QSurfaceFormat::DoubleBuffer );
    format.setSwapInterval( 0 );
    QSurfaceFormat::setDefaultFormat( format );

    // Create app and show viewer window
    MinimalApp app(argc, argv);
    app.m_viewer->show();
    CORE_ASSERT( app.m_viewer->getContext()->isValid(), "OpenGL was not initialized" );
    // process all events so that everithing is initialized
    QApplication::processEvents();

    // Create one system
    Ra::Engine::System* sys = new MinimalSystem;
    app.m_engine->registerSystem("Minimal system", sys);

    // Create and initialize entity and component
    Ra::Engine::Entity* e = app.m_engine->getEntityManager()->createEntity("Cube");
    Ra::Engine::Component* c = new MinimalComponent;
    e->addComponent(c);
    sys->registerComponent(e, c);
    c->initialize();

    // Start the app.
    app.m_frame_timer->start();
    return app.exec();
}
