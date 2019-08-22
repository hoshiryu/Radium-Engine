#include <CameraManipPlugin.hpp>

#include <fstream>

#include <QAction>
#include <QFileDialog>
#include <QSettings>

#include <Engine/ItemModel/ItemEntry.hpp>
#include <Engine/Managers/CameraManager/CameraManager.hpp>
#include <Engine/Managers/EntityManager/EntityManager.hpp>
#include <Engine/Renderer/Camera/Camera.hpp>

#include <GuiBase/SelectionManager/SelectionManager.hpp>
#include <GuiBase/Viewer/TrackballCamera.hpp>
#include <GuiBase/Viewer/Viewer.hpp>

#include "ui_CameraManipUI.h"
#include <CameraManipSystem.hpp>
#include <UI/CameraManipUI.h>

#include <Core/Animation/Interpolation.hpp>

namespace CameraManipPlugin {

using namespace Ra::Core::Utils; // log

CameraManipPluginC::CameraManipPluginC() :
    m_widget( nullptr ),
    m_system( nullptr ),
    m_engine( nullptr ),
    m_selectionManager( nullptr ),
    m_viewer( nullptr ) {}

CameraManipPluginC::~CameraManipPluginC() {}

void CameraManipPluginC::registerPlugin( const Ra::Plugins::Context& context ) {
    // register selection context
    m_engine           = context.m_engine;
    m_selectionManager = context.m_selectionManager;
    m_viewer           = context.m_viewer;
    connect( m_selectionManager,
             &Ra::GuiBase::SelectionManager::currentChanged,
             this,
             &CameraManipPluginC::onCurrentChanged );
    connect(
        this, &CameraManipPluginC::askForUpdate, &context, &Ra::Plugins::Context::askForUpdate );
    connect( this,
             &CameraManipPluginC::setContinuousUpdate,
             &context,
             &Ra::Plugins::Context::setContinuousUpdate );
    m_system = new CameraManipSystem( this );
    m_engine->registerSystem( "CameraManipSystem", m_system );
}

bool CameraManipPluginC::doAddWidget( QString& name ) {
    name = "CameraManip";
    return true;
}

QWidget* CameraManipPluginC::getWidget() {
    m_widget = new CameraManipUI();
    connect( m_widget->ui->m_useCamera,
             &QPushButton::clicked,
             this,
             &CameraManipPluginC::useSelectedCamera );
    connect(
        m_widget->ui->m_saveCamera, &QPushButton::clicked, this, &CameraManipPluginC::saveCamera );
    connect( m_widget->ui->m_createCamera,
             &QPushButton::clicked,
             this,
             &CameraManipPluginC::createCamera );
    connect( m_widget->ui->m_addKeyFrame,
             &QPushButton::clicked,
             this,
             &CameraManipPluginC::addKeyFrame );
    connect(
        m_widget->ui->m_showFrames, &QPushButton::clicked, this, &CameraManipPluginC::showFrames );
    connect( m_widget->ui->m_playPath, &QPushButton::toggled, this, &CameraManipPluginC::playPath );
    connect(
        m_widget->ui->m_resetPath, &QPushButton::clicked, this, &CameraManipPluginC::resetPath );
    connect( m_widget->ui->m_savePath, &QPushButton::clicked, this, &CameraManipPluginC::savePath );
    connect( m_widget->ui->m_loadPath, &QPushButton::clicked, this, &CameraManipPluginC::loadPath );
    return m_widget;
}

bool CameraManipPluginC::doAddMenu() {
    return false;
}

QMenu* CameraManipPluginC::getMenu() {
    return nullptr;
}

bool CameraManipPluginC::doAddAction( int& nb ) {
    nb = 0;
    return false;
}

QAction* CameraManipPluginC::getAction( int id ) {
    return nullptr;
}

void CameraManipPluginC::useSelectedCamera() {
    if ( m_selectionManager->hasSelection() )
    {
        const Ra::Engine::ItemEntry& ent = m_selectionManager->currentItem();
        if ( ent.m_component == nullptr ) { return; }
        if ( ent.m_component->getName().compare( 0, 7, "CAMERA_" ) == 0 )
        {
            Ra::Engine::Camera* camera = static_cast<Ra::Engine::Camera*>( ent.m_component );
            m_viewer->getCameraInterface()->getCamera()->show( true );
            m_viewer->getCameraInterface()->setCamera( camera );
        }
    }
}

void CameraManipPluginC::saveCamera() {
    QSettings settings;
    QString path = settings.value( "CameraManip::camera_file", QDir::homePath() ).toString();
    path         = QFileDialog::getSaveFileName( nullptr, "Open Camera", path, "*.cam" );
    if ( path.size() == 0 ) { return; }
    settings.setValue( "CameraManip::camera_file", path );

    std::ofstream outFile( path.toStdString() );
    if ( !outFile.is_open() )
    {
        LOG( logWARNING ) << "Could not open file to save the camera: " << path.toStdString();
        return;
    }

    auto manip  = static_cast<Ra::Gui::TrackballCamera*>( m_viewer->getCameraInterface() );
    auto camera = manip->getCamera();
    outFile << "#Radium_camera_state" << std::endl;
    outFile << (int)camera->getType() << std::endl;
    outFile << camera->getFrame().matrix() << std::endl;
    outFile << std::endl;
    outFile << camera->getFOV() << " " << camera->getZNear() << " " << camera->getZFar() << " "
            << camera->getZoomFactor() << " " << camera->getAspect() << std::endl;
    outFile << std::endl;
}

void CameraManipPluginC::createCamera() {
    // Create new entity with camera component only
    auto camMngr =
        static_cast<Ra::Engine::CameraManager*>( m_engine->getSystem( "DefaultCameraManager" ) );
    std::string camName = "CAMERA_" + std::to_string( camMngr->count() );
    auto entity         = m_engine->getEntityManager()->createEntity( camName );
    Ra::Engine::Camera* cam =
        new Ra::Engine::Camera( entity, camName, m_viewer->width(), m_viewer->height() );
    // Copy Camera data
    auto manip  = static_cast<Ra::Gui::TrackballCamera*>( m_viewer->getCameraInterface() );
    auto camera = manip->getCamera();
    cam->resize( camera->getWidth(), camera->getHeight() );
    cam->setType( camera->getType() );
    cam->setFrame( camera->getFrame() );
    cam->setFOV( camera->getFOV() );
    cam->setZNear( camera->getZNear() );
    cam->setZFar( camera->getZFar() );
    cam->setZoomFactor( camera->getZoomFactor() );
    cam->initialize();
    cam->show( true );
    // Register entity and camera in Camera manager
    camMngr->addCamera( cam );
}

void CameraManipPluginC::onCurrentChanged( const QModelIndex& current, const QModelIndex& prev ) {
    m_widget->ui->m_useCamera->setEnabled( false );
    if ( m_selectionManager->hasSelection() )
    {
        const Ra::Engine::ItemEntry& ent = m_selectionManager->currentItem();
        if ( ent.m_component == nullptr ) { return; }
        if ( ent.m_component->getName().compare( 0, 7, "CAMERA_" ) == 0 )
        { m_widget->ui->m_useCamera->setEnabled( true ); }
    }
    // update keyFrames for removing Camera
    auto camMngr =
        static_cast<Ra::Engine::CameraManager*>( m_engine->getSystem( "DefaultCameraManager" ) );
    for ( auto it = m_keyFrames.begin(); it != m_keyFrames.end(); ++it )
    {
        bool found = false;
        for ( size_t i = 0; i < camMngr->count(); ++i )
        {
            if ( camMngr->getCamera( i ) == it->second ) { found = true; }
        }
        if ( !found ) { it = m_keyFrames.erase( it ); }
    }
}

#define TIME_DELTA 1_ra
#define DT 0.1_ra

void CameraManipPluginC::addKeyFrame() {
    createCamera();
    auto camMngr =
        static_cast<Ra::Engine::CameraManager*>( m_engine->getSystem( "DefaultCameraManager" ) );
    auto cam = const_cast<Ra::Engine::Camera*>( camMngr->getCamera( camMngr->count() - 1 ) );
    cam->show( false );
    m_keyFrames[m_time] = cam;
    m_time += TIME_DELTA;
}

void CameraManipPluginC::showFrames( bool show ) {
    for ( auto& cam : m_keyFrames )
    {
        cam.second->show( show );
    }
    askForUpdate();
}

void CameraManipPluginC::playPath( bool play ) {
    m_isPlaying = play;
    m_widget->ui->m_addKeyFrame->setEnabled( !play );
    setContinuousUpdate( play );
}

void CameraManipPluginC::resetPath() {
    m_isReset = true;
    askForUpdate();
}

void CameraManipPluginC::advance() {
    if ( ( !m_isPlaying && !m_isReset ) || m_keyFrames.empty() ) { return; }

    if ( m_isReset )
    {
        m_time    = 0;
        m_isReset = false;
    }
    else
    { m_time += DT; }

    Scalar t0, t1, dt;
    auto it = m_keyFrames.find( m_time );
    // exact match
    if ( it != m_keyFrames.end() )
    {
        t0 = it->first;
        t1 = t0;
        dt = 0.0;
    }
    // before first
    else if ( m_time < m_keyFrames.begin()->first )
    {
        t0 = m_keyFrames.begin()->first;
        t1 = t0;
        dt = 0.0;
    }
    // after last
    else if ( m_time > m_keyFrames.rbegin()->first )
    {
        t0 = m_keyFrames.rbegin()->first;
        t1 = t0;
        dt = 0.0;
    }
    // in-between
    else
    {
        auto upper = m_keyFrames.upper_bound( m_time );
        auto lower = upper;
        --lower;
        t0 = lower->first;
        t1 = upper->first;
        dt = ( m_time - t0 ) / ( t1 - t0 );
    }
    Ra::Core::Transform frame;
    Ra::Core::Animation::interpolate(
        m_keyFrames[t0]->getFrame(), m_keyFrames[t1]->getFrame(), dt, frame );

    auto camMngr =
        static_cast<Ra::Engine::CameraManager*>( m_engine->getSystem( "DefaultCameraManager" ) );
    std::string camName = "CAMERA_" + std::to_string( camMngr->count() );
    auto manip          = static_cast<Ra::Gui::TrackballCamera*>( m_viewer->getCameraInterface() );
    auto camera         = manip->getCamera();
    camera->setFrame( frame );
}

void CameraManipPluginC::savePath() const {
    QSettings settings;
    QString path = settings.value( "CameraManip::path_file", QDir::homePath() ).toString();
    path         = QFileDialog::getSaveFileName( nullptr, "Open Camera Path", path, "*.camPath" );
    if ( path.size() == 0 ) { return; }
    settings.setValue( "CameraManip::path_file", path );

    std::ofstream outFile( path.toStdString() );
    if ( !outFile.is_open() )
    {
        LOG( logWARNING ) << "Could not open file to save the camera path: " << path.toStdString();
        return;
    }

    outFile << m_keyFrames.size() << std::endl;
    for ( const auto& cam : m_keyFrames )
    {
        auto camera = cam.second;
        outFile << "#Radium_camera_state" << std::endl;
        outFile << cam.first << std::endl;
        outFile << int( camera->getType() ) << std::endl;
        outFile << camera->getFrame().matrix() << std::endl;
        outFile << std::endl;
        outFile << camera->getFOV() << " " << camera->getZNear() << " " << camera->getZFar() << " "
                << camera->getZoomFactor() << " " << camera->getAspect() << std::endl;
        outFile << std::endl;
    }
}

void CameraManipPluginC::loadPath() {
    QSettings settings;
    QString path = settings.value( "CameraManip::path_file", QDir::homePath() ).toString();
    path         = QFileDialog::getOpenFileName( nullptr, "Open Camera Path", path, "*.camPath" );
    if ( path.size() == 0 ) { return; }
    settings.setValue( "CameraManip::path_file", path );

    std::ifstream inFile( path.toStdString() );
    if ( !inFile.is_open() )
    {
        LOG( logWARNING ) << "Could not open file to save the camera path: " << path.toStdString();
        return;
    }

    m_keyFrames.clear();

    int n;
    Scalar t;
    int type;
    Scalar M[16]; // 4x4 view matrix;
    Scalar fov, znear, zfar, zoom, aspect;
    std::string str;
    bool result;

    auto camMngr =
        static_cast<Ra::Engine::CameraManager*>( m_engine->getSystem( "DefaultCameraManager" ) );

    inFile >> n;
    result = !inFile.fail();
    for ( int i = 0; i < n; ++i )
    {
        inFile >> str;
        result &= !inFile.fail();
        inFile >> t;
        result &= !inFile.fail();
        inFile >> type;
        result &= !inFile.fail();
        for ( uint i = 0; i < 16; ++i )
        {
            inFile >> M[i];
            result &= !inFile.fail();
        }
        inFile >> fov >> znear >> zfar >> zoom >> aspect;
        result &= !inFile.fail();
        if ( !result )
        {
            LOG( logERROR ) << "[CameraManip] Error while loading camera path file: "
                            << path.toStdString();
            return;
        }

        // Create new entity with camera component only
        std::string camName = "CAMERA_" + std::to_string( camMngr->count() );
        auto entity         = m_engine->getEntityManager()->createEntity( camName );
        Ra::Engine::Camera* cam =
            new Ra::Engine::Camera( entity, camName, m_viewer->width(), m_viewer->height() );
        // Copy Camera data
        auto manip  = static_cast<Ra::Gui::TrackballCamera*>( m_viewer->getCameraInterface() );
        auto camera = manip->getCamera();
        cam->resize( camera->getWidth(), camera->getHeight() );
        cam->setType( Ra::Engine::Camera::ProjType( type ) );
        cam->setFrame( Ra::Core::Transform( Ra::Core::Matrix4( M ).transpose() ) );
        cam->setFOV( fov );
        cam->setZNear( znear );
        cam->setZFar( zfar );
        cam->setZoomFactor( zoom );
        cam->initialize();
        cam->show( true );
        // Register entity and camera in Camera manager
        camMngr->addCamera( cam );

        m_keyFrames[t] = cam;
    }
}

} // namespace CameraManipPlugin
