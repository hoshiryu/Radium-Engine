#include <AnimationPlugin.hpp>

#include <QAction>
#include <QFileDialog>
#include <QIcon>
#include <QSettings>
#include <QToolBar>

#include <AnimationSystem.hpp>
#include <Engine/Managers/SignalManager/SignalManager.hpp>
#include <Engine/RadiumEngine.hpp>
#include <GuiBase/SelectionManager/SelectionManager.hpp>

#include "ui_AnimationUI.h"
#include <UI/AnimationUI.h>

namespace AnimationPlugin {

AnimationPluginC::AnimationPluginC() = default;

AnimationPluginC::~AnimationPluginC() = default;

void AnimationPluginC::registerPlugin( const Ra::PluginContext& context ) {
    QSettings settings;
    QString path = settings.value( "AnimDataDir" ).toString();
    if ( path.isEmpty() ) { path = QString( context.m_exportDir.c_str() ); }
    m_system = new AnimationSystem( this );
    context.m_engine->registerSystem( "AnimationSystem", m_system );
    context.m_engine->getSignalManager()->m_frameEndCallbacks.push_back(
        std::bind( &AnimationPluginC::updateAnimTime, this ) );
    m_selectionManager = context.m_selectionManager;
}

bool AnimationPluginC::doAddWidget( QString& name ) {
    name = "Animation";
    return true;
}

QWidget* AnimationPluginC::getWidget() {
    m_widget = new AnimationUI();
    connect( m_widget, &AnimationUI::toggleXray, this, &AnimationPluginC::toggleXray );
    connect( m_widget, &AnimationUI::showSkeleton, this, &AnimationPluginC::toggleSkeleton );
    connect( m_widget, &AnimationUI::animationID, this, &AnimationPluginC::setAnimation );
    connect( m_widget,
             &AnimationUI::toggleAnimationTimeStep,
             this,
             &AnimationPluginC::toggleAnimationTimeStep );
    connect( m_widget, &AnimationUI::animationSpeed, this, &AnimationPluginC::setAnimationSpeed );
    connect( m_widget, &AnimationUI::toggleSlowMotion, this, &AnimationPluginC::toggleSlowMotion );
    connect( m_widget, &AnimationUI::play, this, &AnimationPluginC::play );
    connect( m_widget, &AnimationUI::pause, this, &AnimationPluginC::pause );
    connect( m_widget, &AnimationUI::step, this, &AnimationPluginC::step );
    connect( m_widget, &AnimationUI::stop, this, &AnimationPluginC::reset );
    connect( m_widget, &AnimationUI::cacheFrame, this, &AnimationPluginC::cacheFrame );
    connect( m_widget, &AnimationUI::restoreFrame, this, &AnimationPluginC::restoreFrame );
    connect( m_widget, &AnimationUI::changeDataDir, this, &AnimationPluginC::changeDataDir );

    connect( m_widget, &AnimationUI::playZoneID, this, &AnimationPluginC::setPlayzone );
    connect( m_widget, &AnimationUI::newPlayzone, this, &AnimationPluginC::newPlayzone );
    connect( m_widget, &AnimationUI::removePlayzone, this, &AnimationPluginC::removePlayzone );
    connect( m_widget, &AnimationUI::newAnimation, this, &AnimationPluginC::newAnimation );
    connect( m_widget, &AnimationUI::removeAnimation, this, &AnimationPluginC::removeAnimation );
    connect( m_widget, &AnimationUI::loadRDMA, this, &AnimationPluginC::loadRDMA );
    connect( m_widget, &AnimationUI::saveRDMA, this, &AnimationPluginC::saveRDMA );

    /// Timeline signals
    connect( m_widget, &AnimationUI::cursorChanged, this,
             &AnimationPluginC::setCurrentAnimationTime );
    connect( m_widget, &AnimationUI::startChanged, this, &AnimationPluginC::setStart );
    connect( m_widget, &AnimationUI::endChanged, this, &AnimationPluginC::setEnd );
    connect( m_widget, &AnimationUI::keyPoseAdded, this, &AnimationPluginC::addKeyPose );
    connect( m_widget, &AnimationUI::keyPoseDeleted, this, &AnimationPluginC::removeKeyPose );
    connect( m_widget, &AnimationUI::keyPoseChanged, this, &AnimationPluginC::setKeyPoseTime );
    connect( m_widget, &AnimationUI::keyPosesChanged, this, &AnimationPluginC::offsetKeyPoses );

    return m_widget;
}

bool AnimationPluginC::doAddMenu() {
    return false;
}

QMenu* AnimationPluginC::getMenu() {
    return nullptr;
}

bool AnimationPluginC::doAddAction( int& nb ) {
    nb = 4;
    return true;
}

QAction* AnimationPluginC::getAction( int id ) {
    switch ( id )
    {
    case 0:
        return m_widget->ui->actionXray;
    case 1:
        return m_widget->ui->actionPlay;
    case 2:
        return m_widget->ui->actionStep;
    case 3:
        return m_widget->ui->actionStop;
    default:
        return nullptr;
    }
}

void AnimationPluginC::setupUIAnimation() {
    m_widget->ui->groupBox_animation->setEnabled( true );
    m_widget->ui->groupBox_playZone->setEnabled( true );
    m_widget->setAnimationComboBox( m_system->animationCount() );
    setupUIPlayzones();
}

void AnimationPluginC::setupUIPlayzones() {
    m_widget->setPlayzoneComboBox( m_system->playzonesLabels() );
}

void AnimationPluginC::toggleXray( bool on ) {
    CORE_ASSERT( m_system, "System should be there " );
    m_system->setXray( on );
}

void AnimationPluginC::play() {
    CORE_ASSERT( m_system, "System should be there " );
    const double animationTime = m_system->animationTime();
    const double start = m_system->getStart();

    if ( animationTime > m_system->getEnd() || animationTime < start )
    {
        m_system->setCurrentAnimationTime( start );
    }
    m_system->play( true );
}

void AnimationPluginC::pause() {
    CORE_ASSERT( m_system, "System should be there " );
    m_system->play( false );
}

void AnimationPluginC::step() {
    CORE_ASSERT( m_system, "System should be there " );
    pause();
    m_system->step();
}

void AnimationPluginC::reset() {
    CORE_ASSERT( m_system, "System should be there " );
    pause();
    m_system->reset();
}

void AnimationPluginC::toggleSkeleton( bool status ) {
    m_system->toggleSkeleton( status );
}

void AnimationPluginC::setAnimation( uint i ) {
    m_system->setAnimation( i );
    m_system->setPlayzone( 0 );
    m_widget->setKeyposes( m_system->keyposesTimes() );
    m_widget->setPlayzoneComboBox( m_system->playzonesLabels() );
}

void AnimationPluginC::toggleAnimationTimeStep( bool status ) {
    m_system->toggleAnimationTimeStep( status );
}

void AnimationPluginC::setAnimationSpeed( Scalar value ) {
    m_system->setAnimationSpeed( value );
}

void AnimationPluginC::toggleSlowMotion( bool status ) {
    m_system->toggleSlowMotion( status );
}

void AnimationPluginC::updateAnimTime() {
    m_widget->setMaxFrame( m_system->getMaxFrame() );
    m_widget->updateTime( m_system->getTime( m_selectionManager->currentItem() ) );
    m_widget->updateFrame( m_system->getAnimFrame() );

    emit m_widget->changeCursor( m_system->animationTime() );
}

void AnimationPluginC::cacheFrame() {
    m_system->cacheFrame( m_dataDir );
}

void AnimationPluginC::restoreFrame( int frame ) {
    if ( m_system->restoreFrame( m_dataDir, frame ) ) { m_widget->frameLoaded( frame ); }
}

void AnimationPluginC::changeDataDir() {
    QSettings settings;
    QString path = settings.value( "AnimDataDir", QDir::homePath() ).toString();
    path = QFileDialog::getExistingDirectory( nullptr, "Animation Data Dir", path );
    if ( !path.isEmpty() )
    {
        settings.setValue( "AnimDataDir", path );
        m_dataDir = path.toStdString();
    }
}

void AnimationPluginC::setPlayzone( int i ) {
    if ( i >= 0 )
    {
        m_system->setPlayzone( i );

        const double end = m_system->getEnd();
        emit m_widget->changeStart( m_system->getStart() );
        emit m_widget->changeEnd( end );
        emit m_widget->changeDuration( end * 1.25 );
    }
}

void AnimationPluginC::newPlayzone( const std::string& name ) {
    m_system->newPlayzone( name );
}

void AnimationPluginC::removePlayzone( int i ) {
    m_system->removePlayzone( i );
}

void AnimationPluginC::newAnimation() {
    m_system->newAnimation();
}

void AnimationPluginC::removeAnimation( int i ) {
    m_system->removeAnimation( i );
}

void AnimationPluginC::loadRDMA( std::string filename ) {
    m_system->loadRDMA( filename );
}

void AnimationPluginC::saveRDMA( std::string filename ) {
    m_system->saveRDMA( filename );
}

void AnimationPluginC::setCurrentAnimationTime( double timestamp ) {
    m_system->setCurrentAnimationTime(static_cast<Scalar>(timestamp));
}

void AnimationPluginC::setStart( double timestamp ) {
    m_system->setStart( timestamp );
}

void AnimationPluginC::setEnd( double timestamp ) {
    m_system->setEnd( timestamp );
}

void AnimationPluginC::addKeyPose( double timestamp ) {
    m_system->addKeyPose(static_cast<Scalar>(timestamp));
}

void AnimationPluginC::removeKeyPose( int i ) {
    m_system->removeKeyPose(i);
}

void AnimationPluginC::setKeyPoseTime( int i, double timestamp ) {
    m_system->setKeyPoseTime( i, timestamp );
}

void AnimationPluginC::offsetKeyPoses( double offset ) {
    m_system->offsetKeyPoses(static_cast<Scalar>(offset));
}

} // namespace AnimationPlugin
