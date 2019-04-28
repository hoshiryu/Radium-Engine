#include "AnimationUI.h"
#include "ui_AnimationUI.h"

#include <QDebug>
#include <QFileDialog>
#include <QInputDialog>
#include <QSettings>
#include <iostream>

AnimationUI::AnimationUI( QWidget* parent ) : QFrame( parent ), ui( new Ui::AnimationUI ) {
    ui->setupUi( this );
    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();

    connect( ui->actionXray, &QAction::toggled, this, &AnimationUI::on_m_xray_clicked );
    connect( ui->actionXray, &QAction::toggled, this, &AnimationUI::on_m_showSkeleton_toggled );
    connect( ui->actionPlay, &QAction::toggled, this, &AnimationUI::on_m_play_clicked );
    connect( ui->actionStep, &QAction::triggered, this, &AnimationUI::on_m_step_clicked );
    connect( ui->actionStop, &QAction::triggered, this, &AnimationUI::on_m_reset_clicked );

    // parent is null, can't access Ra::Gui::MainWindow so change Animation CMakeLists.txt
    // or change constructor calling for AnimationUI
    // so first timeline pos is fixed at the bottom right of current screen
    timeline = new AnimTimelineWithSession( parent );

    connect( timeline, &AnimTimelineWithSession::playClicked, this, &AnimationUI::play );
    connect( timeline, &AnimTimelineWithSession::pauseClicked, this, &AnimationUI::pause );
    connect( timeline, &AnimTimelineWithSession::cursorChanged, this, &AnimationUI::cursorChanged );
    connect( timeline, &AnimTimelineWithSession::startChanged, this, &AnimationUI::startChanged );
    connect( timeline, &AnimTimelineWithSession::endChanged, this, &AnimationUI::endChanged );
    connect( timeline, &AnimTimelineWithSession::keyPoseAdded, this, &AnimationUI::keyPoseAdded );
    connect( timeline, &AnimTimelineWithSession::keyPoseDeleted, this, &AnimationUI::keyPoseDeleted );
    connect( timeline, &AnimTimelineWithSession::keyPoseChanged, this, &AnimationUI::keyPoseChanged );
    connect( timeline, &AnimTimelineWithSession::keyPosesMoved, this, &AnimationUI::keyPosesMoved );
    connect( timeline, &AnimTimelineWithSession::keyPoseMoved, this, &AnimationUI::keyPoseMoved );

    // session (undo/redo)
    connect( timeline, &AnimTimelineWithSession::envSaved, this, &AnimationUI::envSaved);
    connect( timeline, &AnimTimelineWithSession::rendered, this, &AnimationUI::rendered);
    connect( timeline, &AnimTimelineWithSession::renderDeleted, this, &AnimationUI::renderDeleted);


    connect( this, &AnimationUI::changeCursor, timeline, &AnimTimelineWithSession::onChangeCursor );
    connect( this, &AnimationUI::addKeyPose, timeline, &AnimTimelineWithSession::onAddingKeyPose );
    connect( this, &AnimationUI::changeStart, timeline, &AnimTimelineWithSession::onChangeStart );
    connect( this, &AnimationUI::changeEnd, timeline, &AnimTimelineWithSession::onChangeEnd );
    connect( this, &AnimationUI::changeDuration, timeline,
             &AnimTimelineWithSession::onChangeDuration );
    connect( this, &AnimationUI::play, timeline, &AnimTimelineWithSession::onSetPlayMode );
    connect( this, &AnimationUI::pause, timeline, &AnimTimelineWithSession::onSetPauseMode );
    connect( this, &AnimationUI::clearKeyPoses, timeline, &AnimTimelineWithSession::onClearKeyPoses );

}

AnimationUI::~AnimationUI() {
    delete ui;
    delete timeline;
}

void AnimationUI::setAnimationComboBox( int count, int nonEditableCount ) {
    ui->comboBox_currentAnimation->clear();

    for ( int i = 0; i < nonEditableCount; ++i )
    {
        ui->comboBox_currentAnimation->addItem( "Not editable #" + QString::number( i + 1 ) );
    }

    for ( int i = nonEditableCount; i < count; ++i )
    {
        ui->comboBox_currentAnimation->addItem( "#" + QString::number( i + 1 ) );
    }
}

void AnimationUI::setPlayzoneComboBox( const std::vector<std::string> labels ) {
    ui->comboBox_currentPlayZone->clear();

    for ( const auto& label : labels )
    {
        ui->comboBox_currentPlayZone->addItem( label.c_str() );
    }
}

void AnimationUI::setKeyPoses( std::vector<double> timestamps ) {
    for ( const auto& time : timestamps )
    {
        emit addKeyPose( time );
    }
}

void AnimationUI::showTimeline() {
    timeline->show();
}

void AnimationUI::showEvent( QShowEvent* event ) {
    (void)event;
    if(ui->comboBox_currentAnimation->isEnabled()) {
        timeline->show();
    }
}

void AnimationUI::hideEvent( QHideEvent* event ) {
    timeline->hide();
}

void AnimationUI::on_m_xray_clicked( bool checked ) {
    emit toggleXray( checked );
}

void AnimationUI::on_m_showSkeleton_toggled( bool checked ) {
    emit showSkeleton( checked );
}

void AnimationUI::on_m_enableIK_toggled( bool checked ) {
    emit enableIK( checked );
}

void AnimationUI::switchToPauseButton() {
    ui->m_play->setChecked( true );

    ui->m_play->setProperty( "pressed", true );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();
}

void AnimationUI::switchToPlayButton() {
    ui->m_play->setChecked( false );

    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();
}

void AnimationUI::on_m_play_clicked() {
    if ( ui->m_play->isChecked() )
    {
        switchToPauseButton();
        emit play();
    } else
    {
        switchToPlayButton();
        emit pause();
    }
}

void AnimationUI::on_m_step_clicked() {
    ui->m_play->setChecked( false );

    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();

    emit step();
}

void AnimationUI::on_m_reset_clicked() {
    ui->m_play->setChecked( false );

    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();

    emit stop();
}

void AnimationUI::on_m_timeStep_currentIndexChanged( int index ) {
    emit toggleAnimationTimeStep( ( index == 0 ) );
}

void AnimationUI::on_m_speed_valueChanged( double arg1 ) {
    emit animationSpeed( arg1 );
}

void AnimationUI::on_m_slowMo_toggled( bool checked ) {
    emit toggleSlowMotion( checked );
}

void AnimationUI::on_m_cacheFrame_clicked() {
    emit cacheFrame();
}

void AnimationUI::on_m_restoreFrame_clicked() {
    emit restoreFrame( ui->m_loadedFrame->value() );
}

void AnimationUI::frameLoaded( int f ) {
    ui->m_currentFrame->setText( QString::number( f ) );
}

void AnimationUI::setMaxFrame( int f ) {
    ui->m_loadedFrame->setMaximum( f );
}

void AnimationUI::on_m_saveDir_clicked() {
    emit changeDataDir();
}

void AnimationUI::updateTime( float t ) {
    ui->m_animationTimeDisplay->setText( QString::number( t ) );
}

void AnimationUI::updateFrame( int f ) {
    ui->m_currentFrame->setText( QString::number( f ) );
}

void AnimationUI::on_comboBox_currentPlayZone_currentIndexChanged( int index ) {
    emit playzoneID( index );
}

void AnimationUI::on_pushButton_newPlayZone_clicked() {
    bool ok;
    QString text = QInputDialog::getText( this, tr( "Adding PlayZone" ), tr( "PlayZone name :" ),
                                          QLineEdit::Normal, "", &ok );

    if ( ok && !text.isEmpty() )
    {
        ui->comboBox_currentPlayZone->addItem( text );
        emit newPlayzone( text.toStdString() );
    }
}

void AnimationUI::on_pushButton_removePlayZone_clicked() {
    int removeIndex = ui->comboBox_currentPlayZone->currentIndex();
    if ( ui->comboBox_currentPlayZone->count() > 1 )
    {
        ui->comboBox_currentPlayZone->removeItem( removeIndex );
        emit removePlayzone( removeIndex );
    }
}

void AnimationUI::on_comboBox_currentAnimation_currentIndexChanged( int index ) {
    emit animationID( index );
}

void AnimationUI::on_pushButton_newAnimation_clicked() {
    const int num = ui->comboBox_currentAnimation->count();
    ui->comboBox_currentAnimation->addItem( "#" + QString::number( num + 1 ) );
    emit newAnimation();
}

void AnimationUI::on_pushButton_removeAnimation_clicked() {
    const int removeIndex = ui->comboBox_currentAnimation->currentIndex();

    if ( ui->comboBox_currentAnimation->count() > 1 )
    {
        ui->comboBox_currentAnimation->removeItem( removeIndex );
        emit removeAnimation( removeIndex );
    }
}

void AnimationUI::on_pushButton_loadRdmaFile_clicked() {
    QSettings settings;
    QString path = settings.value( "files/load", QDir::homePath() ).toString();
    QString filename = QFileDialog::getOpenFileName( this, "Open RDMA file", path,
                                                     "Radium Animation File (*.rdma)" );

    if ( !filename.isEmpty() )
    {
        ui->label_currentRDMA->setText( filename );
        ui->pushButton_saveRdmaFile->setEnabled( true );
        emit loadRDMA( filename.toStdString() );
    }
}

void AnimationUI::on_pushButton_saveRdmaFile_clicked() {
    emit saveRDMA( ui->label_currentRDMA->text().toStdString() );
}

void AnimationUI::on_pushButton_newRdmaFile_clicked() {
    QSettings settings;
    QString path = settings.value( "files/load", QDir::homePath() ).toString();
    QString filename = QFileDialog::getSaveFileName( this, "Open RDMA file", path,
                                                     "Radium Animation File (*.rdma)" );

    if ( !filename.isEmpty() )
    {
        ui->label_currentRDMA->setText( filename );
        ui->pushButton_saveRdmaFile->setEnabled( true );
        emit saveRDMA( filename.toStdString() );
    }
}

void AnimationUI::on_saveRendering(void * anim, size_t bytes) {
    timeline->onSaveRendering(anim, bytes);
}
