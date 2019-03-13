#ifndef ANIMATIONUI_H
#define ANIMATIONUI_H

#include <AnimTimeline/animtimeline.h>
#include <QFrame>

namespace UI {
class AnimationUi;
}

namespace AnimationPlugin {
class AnimationPluginC;
}

namespace Ui {
class AnimationUI;
}

class AnimationUI : public QFrame {
    Q_OBJECT

    friend class AnimationPlugin::AnimationPluginC;

  public:
    explicit AnimationUI( QWidget* parent = 0 );
    ~AnimationUI();

  protected:
    void showEvent( QShowEvent* event ) override;
    void hideEvent( QHideEvent* event ) override;

  signals:
    void toggleXray( bool );
    void showSkeleton( bool );
    void play();
    void pause();
    void step();
    void stop();
    void animationID( int );
    void toggleAnimationTimeStep( bool );
    void animationSpeed( double );
    void toggleSlowMotion( bool );
    void cacheFrame();
    void restoreFrame( int );
    void changeDataDir();

    void playZoneID( int );
    void newPlayzone();
    void removePlayzone( int );
    void newAnimation();
    void removeAnimation( int );
    void loadRDMA( std::string filename );
    void saveRDMA( std::string filename );
    void newRDMA( std::string filename );

    /// Timeline signals
    void cursorChanged( Scalar );
    void startChanged( Scalar );
    void endChanged( Scalar );
    void keyPoseAdded( Scalar );
    void keyPoseDeleted( int );
    void keyPoseChanged( int );
    void keyPosesChanged( Scalar );

  private slots:
    void on_m_xray_clicked( bool checked );
    void on_m_showSkeleton_toggled( bool checked );
    void on_m_play_clicked( bool checked );
    void on_m_step_clicked();
    void on_m_reset_clicked();
    void on_m_timeStep_currentIndexChanged( int index );
    void on_m_speed_valueChanged( double arg1 );
    void on_m_slowMo_toggled( bool checked );
    void on_m_cacheFrame_clicked();
    void on_m_restoreFrame_clicked();
    void frameLoaded( int f );
    void setMaxFrame( int f );
    void on_m_saveDir_clicked();

    void on_comboBox_currentPlayZone_currentIndexChanged( int index );
    void on_pushButton_newPlayZone_clicked();
    void on_pushButton_removePlayZone_clicked();
    void on_comboBox_currentAnimation_currentIndexChanged( int index );
    void on_pushButton_newAnimation_clicked();
    void on_pushButton_removeAnimation_clicked();
    void on_pushButton_loadRdmaFile_clicked();
    void on_pushButton_saveRdmaFile_clicked();
    void on_pushButton_newRdmaFile_clicked();

  private:
    Ui::AnimationUI* ui;
    AnimTimeline* animTimeline;

    void updateTime( float t );
    void updateFrame( int f );
};

#endif // ANIMATIONUI_H
