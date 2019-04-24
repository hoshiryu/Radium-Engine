#ifndef ANIMATIONPLUGIN_HPP_
#define ANIMATIONPLUGIN_HPP_

#include <Core/CoreMacros.hpp>
#include <PluginBase/RadiumPluginInterface.hpp>
#include <QAction>
#include <QObject>
#include <QtPlugin>

#include <UI/AnimationUI.h>

#include <AnimationPluginMacros.hpp>

namespace Ra {
namespace Engine {
class RadiumEngine;
} // namespace Engine
} // namespace Ra

/// The AnimationPlugin manages skeleton-based character animation.
namespace AnimationPlugin {
// Due to an ambigous name while compiling with Clang, we must differentiate the
// plugin class from plugin namespace
class AnimationPluginC : public QObject, Ra::Plugins::RadiumPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA( IID "RadiumEngine.PluginInterface" )
    Q_INTERFACES( Ra::Plugins::RadiumPluginInterface )

  public:
    AnimationPluginC();
    ~AnimationPluginC();

    void registerPlugin( const Ra::PluginContext& context ) override;

    bool doAddWidget( QString& name ) override;
    QWidget* getWidget() override;

    bool doAddMenu() override;
    QMenu* getMenu() override;

    bool doAddAction( int& nb ) override;
    QAction* getAction( int id ) override;

    /// \brief Enable the animation and the playzone groupboxex and set the combo boxes.
    void setupUI();

    /// \returns true if the IK solver is enabled, false otherwise.
    bool isIKEnabled();

  public slots:
    /// Slot for the user activating xray display of bones.
    void toggleXray( bool on );

    /// Slot for the user activating display of bones.
    void toggleSkeleton( bool on );

    /// Enable the IK.
    void enableIK( bool status );

    /// Display the timeline.
    void showTimeline();

    /// Slot for the user asking to step the animation once.
    void step();

    /// Slot for the user asking to play the animation.
    void play();

    /// Slot for the user asking to pause the animation.
    void pause();

    /// Slot for the user asking to reset the animation.
    void reset();

    /// Slot for the user changing the animation to play.
    void setAnimation( uint i );

    /// Sets playzone to the i-th.
    void setPlayzone( int i );

    /// Slot for the user asking to use the animation timestep or the application's.
    void toggleAnimationTimeStep( bool status );

    /// Slot for the user changing the animation speed.
    void setAnimationSpeed( Scalar value );

    /// Slot for the user asking for slow motion.
    void toggleSlowMotion( bool status );

    /// Updates the displayed animation time.
    void updateAnimTime();

    /// Save all animation data to a file (one per component).
    void cacheFrame();

    /// Restore all animation from a file, if such a file exists.
    void restoreFrame( int frame );

    /// Request changing the data file directory.
    void changeDataDir();

    /// \brief Creates a new playzone for the current animation.
    /// \param name: the name of the playzone.
    void newPlayzone( const std::string& name );

    /// \brief Remove the i-th playzone for the current animation.
    /// \param i: the index of the playzone to remove.
    void removePlayzone( int i );

    /// \brief Creates a new animation.
    void newAnimation();

    /// \brief Remove the i-th animation (and therefore its playzones).
    /// \param i: the index of the animation to remove.
    void removeAnimation( int i );

    /// \brief Load an RDMA file.
    /// \param filename: the name of the file to load from.
    void loadRDMA( std::string filename );

    /// \brief Save all the animation that were not loaded with the model file.
    /// \param filename: the name of the file to write in.
    void saveRDMA( std::string filename );

    /// \brief Updates the current pose.
    /// \param time: the time to set the animation time.
    void setCurrentAnimationTime( double time );

    /// \brief Sets the current playzone start.
    /// \param timestamp: the timestamp to set the start of the playzone.
    void setStart( double timestamp );

    /// \brief Sets the current playzone end.
    /// \param timestamp: the timestamp to set the end of the playzone.
    void setEnd( double timestamp );

    /// \brief Add a keypose to the current animation at timestamp.
    /// \param timestamp: the timestamp where to add the key pose.
    void addKeyPose( double timestamp );

    /// \brief Remove the i-th key pose.
    /// \param i: the index of the key pose to remove.
    void removeKeyPose( int i );

    /// \brief Set the i-th keypose timestamp.
    /// \param i: the index of the key pose to change.
    /// \param timestamp: the timestamp the keypose should be set at.
    void setKeyPoseTime( int i, double timestamp );

    /// \brief Update the i-th key pose with the current pose.
    /// \param id: the index of the pose to replace.
    void updateKeyPose( int i );

    /// \brief Add an offset to every key pose timestamp after first (included) in the current animation.
    /// \param offset: the offset to add to the key poses.
    /// \param first: the index of the first pose to move.
    void offsetKeyPoses( double offset, int first );

  private:
    /// The data directory.
    std::string m_dataDir;

    /// The AnimationSystem.
    class AnimationSystem* m_system{nullptr};

    /// The Animation widget.
    AnimationUI* m_widget{nullptr};

    /// The SelectionManager of the Viewer.
    Ra::GuiBase::SelectionManager* m_selectionManager{nullptr};
};

} // namespace AnimationPlugin

#endif // ANIMATIONPLUGIN_HPP_
