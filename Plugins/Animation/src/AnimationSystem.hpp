#ifndef ANIMPLUGIN_ANIMATION_SYSTEM_HPP_
#define ANIMPLUGIN_ANIMATION_SYSTEM_HPP_

#include <Engine/System/TimedSystem.hpp>

#include <AnimationPlugin.hpp>
#include <AnimationPluginMacros.hpp>
#include <Engine/ItemModel/ItemEntry.hpp>

namespace AnimationPlugin {

/// The AnimationSystem is the main system for coupling TimedSystems.
/// On one hand, it manages the AnimationComponents, i.e. skeleton animation and display.
/// On the other hand, it is responsible for transmitting calls to animation-related systems,
/// for example physics systems that must play with the animation.
class ANIM_PLUGIN_API AnimationSystem : public Ra::Engine::CoupledTimedSystem
{
  public:
    /// Create a new animation system
    AnimationSystem( AnimationPluginC* plugin );

    AnimationSystem( const AnimationSystem& ) = delete;
    AnimationSystem& operator=( const AnimationSystem& ) = delete;

    /// Create a task for each animation component to advance the current animation.
    void generateTasks( Ra::Core::TaskQueue* taskQueue,
                        const Ra::Engine::FrameInfo& frameInfo ) override;

    /// Load a skeleton and an animation from a file.
    void handleAssetLoading( Ra::Engine::Entity* entity,
                             const Ra::Core::Asset::FileData* fileData ) override;

    /// Toggle on/off playing of animations.
    void play( bool isPlaying ) override;

    /// Advance the animation next frame, then pauses.
    void step() override;

    /// Resets the skeleton to its rest pose.
    void reset() override;

    /// Saves all the state data related to the current frame into a cache file.
    void cacheFrame( const std::string& dir ) const { cacheFrame( dir, m_animFrame ); }

    /// Saves all the state data related to the given frame into a cache file.
    void cacheFrame( const std::string& dir, uint frameId ) const override;

    /// Restores the state data related to the \p frameID -th frame from the cache file.
    /// \returns true if the frame has been successfully restored, false otherwise.
    bool restoreFrame( const std::string& dir, uint frameId ) override;

    /// Set on or off xray bone display.
    void setXray( bool on );

    /// Is xray bone display on.
    bool isXrayOn();

    /// Display the skeleton.
    void toggleSkeleton( const bool status );

    /// Set the animation to play.
    void setAnimation( const uint i );

    /// If \p status is `true`, then use the animation time step if available;
    /// else, use the application timestep.
    void toggleAnimationTimeStep( const bool status );

    /// Set animation speed factor.
    void setAnimationSpeed( const Scalar value );

    /// Toggle the slow motion speed (speed x0.1).
    void toggleSlowMotion( const bool status );

    /// @returns the animation time corresponding to the \p entry 's entity.
    Scalar getTime( const Ra::Engine::ItemEntry& entry ) const;

    /// @returns the system frame.
    uint getAnimFrame() const { return m_animFrame; }

    /// @returns the system frame.
    uint getMaxFrame() const;

    /// Sets playzone to i.
    void setPlayzone( int i );

    /// Creates a new playzone for the current animation.
    void newPlayzone(const std::string& name);

    /// Remove the i-th playzone for the current animation.
    void removePlayzone( int i );

    /// Creates a new animation.
    void newAnimation();

    /// Remove the i-th animation (and therefore its playzones).
    void removeAnimation( int i );

    /// Load and .rdma file.
    void loadRDMA( const std::string& filename );

    /// Save all the animation that were not loaded with the model file.
    void saveRDMA( const std::string& filename );

    /// Updates the current pose.
    void setCurrentAnimationTime( double timestamp );

    /// Sets the current playzone start
    void setStart( double timestamp );

    /// Sets the current playzone end
    void setEnd( double timestamp );

    /// Add a keypose to the current animation at timestamp.
    void addKeyPose( double timestamp );

    /// Remove the i-th keypose.
    void removeKeyPose( int i );

    /// Set the i-th keypose timestamp.
    void setKeyPoseTime( int i, double timestamp );

    /// Add and offset to every key poses of the current animation.
    void offsetKeyPoses( double offset );

    /// Returns current animation playzones' labels.
    std::vector<std::string> playzonesLabels() const;

    /// Returns the animation count.
    int animationCount() const;

    /// Returns the current animation time.
    double animationTime() const;
    
    /// Returns a vector of the keyposes timestamps.
    std::vector<double> keyposesTimes() const; 

    /// Returns the start of the current playzone.
    double getStart() const;

    /// Returns the end of the current playzone.
    double getEnd() const;

    /// Return the duration of the current animation.
    double getCurrentDuration() const;

  private:
    /// Current frame
    uint m_animFrame;

    /// See if animation is playing or paused
    bool m_isPlaying;

    /// True if one step has been required to play.
    bool m_oneStep;

    /// True if we want to show xray-bones
    bool m_xrayOn;

    /// Used to setup the UI according to the events
    AnimationPluginC* m_plugin;
};
} // namespace AnimationPlugin

#endif // ANIMPLUGIN_ANIMATION_SYSTEM_HPP_
