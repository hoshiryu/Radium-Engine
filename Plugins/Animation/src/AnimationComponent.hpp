#ifndef ANIMPLUGIN_ANIMATION_COMPONENT_HPP_
#define ANIMPLUGIN_ANIMATION_COMPONENT_HPP_

#include <AnimationPluginMacros.hpp>

#include <Core/Animation/Animation.hpp>
#include <Core/Animation/HandleWeight.hpp>
#include <Core/Animation/Pose.hpp>
#include <Core/Animation/Skeleton.hpp>
#include <Core/Asset/AnimationData.hpp>
#include <Core/Asset/HandleData.hpp>

#include <Engine/Component/Component.hpp>

#include <memory>

namespace AnimationPlugin {

class SkeletonBoneRenderObject;

/// The AnimationComponent is responsible for the management of skeleton-based
/// character animations. It stores the animation Skeleton and the animation
/// data and is responsible for drawing the skeleton.
class ANIM_PLUGIN_API AnimationComponent : public Ra::Engine::Component
{
  public:
    AnimationComponent( const std::string& name, Ra::Engine::Entity* entity );
    ~AnimationComponent();
    AnimationComponent( const AnimationComponent& ) = delete;
    AnimationComponent& operator=( const AnimationComponent& ) = delete;

    using Playzones = std::vector<std::tuple<std::string, Scalar, Scalar>>;

    virtual void initialize() override {}

    //
    // Build from fileData
    //

    /// Create the skeleton from the given data.
    void handleSkeletonLoading( const Ra::Core::Asset::HandleData* data );

    /// Create the animations from the given data.
    void handleAnimationLoading( const std::vector<Ra::Core::Asset::AnimationData*>& data );

    //
    // Animation
    //

    /// Set the animation skeleton.
    void setSkeleton( const Ra::Core::Animation::Skeleton& skel );

    /// @returns the animation skeleton.
    inline Ra::Core::Animation::Skeleton& getSkeleton() { return m_skel; }

    /// Update the skeleton with an animation.
    void update( Scalar dt );

    /// Resets the animation, thus posing the skeleton into the reference pose.
    void reset();

    /// Saves all the state data related to the current frame into a cache file.
    void cacheFrame( const std::string& dir, int frame ) const;

    /// Restores the state data related to the \p frameID -th frame from the cache file.
    /// \returns true if the frame has been successfully restored, false otherwise.
    /// Note: the AnimationSystem ensures that in case the frame restoration fails,
    ///       the Component still remains in the current frame state
    bool restoreFrame( const std::string& dir, int frame );

    /// If \p status is TRUE, then use the animation time step if available;
    /// else, use the application timestep.
    void toggleAnimationTimeStep( const bool status );

    /// Set animation speed factor.
    void setSpeed( const Scalar value );

    /// Toggle the slow motion speed (speed x0.1).
    void toggleSlowMotion( const bool status );

    /// Set the animation to play.
    void setAnimation( const uint i );

    /// Set the playzone to use.
    void setPlayzone( const uint i );

    /// @returns the current time of animation.
    Scalar getTime() const;

    /// @returns the duration of the current animation.
    Scalar getDuration() const;

    /// @returns the duration of the current animation.
    uint getMaxFrame() const;

    // Skeleton display
    /// Turns xray display on/off for the skeleton bones.
    void setXray( bool on ) const;

    /// Toggle skeleton bones display.
    void toggleSkeleton( const bool status );

    /// Creates a new playzone for the current animation.
    void newPlayzone( const std::string& name );

    /// Remove the i-th playzone for the current animation.
    void removePlayzone( int i );

    /// Creates a new animation with a default playzone.
    void newAnimation();

    /// Creates a new animation from the current one.
    void copyCurrentAnimation();

    /// Remove the i-th animation (and therefore its playzones).
    void removeAnimation( int i );

    /// Loads the animations contained in the file at filepath
    /// Only loads animations and each animation d_t and playzones
    void loadRDMA( const std::string& filepath );

    /// Save the created animations with their d_t and playzones
    void saveRDMA( const std::string& filepath );

    /// Sets current pose: should not be used when playing.
    void setCurrentPose();

    /// Sets current playzoneID to i.
    void setPlayzoneID( int i );

    /// Sets the current animation time and updates the current pose.
    void setCurrentAnimationTime( double timestamp );

    /// Returns the start of the current playzone.
    double getStart() const;

    /// Returns the end of the current playzone.
    double getEnd() const;

    /// Sets the current playzone start.
    void setStart( double timestamp );

    /// Sets the current playzone end.
    void setEnd( double timestamp );

    /// Add a keypose to the current animation at timestamp.
    void addKeyPose( double timestamp );

    /// Remove the i-th keypose
    void removeKeyPose( int i );

    /// Set the i-th keypose timestamp
    void setKeyPoseTime( int i, double timestamp );

    /// Update the i-th keypose by the current skeleton render.
    void updateKeyPose( int id );

    /// Add and offset to every key poses and playzones timestamp after first (included) of the
    /// current animation.
    void offsetKeyPoses( double offset, int first );

    /// Returns the playzones labels.
    std::vector<std::string> playzonesLabels() const;

    /// Returns the animation count.
    int animationCount() const;

    /// Returns the number of non editable animation.
    int nonEditableCount() const;

    /// Returns the keyposes' timestamps.
    std::vector<double> keyposesTimes() const;

    //
    // Editable interface
    //

    virtual bool canEdit( const Ra::Core::Utils::Index& roIdx ) const override;

    virtual Ra::Core::Transform getTransform( const Ra::Core::Utils::Index& roIdx ) const override;

    virtual void setTransform( const Ra::Core::Utils::Index& roIdx,
                               const Ra::Core::Transform& transform ) override;

  private:
    // Internal function to create the bone display objects.
    void setupSkeletonDisplay();

    // debug function to display the hierarchy
    void printSkeleton( const Ra::Core::Animation::Skeleton& skeleton );

    //
    // Component Communication (CC)
    //

    /// Setup CC.
    void setupIO( const std::string& id );

    /// Skeleton getter for CC.
    const Ra::Core::Animation::Skeleton* getSkeletonOutput() const;

    /// Referene Pose getter for CC.
    const Ra::Core::Animation::RefPose* getRefPoseOutput() const;

    /// Reset status getter for CC.
    const bool* getWasReset() const;

    /// Current Animation getter for CC.
    const Ra::Core::Animation::Animation* getAnimationOutput() const;

    /// Current Animation Time for CC.
    const Scalar* getTimeOutput() const;

    /// Map from RO index to bone index for CC.
    const std::map<Ra::Core::Utils::Index, uint>* getBoneRO2idx() const;

  private:
    /// Entity name for CC.
    std::string m_contentName;

    /// The Animation Skeleton.
    Ra::Core::Animation::Skeleton m_skel;

    /// The Reference Pose in model space.
    Ra::Core::Animation::RefPose m_refPose;

    /// The animations.
    std::vector<Ra::Core::Animation::Animation> m_animations;

    /// Each animation saved cursor position.
    std::vector<Playzones> m_animsPlayzones;

    /// Bones ROs.
    std::vector<std::unique_ptr<SkeletonBoneRenderObject>> m_boneDrawables;

    /// Map from bone RO index to bone idx, for CC.
    std::map<Ra::Core::Utils::Index, uint> m_boneMap;

    /// Current animation ID.
    uint m_animationID;
    
    /// First editable animation index.
    uint m_firstEditableID;

    /// Current playzone ID.
    uint m_playzoneID;

    /// Wheither to use the animation timestep (if available) or the app's.
    bool m_animationTimeStep;

    /// Current animation time.
    Scalar m_animationTime;

    /// Time step of each animation.
    std::vector<Scalar> m_dt;

    /// Animation Play speed.
    Scalar m_speed;

    /// Is slow motion active?
    bool m_slowMo;

    /// Was the animation reset?
    bool m_wasReset;

    /// Is the reset process done?
    bool m_resetDone;
};

} // namespace AnimationPlugin

#endif // ANIMPLUGIN_ANIMATION_COMPONENT_HPP_
