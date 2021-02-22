#pragma once

#include <Engine/Scene/ItemEntry.hpp>
#include <Engine/Scene/System.hpp>

namespace Ra {
namespace Engine {
namespace Scene {

/**
 * The SkeletonBasedAnimationSystem manages both SkeletonComponents and SkinningComponents.
 * It is also responsible for transmitting calls to/from animation-related processes.
 */
class RA_ENGINE_API SkeletonBasedAnimationSystem : public System
{
  public:
    /// Create a new animation system
    SkeletonBasedAnimationSystem();

    SkeletonBasedAnimationSystem( const SkeletonBasedAnimationSystem& ) = delete;
    SkeletonBasedAnimationSystem& operator=( const SkeletonBasedAnimationSystem& ) = delete;

    /// \name System Interface
    /// \{

    /**
     * Creates a task for each AnimationComponent to update skeleton display.
     */
    void generateTasks( Ra::Core::TaskQueue* taskQueue,
                        const Ra::Engine::FrameInfo& frameInfo ) override;

    /**
     * Loads Skeletons and Animations from a file data into the givn Entity.
     */
    void handleAssetLoading( Entity* entity,
                             const Ra::Core::Asset::FileData* fileData ) override;
    /// \}

    /// \name Skeleton display
    /// \{

    /**
     * Sets bone display xray mode to \p on for all AnimationComponents.
     */
    void setXray( bool on );

    /**
     * \returns true if bone display xray mode on, false otherwise.
     */
    bool isXrayOn();

    /**
     * Toggles skeleton display for all AnimationComponents.
     */
    void toggleSkeleton( const bool status );
    /// \}

    /// \name Animation parameters
    /// \{

    /**
     * Sets the animation speed factor for all AnimationComponents.
     */
    void setAnimationSpeed( const Scalar value );

    /**
     * Toggles animation auto repeat for all AnimationComponents.
     */
    void autoRepeat( const bool status );

    /**
     * Toggles animation ping-pong for all AnimationComponents.
     */
    void pingPong( const bool status );

    /**
     * \returns the animation time of the AnimationComponent corresponding to \p entry 's entity.
     */
    Scalar getAnimationTime( const ItemEntry& entry ) const;
    /// \}

    /// Enable display of skinning weights.
    void showWeights( bool on );

    /// Sets the type of skinning weights to display: 0 - standard, 1 - stbs.
    void showWeightsType( int type );

  private:
    /// True if we want to show xray-bones.
    bool m_xrayOn{false};

    /// The current animation time.
    Scalar m_time{0};
};

} // namespace Scene
} // namespace Engine
} // namespace Ra
