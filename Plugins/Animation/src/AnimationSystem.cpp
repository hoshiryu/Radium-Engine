#include <AnimationSystem.hpp>

#include <iostream>
#include <string>

#include <Core/Asset/FileData.hpp>
#include <Core/Tasks/Task.hpp>
#include <Core/Tasks/TaskQueue.hpp>

#include <Engine/FrameInfo.hpp>
#include <Engine/RadiumEngine.hpp>

#include <AnimationComponent.hpp>
#include <Drawing/SkeletonBoneDrawable.hpp>

namespace AnimationPlugin {

AnimationSystem::AnimationSystem( AnimationPluginC* plugin ) {
    m_isPlaying = false;
    m_oneStep   = false;
    m_xrayOn    = false;
    m_animFrame = 0;
    m_plugin = plugin;
}

void AnimationSystem::generateTasks( Ra::Core::TaskQueue* taskQueue,
                                     const Ra::Engine::FrameInfo& frameInfo ) {
    const bool playFrame = m_isPlaying || m_oneStep;

    if ( playFrame ) { ++m_animFrame; }

    // deal with AnimationComponents
    Scalar currentDelta = playFrame ? frameInfo.m_dt : 0;
    for ( auto compEntry : m_components )
    {
        auto animComp = static_cast<AnimationComponent*>( compEntry.second );
        auto animFunc = std::bind( &AnimationComponent::update, animComp, currentDelta );
        auto animTask = new Ra::Core::FunctionTask( animFunc, "AnimatorTask" );
        taskQueue->registerTask( animTask );
    }

    CoupledTimedSystem::generateTasks( taskQueue, frameInfo );

    m_oneStep = false;
}

void AnimationSystem::play( bool isPlaying ) {
    m_isPlaying = isPlaying;

    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->setPlaying( isPlaying );
    }
    CoupledTimedSystem::play( isPlaying );
}

void AnimationSystem::step() {
    m_oneStep = true;
    CoupledTimedSystem::step();
}

void AnimationSystem::reset() {
    m_animFrame = 0;

    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->reset();
    }
    CoupledTimedSystem::reset();
}

bool AnimationSystem::isXrayOn() {
    return m_xrayOn;
}

void AnimationSystem::setXray( bool on ) {
    m_xrayOn = on;
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->setXray( on );
    }
}

void AnimationSystem::toggleSkeleton( const bool status ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->toggleSkeleton( status );
    }
}

void AnimationSystem::enableIK( bool status ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->enableIK( status );
    }
}

void AnimationSystem::setAnimation( const uint i ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->setAnimation( i );
    }
}

void AnimationSystem::toggleAnimationTimeStep( const bool status ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->toggleAnimationTimeStep( status );
    }
}

void AnimationSystem::setAnimationSpeed( const Scalar value ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->setSpeed( value );
    }
}

void AnimationSystem::toggleSlowMotion( const bool status ) {
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->toggleSlowMotion( status );
    }
}

void AnimationSystem::handleAssetLoading( Ra::Engine::Entity* entity,
                                          const Ra::Core::Asset::FileData* fileData ) {
    auto geomData = fileData->getGeometryData();
    auto skelData = fileData->getHandleData();
    auto animData = fileData->getAnimationData();

    // deal with AnimationComponents
    for ( const auto& skel : skelData )
    {
        auto component = new AnimationComponent( "AC_" + skel->getName(), entity );
        component->handleSkeletonLoading( skel );
        component->handleAnimationLoading( animData );

        component->setXray( m_xrayOn );
        registerComponent( entity, component );
    }

    CoupledTimedSystem::handleAssetLoading( entity, fileData );
    m_plugin->setupUI();
    m_plugin->showTimeline();
    enableIK( m_plugin->isIKEnabled() );
}

Scalar AnimationSystem::getTime( const Ra::Engine::ItemEntry& entry ) const {
    if ( entry.isValid() )
    {
        // If entry is an existing animation component, we return this one's time
        // if not, look for other components in this entity to see if some are animation
        std::vector<const AnimationComponent*> comps;
        for ( const auto& ec : m_components )
        {
            if ( ec.first == entry.m_entity )
            {
                const auto c = static_cast<AnimationComponent*>( ec.second );
                // Entry match, return that one
                if ( ec.second == c ) { return c->getTime(); }
                comps.push_back( c );
            }
        }
        // If comps is not empty, it means that we have a component in current entity
        // We just pick the first one
        if ( !comps.empty() ) { return comps[0]->getTime(); }
    }
    return 0.f;
}

uint AnimationSystem::getMaxFrame() const {
    uint m = 0;
    for ( const auto& comp : m_components )
    {
        m = std::max( m, static_cast<AnimationComponent*>( comp.second )->getMaxFrame() );
    }
    return m;
}

void AnimationSystem::cacheFrame( const std::string& dir, uint frameId ) const {
    // deal with AnimationComponents
    for ( const auto& comp : m_components )
    {
        static_cast<AnimationComponent*>( comp.second )->cacheFrame( dir, frameId );
    }

    CoupledTimedSystem::cacheFrame( dir, frameId );
}

bool AnimationSystem::restoreFrame( const std::string& dir, uint frameId ) {
    static bool restoringCurrent = false;
    if ( !restoringCurrent )
    {
        // first save current, in case restoration fails.
        cacheFrame( dir, m_animFrame );
    }
    bool success = true;
    // deal with AnimationComponents
    for ( const auto& comp : m_components )
    {
        success &= static_cast<AnimationComponent*>( comp.second )->restoreFrame( dir, frameId );
    }
    // if fail, restore current frame
    if ( !success && !restoringCurrent )
    {
        restoringCurrent = true;
        restoreFrame( dir, m_animFrame );
        restoringCurrent = false;
        return false;
    }

    success &= CoupledTimedSystem::restoreFrame( dir, frameId );
    // if fail, restore current frame
    if ( !success && !restoringCurrent )
    {
        restoringCurrent = true;
        restoreFrame( dir, m_animFrame );
        restoringCurrent = false;
        return false;
    }

    if ( success ) { m_animFrame = frameId; }
    return success;
}

/// Sets playzone to i.
void AnimationSystem::setPlayzone( int i ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->setPlayzone( i );
}

/// Sets the current playzone start.
void AnimationSystem::setStart( double timestamp ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->setStart( timestamp );
}

/// Sets the current playzone end.
void AnimationSystem::setEnd( double timestamp ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->setEnd( timestamp );
}

/// Creates a new playzone for the current animation.
void AnimationSystem::newPlayzone( const std::string& name ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->newPlayzone( name );
}

/// Remove the i-th playzone for the current animation.
void AnimationSystem::removePlayzone( int i ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->removePlayzone( i );
}

/// Creates a new animation.
void AnimationSystem::newAnimation() {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->newAnimation();
}

/// Remove the i-th animation (and therefore its playzones).
void AnimationSystem::removeAnimation( int i ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->removeAnimation( i );
}

/// Load and .rdma file.
void AnimationSystem::loadRDMA( const std::string& filename ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->loadRDMA( filename );
}

/// Save all the animation that were not loaded with the model file.
void AnimationSystem::saveRDMA( const std::string& filename ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->saveRDMA( filename );
}

/// Updates the current pose.
void AnimationSystem::setCurrentAnimationTime( double timestamp ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )
            ->setCurrentAnimationTime( timestamp );
}

/// Add a keypose to the current animation at timestamp.
void AnimationSystem::addKeyPose( double timestamp ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->addKeyPose( timestamp );
}

/// Remove the i-th keypose
void AnimationSystem::removeKeyPose( int i ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->removeKeyPose( i );
}

/// Set the i-th keypose timestamp.
void AnimationSystem::setKeyPoseTime( int i, double timestamp ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )
            ->setKeyPoseTime( i, timestamp );
}

void AnimationSystem::updateKeyPose( int id ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )->updateKeyPose( id );
}

/// Add and offset to every key poses timestamp after first (included) of the current animation.
void AnimationSystem::offsetKeyPoses( double offset, int first ) {
    if ( !m_components.empty() )
        static_cast<AnimationComponent*>( m_components.back().second )
            ->offsetKeyPoses( offset, first );
}

/// Getter for the playzones labels.
std::vector<std::string> AnimationSystem::playzonesLabels() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->playzonesLabels();
    return std::vector<std::string>{};
}

/// Getter for the animation count.
int AnimationSystem::animationCount() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->animationCount();
    return 0;
}

/// Returns the current animation time
double AnimationSystem::animationTime() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->getTime();
    return 0.0;
}

/// Returns the number of non editable animation.
int AnimationSystem::nonEditableCount() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->nonEditableCount();
    return 0;
}

/// Returns a vector of the keyposes timestamps
std::vector<double> AnimationSystem::keyposesTimes() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->keyposesTimes();
    return std::vector<double>{};
}

/// Returns the start of the current playzone.
double AnimationSystem::getStart() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->getStart();
    return 0.0;
}

/// Returns the end of the current playzone.
double AnimationSystem::getEnd() const {
    if ( !m_components.empty() )
        return static_cast<AnimationComponent*>( m_components.back().second )->getEnd();
    return 5.0;
}

/// Returns m_isPlaying.
bool AnimationSystem::isPlaying() const {
    return m_isPlaying;
}

} // namespace AnimationPlugin
