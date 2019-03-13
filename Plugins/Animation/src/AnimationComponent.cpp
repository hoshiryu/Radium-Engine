#include <AnimationComponent.hpp>

#include <fstream>
#include <iostream>
#include <queue>

#include <Core/Animation/KeyPose.hpp>
#include <Core/Animation/KeyTransform.hpp>
#include <Core/Animation/Pose.hpp>
#include <Core/Asset/HandleToSkeleton.hpp>
#include <Core/Containers/AlignedStdVector.hpp>
#include <Core/Geometry/TriangleMesh.hpp>

#include <Engine/Managers/ComponentMessenger/ComponentMessenger.hpp>
#include <Engine/Renderer/RenderObject/RenderObjectManager.hpp>

#include <Drawing/SkeletonBoneDrawable.hpp>

using Ra::Core::Animation::Animation;
using Ra::Core::Animation::Handle;
using Ra::Core::Animation::RefPose;
using Ra::Core::Animation::Skeleton;
using Ra::Core::Animation::WeightMatrix;
using Ra::Engine::ComponentMessenger;

using namespace Ra::Core::Utils; // log

namespace AnimationPlugin {

AnimationComponent::AnimationComponent( const std::string& name, Ra::Engine::Entity* entity ) :
    Component( name, entity ),
    m_animationID( 0 ),
    m_animationTimeStep( true ),
    m_animationTime( 0.0 ),
    m_dt(),
    m_speed( 1.0 ),
    m_slowMo( false ),
    m_wasReset( false ),
    m_resetDone( false ) {}

AnimationComponent::~AnimationComponent() {}

void AnimationComponent::setSkeleton( const Ra::Core::Animation::Skeleton& skel ) {
    m_skel    = skel;
    m_refPose = skel.getPose( Handle::SpaceType::MODEL );
    setupSkeletonDisplay();
}

void AnimationComponent::update( Scalar dt ) {
    if ( dt != 0.0 )
    {
        const Scalar factor = ( m_slowMo ? 0.1f : 1.0f ) * m_speed;
        // Use the animation dt if required AND if we actually have animations.
        dt = factor * ( ( m_animationTimeStep && m_dt.size() > 0 ) ? m_dt[m_animationID] : dt );
    }
    // Ignore large dt that appear when the engine is paused (while loading a file for instance)
    if ( !m_animationTimeStep && ( dt > 0.5f ) ) { dt = 0; }

    // Compute the elapsed time
    m_animationTime += dt;

    if ( m_wasReset )
    {
        if ( !m_resetDone ) { m_resetDone = true; }
        else
        {
            m_resetDone = false;
            m_wasReset  = false;
        }
    }

    // get the current pose from the animation
    if ( dt > 0 && !m_animations.empty() )
    {
        setCurrentPose();
    }

    // update the render objects
    for ( auto& bone : m_boneDrawables )
    {
        bone->update();
    }
}

void AnimationComponent::setupSkeletonDisplay() {
    m_renderObjects.clear();
    m_boneDrawables.clear();
    for ( uint i = 0; i < m_skel.size(); ++i )
    {
        if ( !m_skel.m_graph.isLeaf( i ) && !m_skel.m_graph.isRoot( i ) &&
             m_skel.getLabel( i ).find( "_$AssimpFbx$_" ) == std::string::npos )
        {
            std::string name = m_skel.getLabel( i ) + "_" + std::to_string( i );
            m_boneDrawables.emplace_back(
                new SkeletonBoneRenderObject( name, this, i, getRoMgr() ) );
            m_renderObjects.push_back( m_boneDrawables.back()->getRenderObjectIndex() );
        }
        else
        { LOG( logDEBUG ) << "Bone " << m_skel.getLabel( i ) << " not displayed."; }
    }
    for ( const auto& b : m_boneDrawables )
    {
        m_boneMap[b->getRenderObjectIndex()] = b->getBoneIndex();
    }
}

void AnimationComponent::printSkeleton( const Ra::Core::Animation::Skeleton& skeleton ) {
    std::deque<int> queue;
    std::deque<int> levels;

    queue.push_back( 0 );
    levels.push_back( 0 );
    while ( !queue.empty() )
    {
        int i = queue.front();
        queue.pop_front();
        int level = levels.front();
        levels.pop_front();
        std::cout << i << " " << skeleton.getLabel( i ) << "\t";
        for ( const auto& c : skeleton.m_graph.children()[i] )
        {
            queue.push_back( c );
            levels.push_back( level + 1 );
        }

        if ( levels.front() != level ) { std::cout << std::endl; }
    }
}

void AnimationComponent::reset() {
    m_animationTime = 0;
    m_skel.setPose( m_refPose, Handle::SpaceType::MODEL );
    for ( auto& bone : m_boneDrawables )
    {
        bone->update();
    }
    m_wasReset = true;
}

void AnimationComponent::handleSkeletonLoading( const Ra::Core::Asset::HandleData* data ) {
    std::string name( m_name );
    name.append( "_" + data->getName() );

    std::string skelName = name;
    skelName.append( "_SKEL" );

    m_skel.setName( name );

    m_contentName = data->getName();

    Ra::Core::Asset::createSkeleton( *data, m_skel );

    m_refPose = m_skel.getPose( Handle::SpaceType::MODEL );

    setupSkeletonDisplay();
    setupIO( m_contentName );
}

void AnimationComponent::handleAnimationLoading(
    const std::vector<Ra::Core::Asset::AnimationData*>& data ) {
    m_animations.clear();
    CORE_ASSERT( ( m_skel.size() != 0 ), "At least a skeleton should be loaded first." );
    if ( data.empty() ) return;

    for ( uint n = 0; n < data.size(); ++n )
    {
        std::map<uint, uint> table;
        std::set<Ra::Core::Animation::Time> keyTime;
        auto handleAnim = data[n]->getFrames();
        for ( uint i = 0; i < m_skel.size(); ++i )
        {
            for ( uint j = 0; j < handleAnim.size(); ++j )
            {
                if ( m_skel.getLabel( i ) == handleAnim[j].m_name )
                {
                    table[j] = i;
                    auto set = handleAnim[j].m_anim.timeSchedule();
                    keyTime.insert( set.begin(), set.end() );
                }
            }
        }

        if ( keyTime.empty() ) { continue; }

        Ra::Core::Animation::KeyPose keypose;
        Ra::Core::Animation::Pose pose = m_skel.m_pose;

        m_animations.push_back( Ra::Core::Animation::Animation() );
        for ( const auto& t : keyTime )
        {
            for ( const auto& it : table )
            {
                // pose[it.second] = ( m_skel.m_graph.isRoot( it.second ) ) ?
                // m_skel.m_pose[it.second] : handleAnim[it.first].m_anim.at( t );
                pose[it.second] = handleAnim[it.first].m_anim.at( t );
            }
            m_animations.back().addKeyPose( pose, t );
            keypose.insertKeyFrame( t, pose );
        }

        m_dt.push_back( data[n]->getTimeStep() );
    }
    m_animationID   = 0;
    m_animationTime = 0.0;
}

void AnimationComponent::setupIO( const std::string& id ) {
    ComponentMessenger::CallbackTypes<Skeleton>::Getter skelOut =
        std::bind( &AnimationComponent::getSkeletonOutput, this );
    ComponentMessenger::getInstance()->registerOutput<Skeleton>( getEntity(), this, id, skelOut );

    ComponentMessenger::CallbackTypes<RefPose>::Getter refpOut =
        std::bind( &AnimationComponent::getRefPoseOutput, this );
    ComponentMessenger::getInstance()->registerOutput<Ra::Core::Animation::Pose>(
        getEntity(), this, id, refpOut );

    ComponentMessenger::CallbackTypes<bool>::Getter resetOut =
        std::bind( &AnimationComponent::getWasReset, this );
    ComponentMessenger::getInstance()->registerOutput<bool>( getEntity(), this, id, resetOut );

    ComponentMessenger::CallbackTypes<Animation>::Getter animOut =
        std::bind( &AnimationComponent::getAnimationOutput, this );
    ComponentMessenger::getInstance()->registerOutput<Animation>( getEntity(), this, id, animOut );

    ComponentMessenger::CallbackTypes<Scalar>::Getter timeOut =
        std::bind( &AnimationComponent::getTimeOutput, this );
    ComponentMessenger::getInstance()->registerOutput<Scalar>( getEntity(), this, id, timeOut );

    using BoneMap = std::map<Ra::Core::Utils::Index, uint>;
    ComponentMessenger::CallbackTypes<BoneMap>::Getter boneMapOut =
        std::bind( &AnimationComponent::getBoneRO2idx, this );
    ComponentMessenger::getInstance()->registerOutput<BoneMap>( getEntity(), this, id, boneMapOut );
}

const Ra::Core::Animation::Skeleton* AnimationComponent::getSkeletonOutput() const {
    return &m_skel;
}

const Ra::Core::Animation::RefPose* AnimationComponent::getRefPoseOutput() const {
    return &m_refPose;
}

const bool* AnimationComponent::getWasReset() const {
    return &m_wasReset;
}

void AnimationComponent::setXray( bool on ) const {
    for ( const auto& b : m_boneDrawables )
    {
        b->setXray( on );
    }
}

void AnimationComponent::toggleSkeleton( const bool status ) {
    for ( const auto& b : m_boneDrawables )
    {
        const auto id = b->getRenderObjectIndex();
        getRoMgr()->getRenderObject( id )->setVisible( status );
    }
}

void AnimationComponent::toggleAnimationTimeStep( const bool status ) {
    m_animationTimeStep = status;
}

void AnimationComponent::setSpeed( const Scalar value ) {
    m_speed = value;
}

void AnimationComponent::toggleSlowMotion( const bool status ) {
    m_slowMo = status;
}

void AnimationComponent::setAnimation( const uint i ) {
    if ( i < m_animations.size() ) { m_animationID = i; }
}

bool AnimationComponent::canEdit( const Ra::Core::Utils::Index& roIdx ) const {
    // returns true if the roIdx is one of our bones.
    return (
        std::find_if( m_boneDrawables.begin(), m_boneDrawables.end(), [roIdx]( const auto& bone ) {
            return bone->getRenderObjectIndex() == roIdx;
        } ) != m_boneDrawables.end() );
}

Ra::Core::Transform AnimationComponent::getTransform( const Ra::Core::Utils::Index& roIdx ) const {
    CORE_ASSERT( canEdit( roIdx ), "Transform is not editable" );
    const auto& bonePos =
        std::find_if( m_boneDrawables.begin(), m_boneDrawables.end(), [roIdx]( const auto& bone ) {
            return bone->getRenderObjectIndex() == roIdx;
        } );

    const uint boneIdx = ( *bonePos )->getBoneIndex();
    return m_skel.getPose( Handle::SpaceType::MODEL )[boneIdx];
}

void AnimationComponent::setTransform( const Ra::Core::Utils::Index& roIdx,
                                       const Ra::Core::Transform& transform ) {
    CORE_ASSERT( canEdit( roIdx ), "Transform is not editable" );
    const auto& bonePos =
        std::find_if( m_boneDrawables.begin(), m_boneDrawables.end(), [roIdx]( const auto& bone ) {
            return bone->getRenderObjectIndex() == roIdx;
        } );

    // get bone data
    const uint boneIdx     = ( *bonePos )->getBoneIndex();
    const auto& TBoneModel = m_skel.getTransform( boneIdx, Handle::SpaceType::MODEL );
    const auto& TBoneLocal = m_skel.getTransform( boneIdx, Handle::SpaceType::LOCAL );

    // turn bone translation into rotation for parent
    const uint pBoneIdx = m_skel.m_graph.parents()[boneIdx];
    if ( pBoneIdx != -1 && m_skel.m_graph.children()[pBoneIdx].size() == 1 )
    {
        const auto& pTBoneModel = m_skel.getTransform( pBoneIdx, Handle::SpaceType::MODEL );

        Ra::Core::Vector3 A;
        Ra::Core::Vector3 B;
        m_skel.getBonePoints( pBoneIdx, A, B );
        Ra::Core::Vector3 B_ = transform.translation();
        auto q               = Ra::Core::Quaternion::FromTwoVectors( ( B - A ), ( B_ - A ) );
        Ra::Core::Transform R( q );
        R.pretranslate( A );
        R.translate( -A );
        m_skel.setTransform( pBoneIdx, R * pTBoneModel, Handle::SpaceType::MODEL );
    }

    // update bone local transform
    m_skel.setTransform(
        boneIdx, TBoneLocal * TBoneModel.inverse() * transform, Handle::SpaceType::LOCAL );
}

const Ra::Core::Animation::Animation* AnimationComponent::getAnimationOutput() const {
    if ( m_animations.empty() ) { return nullptr; }
    return &m_animations[m_animationID];
}

const Scalar* AnimationComponent::getTimeOutput() const {
    return &m_animationTime;
}

const std::map<Ra::Core::Utils::Index, uint>* AnimationComponent::getBoneRO2idx() const {
    return &m_boneMap;
}

Scalar AnimationComponent::getTime() const {
    return m_animationTime;
}

Scalar AnimationComponent::getDuration() const {
    if ( m_animations.empty() ) { return Scalar( 0 ); }
    return m_animations[m_animationID].getDuration();
}

uint AnimationComponent::getMaxFrame() const {
    if ( m_animations.empty() ) { return 0; }
    return uint( std::round( getDuration() / m_dt[m_animationID] ) );
}

void AnimationComponent::cacheFrame( const std::string& dir, int frame ) const {
    std::ofstream file( dir + "/" + m_contentName + "_frame" + std::to_string( frame ) + ".anim",
                        std::ios::trunc | std::ios::out | std::ios::binary );
    if ( !file.is_open() ) { return; }
    file.write( reinterpret_cast<const char*>( &m_animationID ), sizeof m_animationID );
    file.write( reinterpret_cast<const char*>( &m_animationTimeStep ), sizeof m_animationTimeStep );
    file.write( reinterpret_cast<const char*>( &m_animationTime ), sizeof m_animationTime );
    file.write( reinterpret_cast<const char*>( &m_speed ), sizeof m_speed );
    file.write( reinterpret_cast<const char*>( &m_slowMo ), sizeof m_slowMo );
    const auto& pose = m_skel.getPose( Handle::SpaceType::LOCAL );
    file.write( reinterpret_cast<const char*>( pose.data() ), ( sizeof pose[0] ) * pose.size() );
    LOG( logINFO ) << "Saving anim data at time: " << m_animationTime;
}

bool AnimationComponent::restoreFrame( const std::string& dir, int frame ) {
    std::ifstream file( dir + "/" + m_contentName + "_frame" + std::to_string( frame ) + ".anim",
                        std::ios::in | std::ios::binary );
    if ( !file.is_open() ) { return false; }
    if ( !file.read( reinterpret_cast<char*>( &m_animationID ), sizeof m_animationID ) )
    { return false; }
    if ( !file.read( reinterpret_cast<char*>( &m_animationTimeStep ), sizeof m_animationTimeStep ) )
    { return false; }
    if ( !file.read( reinterpret_cast<char*>( &m_animationTime ), sizeof m_animationTime ) )
    { return false; } if ( !file.read( reinterpret_cast<char*>( &m_speed ), sizeof m_speed ) )
    { return false; } if ( !file.read( reinterpret_cast<char*>( &m_slowMo ), sizeof m_slowMo ) )
    { return false; } auto pose = m_skel.getPose( Handle::SpaceType::LOCAL );
    if ( !file.read( reinterpret_cast<char*>( pose.data() ), ( sizeof pose[0] ) * pose.size() ) )
    { return false; } m_skel.setPose( pose, Handle::SpaceType::LOCAL );

    // update the render objects
    for ( auto& bone : m_boneDrawables )
    {
        bone->update();
    }

    return true;
}

// TODO: Add playzones to load and save
void AnimationComponent::loadRDMA( const std::string& filepath ) {
    std::ifstream input{filepath, std::ios::binary};

    size_t size;
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    const auto pose_size = size;
    if ( pose_size != m_refPose.size() )
    {
        // TODO: Log error/display a popup ?
        std::cerr << "Animation pose size and current reference pose size mismatch.\n"
                     "Please make sure you're trying to load animation corresponding to the "
                     "currently loaded model."
                  << std::endl;
        return;
    }

    /// Importing animations
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_animations.resize( m_animations.size() + size );
    for ( size_t i = m_firstEditableID; i < m_animations.size(); ++i )
    {
        auto& anim = m_animations[i];
        input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
        for ( size_t i = 0; i < size; ++i )
        {
            Scalar timestamp;
            input.read( reinterpret_cast<char*>( &timestamp ), sizeof( timestamp ) );
            Ra::Core::Animation::Pose pose( pose_size );
            for ( size_t i = 0; i < pose_size; ++i )
            {
                input.read( reinterpret_cast<char*>( pose[i].data() ),
                            sizeof( Ra::Core::Transform ) );
            }
            anim.addKeyPose( pose, timestamp );
        }
    }

    /// Importing each animation playzones
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_animsPlayzones.resize( m_animsPlayzones.size() + size );
    for ( auto& playzones : m_animsPlayzones )
    {
        size_t playzoneSize;
        input.read( reinterpret_cast<char*>( &playzoneSize ), sizeof( playzoneSize ) );
        playzones.resize( playzoneSize );
        for ( auto& playzone : playzones )
        {
            auto& name = std::get<0>( playzone );
            input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
            name.resize( size );
            input.read( reinterpret_cast<char*>( &name[0] ), size );
            input.read( reinterpret_cast<char*>( &std::get<1>( playzone ) ), sizeof( Scalar ) );
            input.read( reinterpret_cast<char*>( &std::get<2>( playzone ) ), sizeof( Scalar ) );
        }
    }

    /// Importing delta t's
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_dt.resize( m_dt.size() + size );
    input.read( reinterpret_cast<char*>( &m_dt[m_firstEditableID] ), size * sizeof( Scalar ) );
}

void AnimationComponent::saveRDMA( const std::string& filepath ) {
    CORE_ASSERT( m_skel.size() != 0, "No skeleton loaded." );
    std::ofstream output{filepath, std::ios::binary};

    /// Exporting animations
    const size_t pose_size = m_refPose.size();
    output.write( reinterpret_cast<const char*>( &pose_size ), sizeof( pose_size ) );
    size_t size = m_animations.size() - m_firstEditableID + 1;
    output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    for ( size_t i = m_firstEditableID; i < m_animations.size(); ++i )
    {
        const auto& anim = m_animations[i];
        // anim.normalize();
        size = anim.size();
        output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
        for ( size_t i = 0; i < size; ++i )
        {
            const auto& keypose = anim.keyPose( i );
            output.write( reinterpret_cast<const char*>( &keypose.first ),
                          sizeof( keypose.first ) );
            for ( const auto& transform : keypose.second )
            {
                output.write( reinterpret_cast<const char*>( transform.data() ),
                              sizeof( Transform ) );
            }
        }
    }

    /// Exporting each animation playzones
    size = m_animsPlayzones.size();
    output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    for ( const auto& playzones : m_animsPlayzones )
    {
        size = playzones.size();
        output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
        for ( const auto& playzone : playzones )
        {
            const auto& name = std::get<0>( playzone );
            size = name.length();
            output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
            output.write( reinterpret_cast<const char*>( &name[0] ), size );
            output.write( reinterpret_cast<const char*>( &std::get<1>( playzone ) ),
                          sizeof( Scalar ) );
            output.write( reinterpret_cast<const char*>( &std::get<2>( playzone ) ),
                          sizeof( Scalar ) );
        }
    }

    /// Exporting delta t's
    size = m_dt.size() - m_firstEditableID + 1;
    output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    output.write( reinterpret_cast<const char*>( &m_dt[m_firstEditableID] ),
                  size * sizeof( Scalar ) );
}

/// Creates a new playzone for the current animation.
void AnimationComponent::newPlayzone() {
    const auto& anim = m_animations[m_animationID];
    m_playzoneID = m_animsPlayzones.size();
    m_animsPlayzones[m_animationID].emplace_back( "Playzone #" + m_playzoneID,
                                                  anim.keyPose( 0 ).first,
                                                  anim.keyPose( anim.size() - 1 ).first );
}

/// Remove the i-th playzone for the current animation.
void AnimationComponent::removePlayzone( int i ) {
    auto& playzone = m_animsPlayzones[m_animationID];
    playzone.erase( playzone.begin() + i );
    if ( i <= m_playzoneID )
    {
        --m_playzoneID;
    }
}

/// Creates a new animation.
void AnimationComponent::newAnimation() {
    m_animationID = m_animations.size();
    m_animations.emplace_back();
}

/// Remove the i-th animation (and therefore its playzones).
void AnimationComponent::removeAnimation( int i ) {
    if ( i < m_firstEditableID )
    {
        return;
    }

    m_animations.erase(m_animations.begin() + i);
    m_animsPlayzones.erase(m_animsPlayzones.begin() + i - m_firstEditableID);
    m_playzoneID = 0;
    if(i <= m_animationID) {
        --m_animationID;
    }
}

/// Creates an empty .rdma file: temporary.
// void AnimationComponent::newRDMA( const std::string& filename ) {}

inline void AnimationComponent::setCurrentPose() {
    const auto& pose = m_animations[m_animationID].getPose(m_animationTime);
    m_skel.setPose(pose, Handle::SpaceType::LOCAL);
}

/// Updates the current pose.
void AnimationComponent::setCurrentAnimationTime( Scalar timestamp ) {
    m_animationTime = timestamp;
    
    setCurrentPose();
}

/// Add a keypose to the current animation at timestamp.
void AnimationComponent::addKeyPose( Scalar timestamp ) {
    if(m_animationID < m_firstEditableID) {
        newAnimation();
    }
    m_animations[m_animationID].addKeyPose(m_skel.getPose(Handle::SpaceType::LOCAL), timestamp);
}

/// Remove the i-th keypose
void AnimationComponent::removeKeyPose( int i ) {
    if(m_animationID >= m_firstEditableID) {
        m_animations[m_animationID].removeKeyPose(i);
        setCurrentAnimationTime(m_animationTime);
    }
}

/// Set the i-th ?????
// void AnimationComponent::setKeyPoseTime( int i ) {}

/// Add and offset to every key poses of the current animation.
void AnimationComponent::offsetKeyPoses( Scalar offset ) {
    if(m_animationID >= m_firstEditableID) {
        m_animations[m_animationID].offsetKeyPoses(offset);
    }
}

} // namespace AnimationPlugin
