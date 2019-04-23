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
    m_animations(),
    m_animsPlayzones(),
    m_animationID( 0 ),
    m_playzoneID( 0 ),
    m_animationTimeStep( true ),
    m_animationTime( 0.0 ),
    m_dt(),
    m_speed( 1.0 ),
    m_slowMo( false ),
    m_wasReset( false ),
    m_resetDone( false ) {}

AnimationComponent::~AnimationComponent() {}

void AnimationComponent::setSkeleton( const Ra::Core::Animation::Skeleton& skel ) {
    m_skel = skel;
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
            m_wasReset = false;
        }
    }

    // get the current pose from the animation
    if ( dt > 0 && !m_animations.empty() && m_animations[m_animationID].size() >= 2 )
    {
        // Not calling setCurrentPose() to avoid unnecessary function calls
        const auto& pose = m_animations[m_animationID].getPose( m_animationTime );
        m_skel.setPose( pose, Handle::SpaceType::LOCAL );
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
        } else
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
    m_animationTime = std::get<1>( m_animsPlayzones[m_animationID][m_playzoneID] );
    setCurrentPose();
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
    m_animsPlayzones.clear();
    CORE_ASSERT( ( m_skel.size() != 0 ), "At least a skeleton should be loaded first." );
    if ( data.empty() )
    {
        newAnimation();
        m_firstEditableID = 0;
        setAnimation( 0 );
    }

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
    setAnimation( 0 );
    m_firstEditableID = m_animations.size();
    m_animsPlayzones.resize( m_firstEditableID );
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

void AnimationComponent::enableIK( bool status ) {
    m_IKsolverEnabled = status;
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
    if ( i < m_animations.size() )
    {
        m_animationID = i;
        if ( m_animsPlayzones.empty() )
        {
            m_animsPlayzones.emplace_back();
        }
        setPlayzone( 0 );
    }
}

void AnimationComponent::setPlayzone( const uint i ) {
    m_playzoneID = i;
    if ( m_animsPlayzones[m_animationID].empty() )
    {
        newPlayzone( "Default playzone" );
    }
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

// Could use some factorization (for the part that changes a translation into a rotation)
void AnimationComponent::setTransform( const Ra::Core::Utils::Index& roIdx,
                                       const Ra::Core::Transform& transform ) {
    CORE_ASSERT( canEdit( roIdx ), "Transform is not editable" );
    const auto& bonePos =
        std::find_if( m_boneDrawables.begin(), m_boneDrawables.end(), [roIdx]( const auto& bone ) {
            return bone->getRenderObjectIndex() == roIdx;
        } );

    // get bone data
    const uint boneIdx = ( *bonePos )->getBoneIndex();
    const uint pBoneIdx = m_skel.m_graph.parents()[boneIdx];
    const auto& TBoneModel = m_skel.getTransform( boneIdx, Handle::SpaceType::MODEL );

    /// if IK solver is on, the transform is a translation, and the selected bone isn't the root
    if ( m_IKsolverEnabled && !transform.translation().isApprox( TBoneModel.translation() ) &&
         pBoneIdx != -1 )
    {
        // reserve a few elements to avoid too much reallocations ?
        std::vector<uint> jointsIndices;
        std::vector<Scalar> lengths;
        std::vector<Ra::Core::Vector3> p;
        Scalar max_length = 0.0;

        uint pJointIdx = m_skel.m_graph.parents()[boneIdx];
        Ra::Core::Vector3 joint, pJoint;
        joint = m_skel.getTransform( boneIdx, Handle::SpaceType::MODEL ).translation();
        p.emplace_back( joint );
        jointsIndices.emplace_back( boneIdx );

        while ( pJointIdx != -1 )
    {
            pJoint = m_skel.getTransform( pJointIdx, Handle::SpaceType::MODEL ).translation();
            Scalar length = ( joint - pJoint ).norm();
            max_length += length;

            jointsIndices.emplace_back( pJointIdx );
            lengths.emplace_back( length );
            p.emplace_back( pJoint );

            joint = std::move( pJoint );
            pJointIdx = m_skel.m_graph.parents()[pJointIdx];
        }

        IKsolver( lengths, p, transform.translation(), max_length );

        // Currently useless when the target is unreachable
        // Turning every rotation into a translation for the father
        for ( int i = jointsIndices.size() - 2; i >= 0; --i )
        {
            const auto& TBoneModel =
                m_skel.getTransform( jointsIndices[i], Handle::SpaceType::MODEL );
            const auto& pTBoneModel =
                m_skel.getTransform( jointsIndices[i + 1], Handle::SpaceType::MODEL );
            const auto& TBoneLocal =
                m_skel.getTransform( jointsIndices[i], Handle::SpaceType::LOCAL );
            auto trf = TBoneModel;
            trf.translation() = p[i];

            if ( m_skel.m_graph.children()[jointsIndices[i + 1]].size() == 1 )
            {
                const Ra::Core::Vector3& A = p[i + 1];
                const Ra::Core::Vector3& B = TBoneModel.translation();
                const Ra::Core::Vector3& B_ = p[i];
                auto q = Ra::Core::Quaternion::FromTwoVectors( ( B - A ), ( B_ - A ) );
                Ra::Core::Transform R( q );
                R.pretranslate( A );
                R.translate( -A );
                m_skel.setTransform( jointsIndices[i + 1], R * pTBoneModel,
                                     Handle::SpaceType::MODEL );
            }

            // update bone local transform
            m_skel.setTransform( chainJoints[i], TBoneLocal * TBoneModel.inverse() * trf,
                                 Handle::SpaceType::LOCAL );
        }
        // }
    } else
    {
        const auto& TBoneLocal = m_skel.getTransform( boneIdx, Handle::SpaceType::LOCAL );
        // turn bone translation into rotation for parent
        if ( pBoneIdx != -1 && m_skel.m_graph.children()[pBoneIdx].size() == 1 )
        {
            const auto& pTBoneModel = m_skel.getTransform( pBoneIdx, Handle::SpaceType::MODEL );
            Ra::Core::Vector3 A;
            Ra::Core::Vector3 B;
            m_skel.getBonePoints( pBoneIdx, A, B );
            Ra::Core::Vector3 B_ = transform.translation();
            auto q = Ra::Core::Quaternion::FromTwoVectors( ( B - A ), ( B_ - A ) );
            Ra::Core::Transform R( q );
            R.pretranslate( A );
            R.translate( -A );
            m_skel.setTransform( pBoneIdx, R * pTBoneModel, Handle::SpaceType::MODEL );
        }

        // update bone local transform
        m_skel.setTransform(
            boneIdx, TBoneLocal * TBoneModel.inverse() * transform, Handle::SpaceType::LOCAL );
    }
}

/// Reference: http://www.andreasaristidou.com/publications/papers/FABRIK.pdf
/// This solver tends to be get slow when getting close to max_length
void AnimationComponent::IKsolver( const std::vector<Scalar>& lengths,
                                   std::vector<Ra::Core::Vector3>& p,
                                   const Ra::Core::Vector3& target,
                                   const Scalar max_length ) const {
    if ( ( p.back() - target ).norm() <= max_length && lengths.size() > 1 )
    {
        const auto rootInitPos = p.back();
        while ( !target.isApprox( p.front() ) )
        {
            // Forward
            p[0] = target;
            for ( int i = 1; i < p.size(); ++i )
            {
                const Scalar u = lengths[i - 1] / ( p[i - 1] - p[i] ).norm();
                p[i] = ( 1 - u ) * p[i - 1] + u * p[i];
            }

            // Backward
            p.back() = rootInitPos;
            for ( int i = p.size() - 2; i >= 0; --i )
            {
                const Scalar u = lengths[i] / ( p[i] - p[i + 1] ).norm();
                p[i] = ( 1 - u ) * p[i + 1] + u * p[i];
            }
        }
    } else
    {
        /// Makes it more uncontrolable than anything, but you're the judge
        // for ( int i = p.size() - 2; i >= 0; --i )
        // {
        //     const Scalar u = lengths[i] / ( target - p[i + 1] ).norm();
        //     p[i] = ( 1 - u ) * p[i + 1] + u * target;
        // }
    }
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
    if ( m_animations.empty() )
    {
        return Scalar( 0 );
    }
    if ( m_animations[m_animationID].size() == 0 )
    {
        return Scalar ( 5.0 );
    }
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

    // Importing animations
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_animations.resize( m_firstEditableID + size );
    for ( int i = m_firstEditableID; i < m_animations.size(); ++i )
    {
        auto& anim = m_animations[i];
        input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
        for ( int i = 0; i < size; ++i )
        {
            Scalar timestamp;
            input.read( reinterpret_cast<char*>( &timestamp ), sizeof( timestamp ) );
            Ra::Core::Animation::Pose pose( pose_size );
            for ( int i = 0; i < pose_size; ++i )
            {
                input.read( reinterpret_cast<char*>( pose[i].data() ),
                            sizeof( Ra::Core::Transform ) );
            }
            anim.addKeyPose( pose, timestamp );
        }
    }

    // Importing each animation playzones
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_animsPlayzones.resize( size );
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

    // Importing delta t's
    input.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    m_dt.resize( m_firstEditableID + size );
    input.read( reinterpret_cast<char*>( &m_dt[m_firstEditableID] ), size * sizeof( Scalar ) );
}

void AnimationComponent::saveRDMA( const std::string& filepath ) {
    std::ofstream output{filepath, std::ios::binary | std::ofstream::trunc};

    // Exporting animations
    const size_t pose_size = m_refPose.size();
    output.write( reinterpret_cast<const char*>( &pose_size ), sizeof( pose_size ) );
    size_t size = m_animations.size() - m_firstEditableID;
    output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    for ( int i = m_firstEditableID; i < m_animations.size(); ++i )
    {
        const auto& anim = m_animations[i];
        size = anim.size();
        output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
        for ( int i = 0; i < size; ++i )
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

    // Exporting each animation playzones
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

    // Exporting delta t's
    size = m_dt.size() - m_firstEditableID;
    output.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    output.write( reinterpret_cast<const char*>( &m_dt[m_firstEditableID] ),
                  size * sizeof( Scalar ) );
}

void AnimationComponent::newPlayzone( const std::string& name) {
    const auto& anim = m_animations[m_animationID];
    m_playzoneID = m_animsPlayzones.size();
    if ( m_animations[m_animationID].size() >= 2 )
        m_animsPlayzones[m_animationID].emplace_back( name, anim.keyPose( 0 ).first,
                                                      anim.keyPose( anim.size() - 1 ).first );
    else
        m_animsPlayzones[m_animationID].emplace_back( name, 0.0, 20.0 );
}

void AnimationComponent::removePlayzone( int i ) {
    auto& playzone = m_animsPlayzones[m_animationID];
    if ( playzone.size() <= i )
    {
        return;
    }

    playzone.erase( playzone.begin() + i );
    if ( i <= m_playzoneID )
    {
        setPlayzone( m_playzoneID - 1 );
    }
}

void AnimationComponent::newAnimation() {
    m_animationID = m_animations.size();
    m_animations.emplace_back();
    m_animsPlayzones.emplace_back();
    newPlayzone( "Default playzone" );
    m_dt.emplace_back( 1. / 60 );
}

void AnimationComponent::copyCurrentAnimation() {
    m_animations.emplace_back( m_animations[m_animationID] );
    m_animsPlayzones.emplace_back( m_animsPlayzones[m_animationID] );
    m_dt.emplace_back( m_dt[m_animationID] );
}

void AnimationComponent::removeAnimation( int i ) {
    if ( i < m_firstEditableID || i >= m_animations.size() || m_animations.size() <= 1 )
    {
        return;
    }

    m_animations.erase(m_animations.begin() + i);
    m_animsPlayzones.erase( m_animsPlayzones.begin() + i );
    if ( i <= m_animationID )
    {
        setAnimation( m_animationID - 1 );
    }
    setPlayzone( 0 );
}

void AnimationComponent::setCurrentPose() {
    if ( m_animations[m_animationID].size() >= 1 )
    {
        if ( m_animationTime <= m_animations[m_animationID].keyPose( 0 ).first )
        {
            m_skel.setPose( m_animations[m_animationID].keyPose( 0 ).second,
                            Handle::SpaceType::LOCAL );
        } else if ( m_animationTime > m_animations[m_animationID]
                                          .keyPose( m_animations[m_animationID].size() - 1 )
                                          .first )
        {
            m_skel.setPose( m_animations[m_animationID]
                                .keyPose( m_animations[m_animationID].size() - 1 )
                                .second,
                            Handle::SpaceType::LOCAL );
        } else
        {
        const auto& pose = m_animations[m_animationID].getPose(m_animationTime);
        m_skel.setPose(pose, Handle::SpaceType::LOCAL);
        }
    } else if ( m_animations[m_animationID].size() == 0 )
    { m_skel.setPose( m_refPose, Handle::SpaceType::MODEL ); }

    // update the render objects
    for ( auto& bone : m_boneDrawables )
    {
        bone->update();
    }
}

void AnimationComponent::setCurrentAnimationTime( double timestamp ) {
    m_animationTime = static_cast<Scalar>( timestamp );
    setCurrentPose();
}

double AnimationComponent::getStart() const {
    return std::get<1>( m_animsPlayzones[m_animationID][m_playzoneID] );
}

double AnimationComponent::getEnd() const {
    return std::get<2>( m_animsPlayzones[m_animationID][m_playzoneID] );
}

void AnimationComponent::setStart( double timestamp ) {
    std::get<1>( m_animsPlayzones[m_animationID][m_playzoneID] ) = static_cast<Scalar>( timestamp );
}

void AnimationComponent::setEnd( double timestamp ) {
    std::get<2>( m_animsPlayzones[m_animationID][m_playzoneID] ) = static_cast<Scalar>( timestamp );
}

void AnimationComponent::addKeyPose( double timestamp ) {
    if ( m_animationID < m_firstEditableID )
    {
        copyCurrentAnimation();
        setAnimation( m_animations.size() - 1 );
    }
    m_animations[m_animationID].addKeyPose( m_skel.getPose( Handle::SpaceType::LOCAL ),
                                            static_cast<Scalar>( timestamp ) );
    m_animations[m_animationID].normalize();
}

void AnimationComponent::removeKeyPose( int i ) {
    if ( m_animationID < m_firstEditableID )
    {
        copyCurrentAnimation();
        setAnimation( m_animations.size() - 1 );
    }
        m_animations[m_animationID].removeKeyPose(i);
    setCurrentPose();
    }

void AnimationComponent::setKeyPoseTime( int i, double timestamp ) {
    if ( m_animationID < m_firstEditableID )
    {
        copyCurrentAnimation();
        setAnimation( m_animations.size() - 1 );
    }
    m_animations[m_animationID].setKeyPoseTime( i, static_cast<Scalar>( timestamp ) );
}

void AnimationComponent::updateKeyPose( int id ) {
    if ( m_animationID < m_firstEditableID )
    {
        copyCurrentAnimation();
        setAnimation( m_animations.size() - 1 );
    }
    m_animations[m_animationID].replacePose(id, m_skel.getPose(Handle::SpaceType::LOCAL));
}

void AnimationComponent::offsetKeyPoses( double offset, int first ) {
    if ( m_animationID < m_firstEditableID )
    {
        copyCurrentAnimation();
        setAnimation( m_animations.size() - 1 );
    }
    m_animations[m_animationID].offsetKeyPoses( static_cast<Scalar>( offset ), first );
    
    for ( int i = first; i < m_animsPlayzones[m_animationID].size(); ++i )
    {
        std::get<1>( m_animsPlayzones[m_animationID][i] ) += offset;
        std::get<2>( m_animsPlayzones[m_animationID][i] ) += offset;
    }
}

std::vector<std::string> AnimationComponent::playzonesLabels() const {
    std::vector<std::string> labels;

    labels.reserve( m_animsPlayzones[m_animationID].size() );
    for ( const auto& playzone : m_animsPlayzones[m_animationID] )
    {
        labels.push_back( std::get<0>( playzone ) );
    }
    return labels;
}

int AnimationComponent::animationCount() const {
    return m_animations.size();
}

int AnimationComponent::nonEditableCount() const {
    return m_firstEditableID;
}

std::vector<double> AnimationComponent::keyposesTimes() const {
    std::vector<double> times;
    times.reserve( m_animations[m_animationID].size() );
    
    for ( int i = 0; i < m_animations[m_animationID].size(); ++i )
    {
        times.push_back( static_cast<double>( m_animations[m_animationID].keyPose(i).first ) );
    }

    return times;
}

} // namespace AnimationPlugin
