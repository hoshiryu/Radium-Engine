#include <Engine/Scene/SkeletonBasedAnimationSystem.hpp>

#include <iostream>
#include <string>

#include <Core/Animation/KeyFramedValueController.hpp>
#include <Core/Asset/FileData.hpp>
#include <Core/Math/Math.hpp>
#include <Core/Tasks/Task.hpp>
#include <Core/Tasks/TaskQueue.hpp>

#include <Engine/FrameInfo.hpp>
#include <Engine/RadiumEngine.hpp>
#include <Engine/Scene/SkeletonComponent.hpp>
#include <Engine/Scene/SkinningComponent.hpp>

using namespace Ra::Core::Animation;

namespace Ra {
namespace Engine {
namespace Scene {

SkeletonBasedAnimationSystem::SkeletonBasedAnimationSystem() :
    System(),
    m_xrayOn( false ) {}

// System Interface

void SkeletonBasedAnimationSystem::generateTasks( Ra::Core::TaskQueue* taskQueue,
                                                  const Ra::Engine::FrameInfo& frameInfo ) {
    for ( auto compEntry : m_components )
    {
        // deal with AnimationComponents
        if ( compEntry.second->getName().compare( 0, 3, "AC_" ) == 0 )
        {
            auto animComp = static_cast<SkeletonComponent*>( compEntry.second );
            if ( !Ra::Core::Math::areApproxEqual( m_time, frameInfo.m_animationTime ) )
            {
                auto animFunc = std::bind( &SkeletonComponent::update, animComp, frameInfo.m_animationTime );
                auto animTask = new Ra::Core::FunctionTask(
                    animFunc, "AnimatorTask_" + animComp->getSkeleton()->getName() );
                taskQueue->registerTask( animTask );
            }
            else
            {
                auto animFunc = std::bind( &SkeletonComponent::updateDisplay, animComp ) ;
                auto animTask = new Ra::Core::FunctionTask(
                    animFunc, "AnimatorTask_" + animComp->getSkeleton()->getName() );
                taskQueue->registerTask( animTask );
            }
        }
        // deal with SkinningComponents
        else if ( compEntry.second->getName().compare( 0, 4, "SkC_" ) == 0 )
        {
            auto skinComp = static_cast<SkinningComponent*>( compEntry.second );
            auto skinFunc = std::bind( &SkinningComponent::skin, skinComp );
            auto skinTask =
                new Ra::Core::FunctionTask( skinFunc, "SkinnerTask_" + skinComp->getMeshName() );
            auto endFunc = std::bind( &SkinningComponent::endSkinning, skinComp );
            auto endTask =
                new Ra::Core::FunctionTask( endFunc, "SkinnerEndTask_" + skinComp->getMeshName() );

            Ra::Core::TaskQueue::TaskId skinTaskId = taskQueue->registerTask( skinTask );
            Ra::Core::TaskQueue::TaskId endTaskId  = taskQueue->registerTask( endTask );
            taskQueue->addPendingDependency( "AnimatorTask_" + skinComp->getSkeletonName(),
                                             skinTaskId );
            taskQueue->addDependency( skinTaskId, endTaskId );
        }
    }
    m_time = frameInfo.m_animationTime;
}

void SkeletonBasedAnimationSystem::handleAssetLoading( Entity* entity,
                                                       const Ra::Core::Asset::FileData* fileData ) {
    auto skelData = fileData->getHandleData();
    auto animData = fileData->getAnimationData();

    // deal with AnimationComponents
    for ( const auto& skel : skelData )
    {
        auto component = new SkeletonComponent( "AC_" + skel->getName(), entity );
        component->handleSkeletonLoading( skel );
        component->handleAnimationLoading( animData );
        component->setXray( m_xrayOn );
        registerComponent( entity, component );
    }

    // deal with SkinningComponents
    auto geomData = fileData->getGeometryData();
    if ( geomData.size() > 0 && skelData.size() > 0 )
    {
        for ( const auto& geom : geomData )
        {
            // look for a skeleton skinning this mesh
            // warning: there should be at most one such skeleton!
            auto it = std::find_if( skelData.begin(), skelData.end(), [&geom]( const auto& skel ) {
                return std::find_if( skel->getBindMeshes().begin(),
                                     skel->getBindMeshes().end(),
                                     [&geom]( const auto& meshName ) {
                                         return meshName == geom->getName();
                                     } ) != skel->getBindMeshes().end();
            } );
            if ( it != skelData.end() )
            {
                const auto& skel             = *it;
                SkinningComponent* component = new SkinningComponent(
                    "SkC_" + geom->getName(), SkinningComponent::LBS, entity );
                component->handleSkinDataLoading( skel, geom->getName(), geom->getFrame() );
                registerComponent( entity, component );
            }
        }
    }
}

// Skeleton display

void SkeletonBasedAnimationSystem::setXray( bool on ) {
    m_xrayOn = on;
    for ( const auto& comp : m_components )
    {
        if ( comp.second->getName().compare( 0, 3, "AC_" ) == 0 )
        { static_cast<SkeletonComponent*>( comp.second )->setXray( on ); }
    }
}

bool SkeletonBasedAnimationSystem::isXrayOn() {
    return m_xrayOn;
}

void SkeletonBasedAnimationSystem::toggleSkeleton( const bool status ) {
    for ( const auto& comp : m_components )
    {
        if ( comp.second->getName().compare( 0, 3, "AC_" ) == 0 )
        { static_cast<SkeletonComponent*>( comp.second )->toggleSkeleton( status ); }
    }
}

// Animation parameters

void SkeletonBasedAnimationSystem::setAnimationSpeed( const Scalar value ) {
    for ( const auto& comp : m_components )
    {
        if ( comp.second->getName().compare( 0, 3, "AC_" ) == 0 )
        { static_cast<SkeletonComponent*>( comp.second )->setSpeed( value ); }
    }
}

void SkeletonBasedAnimationSystem::autoRepeat( const bool status ) {
    for ( const auto& comp : m_components )
    {
        if ( comp.second->getName().compare( 0, 3, "AC_" ) == 0 )
        { static_cast<SkeletonComponent*>( comp.second )->autoRepeat( status ); }
    }
}

void SkeletonBasedAnimationSystem::pingPong( const bool status ) {
    for ( const auto& comp : m_components )
    {
        if ( comp.second->getName().compare( 0, 3, "AC_" ) == 0 )
        { static_cast<SkeletonComponent*>( comp.second )->pingPong( status ); }
    }
}

Scalar SkeletonBasedAnimationSystem::getAnimationTime( const ItemEntry& entry ) const {
    if ( entry.isValid() )
    {
        // If entry is/has an existing animation component, we return this one's time
        auto it =
            std::find_if( m_components.begin(), m_components.end(), [&entry]( const auto& comp ) {
                return comp.first == entry.m_entity;
            } );
        if ( it != m_components.end() )
        { return static_cast<SkeletonComponent*>( it->second )->getAnimationTime(); }
    }
    return 0_ra;
}

void SkeletonBasedAnimationSystem::showWeights( bool on ) {
    for ( auto& compEntry : m_components )
    {
        if ( compEntry.second->getName().compare( 0, 4, "SkC_" ) == 0 )
        { static_cast<SkinningComponent*>( compEntry.second )->showWeights( on ); }
    }
}

void SkeletonBasedAnimationSystem::showWeightsType( int type ) {
    for ( auto& compEntry : m_components )
    {
        if ( compEntry.second->getName().compare( 0, 4, "SkC_" ) == 0 )
        { static_cast<SkinningComponent*>( compEntry.second )->showWeightsType( type ); }
    }
}

} // namespace Scene
} // namespace Engine
} // namespace Ra
