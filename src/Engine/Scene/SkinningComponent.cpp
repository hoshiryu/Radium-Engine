#include <Engine/Scene/SkinningComponent.hpp>

#include <Core/Animation/PoseOperation.hpp>
#include <Core/Geometry/Normal.hpp>

#include <Core/Animation/DualQuaternionSkinning.hpp>
#include <Core/Animation/HandleWeightOperation.hpp>
#include <Core/Animation/LinearBlendSkinning.hpp>
#include <Core/Animation/RotationCenterSkinning.hpp>
#include <Core/Animation/StretchableTwistableBoneSkinning.hpp>
#include <Core/Geometry/DistanceQueries.hpp>
#include <Core/Geometry/TriangleOperation.hpp>
#include <Core/Utils/Color.hpp>
#include <Core/Utils/Log.hpp>

#include <Engine/RadiumEngine.hpp>
#include <Engine/Data/BlinnPhongMaterial.hpp>
#include <Engine/Data/Mesh.hpp>
#include <Engine/Data/ShaderConfigFactory.hpp>
#include <Engine/Data/Texture.hpp>
#include <Engine/Data/TextureManager.hpp>
#include <Engine/Rendering/RenderObject.hpp>
#include <Engine/Rendering/RenderObjectManager.hpp>
#include <Engine/Rendering/RenderTechnique.hpp>

using Ra::Core::DualQuaternion;
using Ra::Core::Quaternion;
using Ra::Core::Transform;

using Ra::Core::Animation::relativePose;
using Ra::Core::Animation::Pose;
using Ra::Core::Animation::RefPose;
using Ra::Core::Animation::Skeleton;
using Ra::Core::Animation::WeightMatrix;
using Ra::Core::Geometry::TriangleMesh;
using Ra::Core::Geometry::PolyMesh;

using SpaceType = Ra::Core::Animation::HandleArray::SpaceType;

using Ra::Core::Skinning::FrameData;
using Ra::Core::Skinning::RefData;

using namespace Ra::Core::Utils;

namespace Ra {
namespace Engine {
namespace Scene {

bool findDuplicates( const TriangleMesh& mesh,
                     std::vector<Ra::Core::Utils::Index>& duplicatesMap ) {
    bool hasDuplicates = false;
    duplicatesMap.clear();
    const uint numVerts = mesh.vertices().size();
    duplicatesMap.resize( numVerts, Ra::Core::Utils::Index::Invalid() );

    Ra::Core::Vector3Array::const_iterator vertPos;
    Ra::Core::Vector3Array::const_iterator duplicatePos;
    std::vector<std::pair<Ra::Core::Vector3, Ra::Core::Utils::Index>> vertices;

    for ( uint i = 0; i < numVerts; ++i )
    {
        vertices.push_back( std::make_pair( mesh.vertices()[i], Ra::Core::Utils::Index( i ) ) );
    }

    std::sort( vertices.begin(),
               vertices.end(),
               []( std::pair<Ra::Core::Vector3, int> a, std::pair<Ra::Core::Vector3, int> b ) {
                   if ( a.first.x() == b.first.x() )
                   {
                       if ( a.first.y() == b.first.y() )
                           if ( a.first.z() == b.first.z() )
                               return a.second < b.second;
                           else
                               return a.first.z() < b.first.z();
                       else
                           return a.first.y() < b.first.y();
                   }
                   return a.first.x() < b.first.x();
               } );
    // Here vertices contains vertex pos and idx, with equal
    // vertices contiguous, sorted by idx, so checking if current
    // vertex equals the previous one state if its a duplicated
    // vertex position.
    duplicatesMap[vertices[0].second] = vertices[0].second;
    for ( uint i = 1; i < numVerts; ++i )
    {
        if ( vertices[i].first == vertices[i - 1].first )
        {
            duplicatesMap[vertices[i].second] = duplicatesMap[vertices[i - 1].second];
            hasDuplicates                     = true;
        }
        else
        { duplicatesMap[vertices[i].second] = vertices[i].second; }
    }

    return hasDuplicates;
}

TriangleMesh triangulate( const PolyMesh& polyMesh )
{
    TriangleMesh res;
    res.setVertices( polyMesh.vertices() );
    res.setNormals( polyMesh.normals() );
    Ra::Core::AlignedStdVector<Ra::Core::Vector3ui> indices;
    // using the same triangulation as in Ra::Engine::PolyMesh::triangulate
    for ( const auto& face : polyMesh.getIndices() )
    {
        if ( face.size() == 3 ) { indices.push_back( face ); }
        else
        {
            int minus {int( face.size() ) - 1};
            int plus {0};
            while ( plus + 1 < minus )
            {
                if ( ( plus - minus ) % 2 )
                {
                    indices.emplace_back( face[plus], face[plus + 1], face[minus] );
                    ++plus;
                }
                else
                {
                    indices.emplace_back( face[minus], face[plus], face[minus - 1] );
                    --minus;
                }
            }
        }
    }
    res.setIndices( std::move( indices ) );
    return res;
}

void SkinningComponent::initialize() {
    auto compMsg = ComponentMessenger::getInstance();
    // get the current animation data.
    bool hasSkel     = compMsg->canGet<Skeleton>( getEntity(), m_skelName );
    bool hasRefPose  = compMsg->canGet<RefPose>( getEntity(), m_skelName );
    bool hasTriMesh  = compMsg->canGet<TriangleMesh>( getEntity(), m_meshName );
    m_meshIsPoly     = compMsg->canGet<PolyMesh>( getEntity(), m_meshName );

    if ( hasSkel && hasRefPose && (hasTriMesh || m_meshIsPoly) )
    {
        m_renderObjectReader = compMsg->getterCallback<Ra::Core::Utils::Index>( getEntity(), m_meshName );
        m_skeletonGetter     = compMsg->getterCallback<Skeleton>( getEntity(), m_skelName );
        if ( !m_meshIsPoly )
        {
            m_triMeshWriter = compMsg->rwCallback<TriangleMesh>( getEntity(), m_meshName );
        }
        else
        {
            m_polyMeshWriter = compMsg->rwCallback<PolyMesh>( getEntity(), m_meshName );
        }

        // copy mesh triangles and find duplicates for normal computation.
        TriangleMesh mesh;
        if ( !m_meshIsPoly )
        {
            mesh = *m_triMeshWriter();
        }
        else {
            mesh = triangulate( *m_polyMeshWriter() );
        }
        // = const_cast<TriangleMesh*>( m_meshWriter() );
        m_refData.m_referenceMesh.copy( mesh );
        findDuplicates( mesh, m_duplicatesMap );

        // get other data
        m_refData.m_skeleton = *m_skeletonGetter();
        createWeightMatrix();
        m_refData.m_refPose  = m_refData.m_skeleton.getPose( SpaceType::MODEL );
        applyBindMatrices( m_refData.m_refPose );

        // initialize frame data
        m_frameData.m_frameCounter = 0;
        m_frameData.m_doSkinning   = true;
        m_frameData.m_doReset      = false;

        m_frameData.m_previousPos   = m_refData.m_referenceMesh.vertices();
        m_frameData.m_currentPos    = m_refData.m_referenceMesh.vertices();
        m_frameData.m_currentNormal = m_refData.m_referenceMesh.normals();

        m_frameData.m_previousPose = m_refData.m_refPose;
        m_frameData.m_currentPose  = m_refData.m_refPose;
        m_frameData.m_refToCurrentRelPose = relativePose( m_frameData.m_currentPose, m_refData.m_refPose );
        m_frameData.m_prevToCurrentRelPose = relativePose( m_frameData.m_currentPose, m_frameData.m_previousPose );

        // setup comp data
        m_isReady = true;
        m_forceUpdate = true;
        setupSkinningType( m_skinningType );
        setupSkinningType( STBS_LBS ); // ensure weights are present for display

        // prepare RO for skinning weights display
        auto ro         = getRoMgr()->getRenderObject( *m_renderObjectReader() );
        m_baseMaterial  = ro->getMaterial();

        auto attrUV = Data::Mesh::getAttribName( Data::Mesh::VERTEX_TEXCOORD );
        Ra::Core::Geometry::AttribArrayGeometry* geom;
        if ( !m_meshIsPoly )
        {
            geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
        }
        else {
            geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
        }
        if ( geom->hasAttrib( attrUV ) )
        {
            auto handle = geom->getAttribHandle<Ra::Core::Vector3>( attrUV );
            m_baseUV    = geom->getAttrib( handle ).data();
        }

        auto mat  = new Data::BlinnPhongMaterial( "SkinningWeights_Mat" );
        mat->m_kd = Ra::Core::Utils::Color::Skin();
        mat->m_ks = Ra::Core::Utils::Color::White();
        // assign texture
        Data::TextureParameters texParam;
        texParam.name = ":/Resources/Textures/Influence0.png";
        auto tex = Ra::Engine::RadiumEngine::getInstance()->getTextureManager()->getOrLoadTexture(
            texParam );
        mat->addTexture( Data::BlinnPhongMaterial::TextureSemantic::TEX_DIFFUSE, tex );
        m_weightMaterial.reset( mat );
        // compute default weights uv
        showWeightsType( 0 );
    }
}

void SkinningComponent::skin() {
    CORE_ASSERT( m_isReady, "Skinning is not setup" );

    const Skeleton* skel = m_skeletonGetter();

    bool reset = ComponentMessenger::getInstance()->get<bool>( getEntity(), m_skelName );

    // Reset the skin if it wasn't done before
    if ( reset && !m_frameData.m_doReset )
    {
        m_frameData.m_doReset      = true;
        m_frameData.m_frameCounter = 0;
        m_forceUpdate              = true;
    }
    {
        m_frameData.m_currentPose = skel->getPose( SpaceType::MODEL );
        if ( m_smartStretch ){ applySmartStretch(); }
        applyBindMatrices( m_frameData.m_currentPose );
        if ( !Ra::Core::Animation::areEqual( m_frameData.m_currentPose,
                                             m_frameData.m_previousPose ) ||
             m_forceUpdate )
        {
            m_forceUpdate            = false;
            m_frameData.m_doSkinning = true;
            m_frameData.m_frameCounter++;
            m_frameData.m_refToCurrentRelPose = relativePose( m_frameData.m_currentPose, m_refData.m_refPose );
            m_frameData.m_prevToCurrentRelPose = relativePose( m_frameData.m_currentPose, m_frameData.m_previousPose );

            switch ( m_skinningType )
            {
            case LBS:
            {
                Ra::Core::Animation::linearBlendSkinning( m_refData.m_referenceMesh.vertices(),
                                                          m_frameData.m_currentPose,
                                                          m_refData.m_weights,
                                                          m_frameData.m_currentPos );
                break;
            }
            case DQS:
            {
                Ra::Core::AlignedStdVector<DualQuaternion> DQ;
                Ra::Core::Animation::computeDQ(
                    m_frameData.m_currentPose, m_refData.m_weights, DQ );
                Ra::Core::Animation::dualQuaternionSkinning(
                    m_refData.m_referenceMesh.vertices(), DQ, m_frameData.m_currentPos );
                break;
            }
            case COR:
            {
                Ra::Core::Animation::corSkinning( m_refData.m_referenceMesh.vertices(),
                                                  m_frameData.m_currentPose,
                                                  m_refData.m_weights,
                                                  m_refData.m_CoR,
                                                  m_frameData.m_currentPos );
                break;
            }
            case STBS_LBS:
            {
                Ra::Core::Animation::linearBlendSkinningSTBS( m_refData.m_referenceMesh.vertices(),
                                                              m_frameData.m_refToCurrentRelPose,
                                                              *skel,
                                                              m_refData.m_skeleton,
                                                              m_refData.m_weights,
                                                              m_weightSTBS,
                                                              m_frameData.m_currentPos );
                break;
            }
            case STBS_DQS:
            {
                Ra::Core::AlignedStdVector<DualQuaternion> DQ;
                Ra::Core::Animation::computeDQSTBS(
                    m_frameData.m_refToCurrentRelPose, *skel, m_refData.m_skeleton, m_refData.m_weights, m_weightSTBS, DQ );
                Ra::Core::Animation::dualQuaternionSkinning(
                    m_refData.m_referenceMesh.vertices(), DQ, m_frameData.m_currentPos );
                break;
            }
            }
        }
    }
}

void uniformNormal( const Ra::Core::Vector3Array& p,
                    const Ra::Core::AlignedStdVector<Ra::Core::Vector3ui>& T,
                    const std::vector<Ra::Core::Utils::Index>& duplicateTable,
                    Ra::Core::Vector3Array& normal ) {
    const uint N = p.size();
    normal.clear();
    normal.resize( N, Ra::Core::Vector3::Zero() );

    for ( const auto& t : T )
    {
        const Ra::Core::Utils::Index i = duplicateTable.at( t( 0 ) );
        const Ra::Core::Utils::Index j = duplicateTable.at( t( 1 ) );
        const Ra::Core::Utils::Index k = duplicateTable.at( t( 2 ) );
        const Ra::Core::Vector3 triN   = Ra::Core::Geometry::triangleNormal( p[i], p[j], p[k] );
        if ( !triN.allFinite() ) { continue; }
        normal[i] += triN;
        normal[j] += triN;
        normal[k] += triN;
    }

#pragma omp parallel for
    for ( int i = 0; i < N; ++i )
    {
        if ( !normal[i].isApprox( Ra::Core::Vector3::Zero() ) ) { normal[i].normalize(); }
    }

#pragma omp parallel for
    for ( int i = 0; i < N; ++i )
    {
        normal[i] = normal[duplicateTable[i]];
    }
}

void SkinningComponent::endSkinning() {
    if ( m_frameData.m_doSkinning )
    {
        Ra::Core::Geometry::AttribArrayGeometry* geom;
        if ( !m_meshIsPoly )
        {
            geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
        }
        else {
            geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
        }
        Ra::Core::Vector3Array& vertices = geom->verticesWithLock();
        Ra::Core::Vector3Array& normals  = geom->normalsWithLock();

        vertices = m_frameData.m_currentPos;

        // FIXME: normals should be computed by the Skinning method!
        uniformNormal( vertices, m_refData.m_referenceMesh.getIndices(), m_duplicatesMap, normals );

        std::swap( m_frameData.m_previousPose, m_frameData.m_currentPose );
        std::swap( m_frameData.m_previousPos, m_frameData.m_currentPos );

        m_frameData.m_doReset    = false;
        m_frameData.m_doSkinning = false;

        geom->verticesUnlock();
        geom->normalsUnlock();
    }
}

void SkinningComponent::handleSkinDataLoading( const Ra::Core::Asset::HandleData* data,
                                               const std::string& meshName,
                                               const Ra::Core::Transform& meshFrame ) {
    m_skelName = data->getName();
    m_meshName = meshName;
    setupIO( meshName );
    m_meshFrameInv = meshFrame.inverse();
    for ( const auto& bone : data->getComponentData() )
    {
        auto it_w = bone.m_weights.find( meshName );
        if ( it_w != bone.m_weights.end() )
        {
            m_loadedWeights[bone.m_name] = it_w->second;
            auto it_b                    = bone.m_bindMatrices.find( meshName );
            if ( it_b != bone.m_bindMatrices.end() )
            { m_loadedBindMatrices[bone.m_name] = it_b->second; }
            else
            {
                LOG( logWARNING ) << "Bone " << bone.m_name
                                  << " has skinning weights but no bind matrix. Using Identity.";
                m_loadedBindMatrices[bone.m_name] = Ra::Core::Transform::Identity();
            }
        }
    }
}

void SkinningComponent::createWeightMatrix() {
    m_refData.m_weights.resize( int( m_refData.m_referenceMesh.vertices().size() ),
                                m_refData.m_skeleton.size() );
    std::vector<Eigen::Triplet<Scalar>> triplets;
    for ( uint col = 0; col < m_refData.m_skeleton.size(); ++col )
    {
        std::string boneName = m_refData.m_skeleton.getLabel( col );
        auto it              = m_loadedWeights.find( boneName );
        if ( it != m_loadedWeights.end() )
        {
            const auto& W   = it->second;
            for ( uint i = 0; i < W.size(); ++i )
            {
                const auto& w = W[i];
                int row{int( w.first )};
                CORE_ASSERT( row < m_refData.m_weights.rows(),
                             "Weights are incompatible with mesh." );
                triplets.push_back( {row, int( col ), w.second} );
            }
            m_refData.m_bindMatrices[col] = m_loadedBindMatrices[boneName];
        }
    }
    m_refData.m_weights.setFromTriplets( triplets.begin(), triplets.end() );

    Ra::Core::Animation::checkWeightMatrix( m_refData.m_weights, false, true );

    if ( Ra::Core::Animation::normalizeWeights( m_refData.m_weights, true ) )
    { LOG( logINFO ) << "Skinning weights have been normalized"; }
}

void SkinningComponent::applySmartStretch() {
    auto refSkel = m_refData.m_skeleton;
    auto* currentSkel = m_skeletonGetter();
    for( int i = 0 ; i < int( refSkel.size() ) ; ++i )
    {
        if( refSkel.m_graph.isRoot( i ) )
        {
            continue;
        }

        // get transforms
        const uint parent = refSkel.m_graph.parents()[i];
        const auto& boneModel = currentSkel->getTransform( i, SpaceType::MODEL );
        const auto& parentModel = currentSkel->getTransform( parent, SpaceType::MODEL );
        const auto& parentRef = refSkel.getTransform( parent, SpaceType::MODEL );
        const auto parentT = parentModel * parentRef.inverse( Eigen::Affine );

        // do nothing for multi-siblings
        if( refSkel.m_graph.children()[parent].size() > 1 )
        {
            refSkel.setTransform( parent, parentModel, SpaceType::MODEL );
            continue;
        }

        // rotate parent to align with bone
        Ra::Core::Vector3 A;
        Ra::Core::Vector3 B;
        refSkel.getBonePoints( parent, A, B );
        B = parentT * B;
        Ra::Core::Vector3 B_ = boneModel.translation();
        auto q = Ra::Core::Quaternion::FromTwoVectors( ( B - A ), ( B_ - A ) );
        Ra::Core::Transform R( q );
        R.pretranslate( A );
        R.translate( -A );
        refSkel.setTransform( parent, R * parentModel, SpaceType::MODEL );
    }
    m_frameData.m_currentPose = refSkel.getPose( SpaceType::MODEL );
}

void SkinningComponent::applyBindMatrices( Ra::Core::Animation::Pose& pose ) const {
    for ( const auto& bM : m_refData.m_bindMatrices )
    {
        pose[bM.first] = m_meshFrameInv * pose[bM.first] * bM.second;
    }
}

void SkinningComponent::setupIO( const std::string& id ) {
    auto compMsg = ComponentMessenger::getInstance();
    auto wOut = std::bind( &SkinningComponent::getWeightsOutput, this );
    compMsg->registerOutput<Ra::Core::Animation::WeightMatrix>( getEntity(), this, id, wOut );

    auto refData = std::bind( &SkinningComponent::getRefData, this );
    compMsg->registerOutput<Ra::Core::Skinning::RefData>( getEntity(), this, id, refData );

    auto frameData = std::bind( &SkinningComponent::getFrameData, this );
    compMsg->registerOutput<FrameData>( getEntity(), this, id, frameData );
}

void SkinningComponent::setSkinningType( SkinningType type ) {
    m_skinningType = type;
    if ( m_isReady )
    {
        setupSkinningType( type );
        m_forceUpdate = true;
    }
}

const Ra::Core::Animation::WeightMatrix* SkinningComponent::getWeightsOutput() const {
    return &m_refData.m_weights;
}

const std::string SkinningComponent::getMeshName() const {
    return m_meshName;
}

const std::string SkinningComponent::getSkeletonName() const {
    return m_skeletonGetter()->getName();
}

void SkinningComponent::setupSkinningType( SkinningType type ) {
    CORE_ASSERT( m_isReady, "component is not ready" );
    switch ( type )
    {
    case LBS:
        break;
    case DQS:
        break;
    case COR:
    {
        if ( m_refData.m_CoR.empty() ) { Ra::Core::Animation::computeCoR( m_refData ); }
        break;
    }
    case STBS_DQS:
        [[fallthrough]];
    case STBS_LBS:
    {
        if ( m_weightSTBS.size() == 0 )
        {
            Ra::Core::Animation::computeSTBS_weights( m_refData.m_referenceMesh.vertices(),
                                                      m_refData.m_skeleton, m_weightSTBS );
        }
    }
    } // end of switch.
}

void SkinningComponent::setSmartStretch( bool on ){
    m_smartStretch = on;
}

void SkinningComponent::showWeights( bool on ) {
    m_showingWeights   = on;
    auto ro            = getRoMgr()->getRenderObject( *m_renderObjectReader() );
    auto attrUV        = Data::Mesh::getAttribName( Data::Mesh::VERTEX_TEXCOORD );
    Ra::Core::Utils::AttribHandle<Ra::Core::Vector3> handle;

    Ra::Core::Geometry::AttribArrayGeometry* geom;
    if ( !m_meshIsPoly )
    {
        geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
    }
    else {
        geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
    }

    if ( m_showingWeights )
    {
        ro->setMaterial( m_weightMaterial );
        ro->getRenderTechnique()->setParametersProvider( m_weightMaterial );
        // get the UV attrib handle, will create it if not there.
        handle = geom->addAttrib<Ra::Core::Vector3>( attrUV );
        geom->getAttrib( handle ).setData( m_weightsUV );
    }
    else
    {
        ro->setMaterial( m_baseMaterial );
        ro->getRenderTechnique()->setParametersProvider( m_baseMaterial );

        handle = geom->getAttribHandle<Ra::Core::Vector3>( attrUV );
        // if the UV attrib existed before, reset it, otherwise remove it.
        if ( m_baseUV.size() > 0 ) { geom->getAttrib( handle ).setData( m_baseUV ); }
        else
        { geom->removeAttrib( handle ); }
    }
    m_forceUpdate = true;
}

void SkinningComponent::showWeightsType( int type ) {
    if ( !m_showingWeights ) return;
    const uint size = m_frameData.m_currentPos.size();
    m_weightsUV.resize( size, Ra::Core::Vector3::Zero() );
    m_weightType = type;
    if ( type == 0 )
    {
#pragma omp parallel for
        for ( int i = 0; i < int( size ); ++i )
        {
            m_weightsUV[i][0] = m_refData.m_weights.coeff( i, m_weightBone );
        }
    }
    else
    {
#pragma omp parallel for
        for ( int i = 0; i < int( size ); ++i )
        {
            m_weightsUV[i][0] = m_weightSTBS.coeff( i, m_weightBone );
        }
    }
    showWeights( true );
}

void SkinningComponent::setWeightBone( uint bone ) {
    m_weightBone = bone;
    showWeightsType( m_weightType );
}

} // namespace Scene
} // namespace Engine
} // namespace Ra
