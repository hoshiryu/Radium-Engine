#include <Engine/Scene/SkinningComponent.hpp>

#include <Core/Animation/PoseOperation.hpp>
#include <Core/Geometry/Normal.hpp>

#include <Core/Animation/DualQuaternionSkinning.hpp>
#include <Core/Animation/HandleWeightOperation.hpp>
#include <Core/Animation/LinearBlendSkinning.hpp>
#include <Core/Animation/RotationCenterSkinning.hpp>
#include <Core/Animation/StretchableTwistableBoneSkinning.hpp>
#include <Core/Geometry/Adjacency.hpp>
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

#include <Core/Utils/Timer.hpp>

using namespace Ra::Core;

using Geometry::AttribArrayGeometry;
using Geometry::PolyMesh;
using Geometry::TriangleMesh;

using namespace Animation;
using SpaceType = HandleArray::SpaceType;

using namespace Skinning;

using namespace Utils;

static bool g_standardLBS = true;

namespace Ra {
namespace Engine {
namespace Scene {

static const auto tangentName = Data::Mesh::getAttribName( Data::Mesh::VERTEX_TANGENT );
static const auto bitangentName = Data::Mesh::getAttribName( Data::Mesh::VERTEX_BITANGENT );

bool findDuplicates( const TriangleMesh& mesh,
                     std::vector<Index>& duplicatesMap ) {
    bool hasDuplicates = false;
    duplicatesMap.clear();
    const uint numVerts = mesh.vertices().size();
    duplicatesMap.resize( numVerts, Index::Invalid() );

    Vector3Array::const_iterator vertPos;
    Vector3Array::const_iterator duplicatePos;
    std::vector<std::pair<Vector3, Index>> vertices;

    for ( uint i = 0; i < numVerts; ++i )
    {
        vertices.push_back( std::make_pair( mesh.vertices()[i], Index( i ) ) );
    }

    std::sort( vertices.begin(),
               vertices.end(),
               []( std::pair<Vector3, int> a, std::pair<Vector3, int> b ) {
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
    res.copyAllAttributes( polyMesh );
    AlignedStdVector<Vector3ui> indices;
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
        m_renderObjectReader = compMsg->getterCallback<Index>( getEntity(), m_meshName );
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
        m_refData.m_referenceMesh.copy( mesh );
        // make sure we have the tangents and bitangents
        // FIXME: Is it ok to suppose that the mesh has normals ?
        bool hasTangents = m_refData.m_referenceMesh.hasAttrib( tangentName );
        bool hasBitangents = m_refData.m_referenceMesh.hasAttrib( bitangentName );
        if ( !hasTangents && !hasBitangents )
        {
            const auto& normals = m_refData.m_referenceMesh.normals();
            const auto N = normals.size();
            Vector3Array tangents( N );
            Vector3Array bitangents( N );
#pragma omp parallel for
            for ( int i = 0; i < int( N ); ++i )
            {
                Math::getOrthogonalVectors( normals[i], tangents[i], bitangents[i] );
            }
            m_refData.m_referenceMesh.addAttrib( tangentName, tangents );
            m_refData.m_referenceMesh.addAttrib( bitangentName, bitangents );
        }
        else if ( !hasTangents )
        {
            const auto& normals = m_refData.m_referenceMesh.normals();
            const auto& bitangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName ) ).data();
            const auto N = normals.size();
            Vector3Array tangents( N );
#pragma omp parallel for
            for ( int i = 0; i < int( N ); ++i )
            {
                tangents[i] = bitangents[i].cross( normals[i] );
            }
            m_refData.m_referenceMesh.addAttrib( tangentName, tangents );
        }
        else if ( !hasBitangents )
        {
            const auto& normals = m_refData.m_referenceMesh.normals();
            const auto& tangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName ) ).data();
            const auto N = normals.size();
            Vector3Array bitangents( N );
#pragma omp parallel for
            for ( int i = 0; i < int( N ); ++i )
            {
                bitangents[i] = normals[i].cross( tangents[i] );
            }
            m_refData.m_referenceMesh.addAttrib( bitangentName, bitangents );
        }
        findDuplicates( m_refData.m_referenceMesh, m_duplicatesMap );

        // get other data
        m_refData.m_skeleton = *m_skeletonGetter();
        createWeightMatrix();
        m_refData.m_refPose  = m_refData.m_skeleton.getPose( SpaceType::MODEL );
        applyBindMatrices( m_refData.m_refPose );

        // compute the per-vertex deform factors
        {
            auto angle = []( Eigen::Ref<Vector3> p1, Eigen::Ref<Vector3> p2 )
            {
                Scalar w = p1.norm()*p2.norm();
                if(w==0) return Scalar(-1);
                Scalar t = (p1.dot(p2))/w;
                if(t>1) t = 1;
                else if(t<-1) t = -1;
                return std::acos(t);
            };

            auto start = Clock::now();
#if 0 // 749494 us with first implem (Vector4 and Matrix4), 128621 us with theirs
            const auto& vertices = m_refData.m_referenceMesh.vertices();
            // prepare per-vertex influencing bone list
            const auto& W = m_refData.m_weights;
            std::vector<std::vector<uint>> vbones( vertices.size() );
#pragma omp parallel for
            for ( int i = 0; i < vertices.size(); ++i )
            {
                const Eigen::SparseVector<Scalar> bones = W.row( i );
                for ( Eigen::SparseVector<Scalar>::InnerIterator it(bones); it; ++it )
                {
                    if ( it.value() > 1e-7 )
                    {
                        vbones[i].emplace_back( it.index() );
                    }
                }
            }
            // compute per-triangle constant homogeneous vector a[s]
            const auto& triangles = m_refData.m_referenceMesh.getIndices();
            std::vector<std::map<uint,Vector3>> AS( triangles.size() );
#pragma omp parallel for
            for ( int t = 0; t < triangles.size(); ++t )
            {
                const auto& T = triangles[t];
                // get the set B of bones influencing the vertices of T
                std::vector<uint> B;
                {
                    std::set<uint> bones;
                    for ( int v = 0; v < 3; ++v )
                    {
                        bones.insert( vbones[T[v]].begin(), vbones[T[v]].end() );
                    }
                    B.resize( bones.size() );
                    std::copy( bones.begin(), bones.end(), B.begin() );
                }
                // compute A for each bone s
                const auto e1 = vertices[T[1]] - vertices[T[0]];
                const auto e2 = vertices[T[2]] - vertices[T[0]];
                const auto n = e2.cross( e1 );
                Matrix3 M;
                M.row( 0 ) = e1.transpose();
                M.row( 1 ) = e2.transpose();
                M.row( 2 ) = n.transpose();
                M = M.inverse().eval();
                for ( const auto& s : B )
                {
                    Scalar dw0 = W.coeff( T[1], s ) - W.coeff( T[0], s );
                    Scalar dw1 = W.coeff( T[2], s ) - W.coeff( T[0], s );
                    AS[t][s] = M * Vector3( dw0, dw1, 0 );
                }
            }
            // compute per-vertex a[s] through cotangent weights
            const auto COT = Geometry::cotangentWeightAdjacency( vertices, triangles );
            std::vector<std::map<uint,Vector3>> AIS( vertices.size() );
            std::vector<Scalar> sums( vertices.size() );
#pragma omp parallel for
            for ( int t = 0; t < triangles.size(); ++t )
            {
                const auto& T = triangles[t];
                Vector3 e1 = vertices[T[1]]-vertices[T[0]];
                Vector3 e2 = vertices[T[2]]-vertices[T[0]];
                const Scalar area = e2.cross(e1).norm();
                for ( int v = 0; v < 3; ++v )
                {
                    const Scalar w = COT.coeff( T[v], T[( v + 1 ) % 3] );
//                    Vector3 e1 = vertices[T[(v+1)%3]]-vertices[T[v]];
//                    Vector3 e2 = vertices[T[(v+2)%3]]-vertices[T[v]];
//                    const Scalar w = angle(e1, e2) * area;
                    sums[T[v]] += w;
                    for ( const auto& s : vbones[T[v]] )
#pragma omp critical
                    {
                        AIS[T[v]][s] += w * AS[t][s];
                    }
                }
            }
#pragma omp parallel for
            for ( int v = 0; v < vertices.size(); ++v )
            {
                const Scalar sum = COT.row( v ).sum();
//                auto sum = sums[v];
                for ( auto& a : AIS[v] )
                {
                    a.second /= sum;
                }
            }
            // compute per-vertex, per-bone Alpha-Beta
            const auto& tangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName ) ).data();
            const auto& bitangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName ) ).data();
            m_refData.m_alphaBeta.resize( vertices.size() );
#pragma omp parallel for
            for ( int v = 0; v < vertices.size(); ++v )
            {
                m_refData.m_alphaBeta[v].reserve( vbones[v].size() );
                for ( auto s : vbones[v] )
                {
                    m_refData.m_alphaBeta[v].emplace_back( std::make_tuple( s,
                        AIS[v][s].dot( tangents[v] ), AIS[v][s].dot( bitangents[v] ) ) );
                }
            }

#else // 22223 us
            // Here is the authors' code refactored

            // Here we derive from the authors' code
            const auto& vertices = m_refData.m_referenceMesh.vertices();
            const auto& tangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName ) ).data();
            const auto& bitangents = m_refData.m_referenceMesh.getAttrib(
                m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName ) ).data();
            // prepare per-vertex influencing bone list
            const auto& W = m_refData.m_weights;
            std::vector<std::vector<uint>> vbones( vertices.size() );
#pragma omp parallel for
            for ( int i = 0; i < vertices.size(); ++i )
            {
                const Eigen::SparseVector<Scalar> bones = W.row( i );
                for ( Eigen::SparseVector<Scalar>::InnerIterator it(bones); it; ++it )
                {
                    if ( it.value() > 1e-7 )
                    {
                        vbones[i].emplace_back( it.index() );
                    }
                }
            }
            // initialize all alphaBeta to 0 and weight sum to 0
            m_refData.m_alphaBeta.resize( vertices.size() );
            std::vector<std::vector<Scalar>> summator( vertices.size() );
#pragma omp parallel for
            for ( int v = 0; v < vertices.size(); ++v )
            {
                for ( auto s : vbones[v] )
                {
                    m_refData.m_alphaBeta[v].emplace_back( std::make_tuple( s, 0, 0 ) );
                }
                summator[v].resize( vbones[v].size(), 0 );
            }
            const auto& face = m_refData.m_referenceMesh.getIndices();
            int nBones = m_refData.m_skeleton.size();
#pragma omp parallel for
            for (unsigned int ff=0; ff<face.size(); ff++)
            {
                int pi[3];
                pi[0] = face[ff][0];
                pi[1] = face[ff][1];
                pi[2] = face[ff][2];

                Matrix3 m;
                Vector3 e0 = vertices[pi[1]] - vertices[pi[0]];
                Vector3 e1 = vertices[pi[2]] - vertices[pi[0]];
                Vector3 n = e1.cross( e0 );

                float faceArea = n.norm();

                m.row( 0 ) = e0.transpose();
                m.row( 1 ) = e1.transpose();
                m.row( 2 ) = n.transpose();
                m = m.inverse().eval();

                std::vector< bool > weightsDone(nBones , false );

                for (int w=0; w<3; w++) {

                    for (int r=0; r<vbones[pi[w]].size(); r++) {

                        int bi = vbones[pi[w]][r];
                        if ((bi<0) || ( W.coeff( pi[w], bi ) ==0)) continue;
                        if ( weightsDone[bi] ) continue;

                        weightsDone[bi] = true;

                        float dw0 = W.coeff( pi[1], bi ) - W.coeff( pi[0], bi );
                        float dw1 = W.coeff( pi[2], bi ) - W.coeff( pi[0], bi );

                        if ((dw0==0)&&(dw1==0)) continue;

                        Vector3 faceWeightGradient = ( m * Vector3( dw0, dw1, 0 ) ) ;

                        // add it to all verteces
                        for (int z=0; z<3; z++)
                        {
                            auto it = std::find_if( m_refData.m_alphaBeta[pi[z]].begin(),
                                                    m_refData.m_alphaBeta[pi[z]].end(),
                                    [&bi]( const auto& tuple ) { return std::get<0>( tuple ) == bi; } );
                            if ( it == m_refData.m_alphaBeta[pi[z]].end() ) { continue; }
                            Vector3 e1 = vertices[ pi[(z+1)%3] ] - vertices[ pi[z] ];
                            Vector3 e2 = vertices[ pi[(z+2)%3] ] - vertices[ pi[z] ];
                            Scalar wedgeAngle = angle(e1,e2) * faceArea;
#pragma omp critical
                            {
                                auto& [s,a,b] = *it;
                                a += tangents[ pi[z] ].dot( faceWeightGradient ) * wedgeAngle;
                                b += bitangents[ pi[z] ].dot( faceWeightGradient ) * wedgeAngle;
                                summator[ pi[z] ][ std::distance( m_refData.m_alphaBeta[pi[z]].begin(), it ) ] += wedgeAngle;
                            }
                        }
                    }
                }
            }

            for (unsigned int vi=0; vi<vertices.size(); vi++)
            {
                for (uint k=0; k<vbones[vi].size(); k++) {
                    if (summator[ vi ][k]) {
                        auto& [s,a,b] = m_refData.m_alphaBeta[vi][k];
                        a /= summator[ vi ][k];
                        b /= summator[ vi ][k];
                    }
                }
            }


            /* THIS MIGHT NOT BE MANDATORY SINCE WE SLIGHTLY DIFFER FOR THE EVAL
            // shift 1st three
            for (unsigned int vi=0; vi<vertices.size(); vi++){
                //vert[vi].orderBoneSlotsWithWeights();
                for (uint k=0; k<3; k++) {
                   vert[vi].deformFactorsTang[k] = vert[vi].deformFactorsTang[k+1];
                   vert[vi].deformFactorsBtan[k] = vert[vi].deformFactorsBtan[k+1];
                }
            }
            */
#endif
            std::cout << getIntervalMicro( start, Clock::now() ) << std::endl;
            for ( int i=0; i<vertices.size(); ++i)
                for ( auto [s,a,b] : m_refData.m_alphaBeta[i] )
                    std::cout << i << ": " << a << " " << b << std::endl;
        }

        // initialize frame data
        m_frameData.m_frameCounter = 0;
        m_frameData.m_doSkinning   = true;
        m_frameData.m_doReset      = false;

        m_frameData.m_previousPos   = m_refData.m_referenceMesh.vertices();
        m_frameData.m_currentPos    = m_refData.m_referenceMesh.vertices();
        m_frameData.m_currentNormal = m_refData.m_referenceMesh.normals();
        m_frameData.m_currentTangent = m_refData.m_referenceMesh.getAttrib(
            m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName ) ).data();
        m_frameData.m_currentBitangent = m_refData.m_referenceMesh.getAttrib(
            m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName ) ).data();

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
        AttribArrayGeometry* geom;
        if ( !m_meshIsPoly )
        {
            geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
        }
        else {
            geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
        }
        if ( geom->hasAttrib( attrUV ) )
        {
            auto handle = geom->getAttribHandle<Vector3>( attrUV );
            m_baseUV    = geom->getAttrib( handle ).data();
        }

        auto mat  = new Data::BlinnPhongMaterial( "SkinningWeights_Mat" );
        mat->m_kd = Color::Skin();
        mat->m_ks = Color::White();
        // assign texture
        Data::TextureParameters texParam;
        texParam.name = ":/Textures/Influence0.png";
        auto tex = RadiumEngine::getInstance()->getTextureManager()->getOrLoadTexture(
            texParam );
        mat->addTexture( Data::BlinnPhongMaterial::TextureSemantic::TEX_DIFFUSE, tex );
        m_weightMaterial.reset( mat );
        // compute default weights uv
        showWeightsType( STANDARD );
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
    m_frameData.m_currentPose = skel->getPose( SpaceType::MODEL );
    if ( m_smartStretch ){ applySmartStretch(); }
    applyBindMatrices( m_frameData.m_currentPose );
    if ( !areEqual( m_frameData.m_currentPose, m_frameData.m_previousPose ) ||
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
            if ( g_standardLBS )
            {
                linearBlendSkinning( m_refData.m_referenceMesh.vertices(),
                                     m_frameData.m_currentPose,
                                     m_refData.m_weights,
                                     m_frameData.m_currentPos );
            }
            else {
                const auto& tHandle = m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName );
                const auto& bHandle = m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName );
                accurateLightingLBS( m_refData,
                                     m_refData.m_referenceMesh.getAttrib( tHandle ).data(),
                                     m_refData.m_referenceMesh.getAttrib( bHandle ).data(),
                                     m_frameData );
            }
            break;
        }
        case DQS:
        {
            AlignedStdVector<DualQuaternion> DQ;
            computeDQ( m_frameData.m_currentPose, m_refData.m_weights, DQ );
            if ( g_standardLBS )
            {
                dualQuaternionSkinning( m_refData.m_referenceMesh.vertices(), DQ, m_frameData.m_currentPos );
            }
            else {
                const auto& tHandle = m_refData.m_referenceMesh.getAttribHandle<Vector3>( tangentName );
                const auto& bHandle = m_refData.m_referenceMesh.getAttribHandle<Vector3>( bitangentName );
                accurateLightingDQS( m_refData, DQ,
                                     m_refData.m_referenceMesh.getAttrib( tHandle ).data(),
                                     m_refData.m_referenceMesh.getAttrib( bHandle ).data(),
                                     m_frameData );
            }
            break;
        }
        case COR:
        {
            corSkinning( m_refData.m_referenceMesh.vertices(),
                         m_frameData.m_currentPose,
                         m_refData.m_weights,
                         m_refData.m_CoR,
                         m_frameData.m_currentPos );
            break;
        }
        case STBS_LBS:
        {
            linearBlendSkinningSTBS( m_refData.m_referenceMesh.vertices(),
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
            AlignedStdVector<DualQuaternion> DQ;
            computeDQSTBS( m_frameData.m_refToCurrentRelPose, *skel, m_refData.m_skeleton, m_refData.m_weights, m_weightSTBS, DQ );
            dualQuaternionSkinning( m_refData.m_referenceMesh.vertices(), DQ, m_frameData.m_currentPos );
            break;
        }
        }
    }
}

void uniformNormal( const Vector3Array& p,
                    const AlignedStdVector<Vector3ui>& T,
                    const std::vector<Index>& duplicateTable,
                    Vector3Array& normal ) {
    const uint N = p.size();
    normal.clear();
    normal.resize( N, Vector3::Zero() );

    for ( const auto& t : T )
    {
        const Index& i = duplicateTable.at( t( 0 ) );
        const Index& j = duplicateTable.at( t( 1 ) );
        const Index& k = duplicateTable.at( t( 2 ) );
        const Vector3 triN   = Geometry::triangleNormal( p[i], p[j], p[k] );
        if ( !triN.allFinite() ) { continue; }
        normal[i] += triN;
        normal[j] += triN;
        normal[k] += triN;
    }

#pragma omp parallel for
    for ( int i = 0; i < N; ++i )
    {
        if ( !normal[i].isApprox( Vector3::Zero() ) ) { normal[i].normalize(); }
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
        AttribArrayGeometry* geom;
        if ( !m_meshIsPoly )
        {
            geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
        }
        else {
            geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
        }

        geom->setVertices( m_frameData.m_currentPos );

        // FIXME: normals should be computed by the Skinning method!
        if ( ( m_skinningType != LBS && m_skinningType != DQS ) || g_standardLBS )
        {
            Vector3Array& normals = geom->normalsWithLock();
            uniformNormal( m_frameData.m_currentPos, m_refData.m_referenceMesh.getIndices(), m_duplicatesMap, normals );
            geom->normalsUnlock();
        }
        else // FIXME: they are computed through skinning for LBS and DQS only
        {
            geom->setNormals( m_frameData.m_currentNormal );
            if ( geom->hasAttrib( tangentName ) )
            {
                geom->getAttrib( geom->getAttribHandle<Vector3>( tangentName ) ).
                    setData( m_frameData.m_currentTangent );
            }
            if ( geom->hasAttrib( bitangentName ) )
            {
                geom->getAttrib( geom->getAttribHandle<Vector3>( bitangentName ) ).
                    setData( m_frameData.m_currentBitangent );
            }
        }

        g_standardLBS = !g_standardLBS;

        std::swap( m_frameData.m_previousPose, m_frameData.m_currentPose );
        std::swap( m_frameData.m_previousPos, m_frameData.m_currentPos );

        m_frameData.m_doReset    = false;
        m_frameData.m_doSkinning = false;
    }
}

void SkinningComponent::handleSkinDataLoading( const Asset::HandleData* data,
                                               const std::string& meshName,
                                               const Transform& meshFrame ) {
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
                m_loadedBindMatrices[bone.m_name] = Transform::Identity();
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

    checkWeightMatrix( m_refData.m_weights, false, true );

    if ( normalizeWeights( m_refData.m_weights, true ) )
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
        Vector3 A;
        Vector3 B;
        refSkel.getBonePoints( parent, A, B );
        B = parentT * B;
        Vector3 B_ = boneModel.translation();
        auto q = Quaternion::FromTwoVectors( ( B - A ), ( B_ - A ) );
        Transform R( q );
        R.pretranslate( A );
        R.translate( -A );
        refSkel.setTransform( parent, R * parentModel, SpaceType::MODEL );
    }
    m_frameData.m_currentPose = refSkel.getPose( SpaceType::MODEL );
}

void SkinningComponent::applyBindMatrices( Pose& pose ) const {
    for ( const auto& bM : m_refData.m_bindMatrices )
    {
        pose[bM.first] = m_meshFrameInv * pose[bM.first] * bM.second;
    }
}

void SkinningComponent::setupIO( const std::string& id ) {
    auto compMsg = ComponentMessenger::getInstance();
    auto wOut = std::bind( &SkinningComponent::getWeightsOutput, this );
    compMsg->registerOutput<WeightMatrix>( getEntity(), this, id, wOut );

    auto refData = std::bind( &SkinningComponent::getRefData, this );
    compMsg->registerOutput<RefData>( getEntity(), this, id, refData );

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

const WeightMatrix* SkinningComponent::getWeightsOutput() const {
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
        if ( m_refData.m_CoR.empty() ) { computeCoR( m_refData ); }
        break;
    }
    case STBS_DQS:
        [[fallthrough]];
    case STBS_LBS:
    {
        if ( m_weightSTBS.size() == 0 )
        {
            computeSTBS_weights( m_refData.m_referenceMesh.vertices(),
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
    AttribHandle<Vector3> handle;

    AttribArrayGeometry* geom;
    if ( !m_meshIsPoly )
    {
        geom = const_cast<TriangleMesh*>( m_triMeshWriter() );
    }
    else {
        geom = const_cast<PolyMesh*>( m_polyMeshWriter() );
    }

    if ( m_showingWeights )
    {
        // update the displayed weights
        const auto size = m_frameData.m_currentPos.size();
        m_weightsUV.resize( size, Vector3::Zero() );
        switch ( m_weightType ) {
        case STANDARD:
        {
    #pragma omp parallel for
            for ( int i = 0; i < int( size ); ++i )
            {
                m_weightsUV[i][0] = m_refData.m_weights.coeff( i, m_weightBone );
            }
        } break;
        case STBS:
        {
    #pragma omp parallel for
            for ( int i = 0; i < int( size ); ++i )
            {
                m_weightsUV[i][0] = m_weightSTBS.coeff( i, m_weightBone );
            }
        } break;
        }
        // change the material
        ro->setMaterial( m_weightMaterial );
        ro->getRenderTechnique()->setParametersProvider( m_weightMaterial );
        // get the UV attrib handle, will create it if not there.
        handle = geom->addAttrib<Vector3>( attrUV );
        geom->getAttrib( handle ).setData( m_weightsUV );
    }
    else
    {
        // change the material
        ro->setMaterial( m_baseMaterial );
        ro->getRenderTechnique()->setParametersProvider( m_baseMaterial );
        // if the UV attrib existed before, reset it, otherwise remove it.
        handle = geom->getAttribHandle<Vector3>( attrUV );
        if ( m_baseUV.size() > 0 ) { geom->getAttrib( handle ).setData( m_baseUV ); }
        else
        { geom->removeAttrib( handle ); }
    }
    m_forceUpdate = true;
}

bool SkinningComponent::isShowingWeights() {
    return m_showingWeights;
}

void SkinningComponent::showWeightsType( WeightType type ) {
    m_weightType = type;
    if ( m_showingWeights )
    {
        showWeights( true );
    }
}

SkinningComponent::WeightType SkinningComponent::getWeightsType() {
    return m_weightType;
}

void SkinningComponent::setWeightBone( uint bone ) {
    m_weightBone = bone;
    if ( m_showingWeights )
    {
        showWeights( true );
    }
}

} // namespace Scene
} // namespace Engine
} // namespace Ra
