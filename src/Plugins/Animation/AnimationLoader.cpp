#include "AnimationLoader.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <Core/Log/Log.hpp>
#include <vector>
#include <Core/Animation/Pose/Pose.hpp>
#include <Core/Utils/Graph/AdjacencyList.hpp>
#include <Core/Animation/Handle/HandleWeight.hpp>
#include <iostream>
#include <map>
#include <string.h>
#include <set>
#include <Core/Animation/Pose/PoseOperation.hpp>

namespace AnimationPlugin
{
	namespace AnimationLoader
	{	
        struct aiStringComparator
        {
            bool operator()(const aiString& left, const aiString& right)
            {
                return strcmp(left.C_Str(), right.C_Str()) < 0;
            }
        };
        typedef std::map<aiString, int, aiStringComparator> BoneMap;
    
		void recursiveSkeletonRead(const aiNode* node, aiMatrix4x4 accTransform, BoneMap& bones, AnimationData& data, int parent);
        void assimpToCore(const aiMatrix4x4& inMatrix, Ra::Core::Transform &outMatrix);
        void assimpToCore(const aiVector3D& inTranslation, const aiQuaternion& inRotation, const aiVector3D& inScaling, Ra::Core::Transform& outTransform);
        void assimpToCore(const aiQuaternion& inQuat, Ra::Core::Quaternion& outQuat);
        void assimpToCore(const aiVector3D& inVec, Ra::Core::Vector3& outVec);
        void getUniqueKeyTimes(aiAnimation* animation, std::vector<double> &times);
        void getTransformFromKey(const aiNodeAnim* key, int i, Ra::Core::Transform& keyTransform);
	
		AnimationData loadFile( const std::string& name, int index)
		{
			Assimp::Importer importer;
	        const aiScene* scene = importer.ReadFile(name, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals |
	                                                  aiProcess_SortByPType | aiProcess_FixInfacingNormals | aiProcess_CalcTangentSpace | aiProcess_GenUVCoords);
			
			AnimationData animData;
            animData.hasLoaded = false;
			
			if (scene == NULL)
			{
				LOG( logERROR ) << "Error while loading file \"" << name << "\" : " << importer.GetErrorString() << ".";
				return animData;
			}
            if (index < 0 || index >= scene->mNumMeshes)
            {
                LOG(logDEBUG) << "Invalid mesh index: " << index << " requested, but " << scene->mNumMeshes << " meshes have been found";
                return animData;
            }

            // skeleton loading
            aiMesh* mesh = scene->mMeshes[index];
            if (mesh->mNumBones == 0)
            {
                LOG(logDEBUG) << "Mesh #" << index << ": no skeleton found.";
                return animData;
            }
            
            BoneMap boneMap; // first: name of the boneNode, second: index of the bone in the hierarchy / pose
            animData.weights.resize(mesh->mNumVertices, mesh->mNumBones);
            
            for (int i = 0; i < mesh->mNumBones; i++)
            {
                boneMap[mesh->mBones[i]->mName] = -1; // the true index will get written during the recursive read of the scene
                for (int j = 0; j < mesh->mBones[i]->mNumWeights; j++)
                {
                    aiVertexWeight vertexWeight = mesh->mBones[i]->mWeights[j];
                    animData.weights.insert(vertexWeight.mVertexId, i) = vertexWeight.mWeight;
                }
            }
            
            // find the bone nodes and create the corresponding skeleton
            recursiveSkeletonRead(scene->mRootNode, aiMatrix4x4(), boneMap, animData, -1);
            
            // animation loading
            LOG(logDEBUG) << "Found " << scene->mNumAnimations << " animations ";
            
            if (scene->mNumAnimations > 0)
            {
                aiAnimation* animation = scene->mAnimations[0];
                int channelCount = animation->mNumChannels;
                int boneCount = boneMap.size();

                std::vector<double> timeSet;
                getUniqueKeyTimes(animation, timeSet);
                int keyCount = timeSet.size();
                
                std::vector<Ra::Core::Animation::Pose> poses;
                for (int i = 0; i < keyCount; i++)
                    poses.push_back(Ra::Core::Animation::Pose(boneCount));
                
                bool animatedBones[boneCount];
                for (int i = 0; i < boneCount; i++)
                    animatedBones[i] = false;
                
                for (int i = 0; i < channelCount; i++)
                {
                    aiNodeAnim* currentNodeAnim = animation->mChannels[i];
                    if (boneMap.find(currentNodeAnim->mNodeName) == boneMap.end()) // We should be able to ignore bones that do not affect the mesh
                        continue;
                    //CORE_ASSERT(boneMap.find(currentNodeAnim->mNodeName) != boneMap.end(), "Unknown bone channel");
                    
                    int channelKeyCount = currentNodeAnim->mNumPositionKeys;
                    int boneIndex = boneMap[currentNodeAnim->mNodeName];
                    animatedBones[boneIndex] = true;
                    
                    int channelKeyIndex = 0;
                    for (int j = 0; j < keyCount; j++)
                    {
                        double channelKeyTime = currentNodeAnim->mPositionKeys[channelKeyIndex].mTime;
                        if (channelKeyTime == timeSet[j])
                        {
                            Ra::Core::Transform keyTransform;
                            getTransformFromKey(currentNodeAnim, channelKeyIndex, keyTransform);
                            poses[j][boneIndex] = keyTransform;
                            if (channelKeyIndex < channelKeyCount - 1)
                                channelKeyIndex++;
                        }
                        else if (channelKeyIndex == 0 || channelKeyIndex == channelKeyCount - 1) // the first channel key is after the current key
                        {
                            Ra::Core::Transform keyTransform;
                            getTransformFromKey(currentNodeAnim, channelKeyIndex, keyTransform);
                            poses[j][boneIndex] = keyTransform;
                        }
                        else if (channelKeyTime > timeSet[j]) // the current key is between two channel keys
                        {
                            // interpolate between the previous and current channel key
                            Ra::Core::Transform previousKeyTransform;
                            Ra::Core::Transform nextKeyTransform;
                            Ra::Core::Transform keyTransform;
                            getTransformFromKey(currentNodeAnim, channelKeyIndex - 1, previousKeyTransform);
                            getTransformFromKey(currentNodeAnim, channelKeyIndex, nextKeyTransform);
                            
                            double prevChannelKeyTime = currentNodeAnim->mPositionKeys[channelKeyIndex - 1].mTime;
                            Scalar t = (timeSet[j] - prevChannelKeyTime) / (channelKeyTime - prevChannelKeyTime);
                            Ra::Core::Animation::interpolateTransforms(previousKeyTransform, nextKeyTransform, t, keyTransform);
                            
                            poses[j][boneIndex] = keyTransform;
                        }
                        else
                        {
                            CORE_ASSERT(false, "AnimationLoader.cpp: should not be there");
                        }
                        
                        if (animData.hierarchy.isRoot(boneIndex))
                            poses[j][boneIndex] = animData.baseTransform * poses[j][boneIndex];
                    }
                }
                
                for (int i = 0; i < boneCount; i++)
                {
                    if (!animatedBones[i])
                    {
                        for (int j = 0; j < keyCount; j++)
                        {
                            poses[j][i] = animData.pose[i];
                        }
                    }
                }
                
                Scalar animationRate = animation->mTicksPerSecond > 0.0 ? animation->mTicksPerSecond : 50.0;
                for (int i = 0; i < keyCount; i++)
                {
                    Scalar keyTime = timeSet[i] / animationRate;
                    animData.animation.addKeyPose(poses[i], keyTime);
                }
                animData.animation.normalize();
            }
			
            animData.hasLoaded = true;
			return animData;
		}
		
		void recursiveSkeletonRead(const aiNode* node, aiMatrix4x4 accTransform, BoneMap &boneMap, AnimationData& data, int parent)
		{           
			aiMatrix4x4 currentTransform  = accTransform * node->mTransformation;
			BoneMap::const_iterator boneIt = boneMap.find(node->mName);
            bool isBoneNode = boneIt != boneMap.end();
			
			int currentIndex = parent;
			if (isBoneNode)
			{
                if (parent == -1)
                {
                    assimpToCore(accTransform, data.baseTransform);
                }
                
				// store the bone in the hierarchy
				currentIndex = data.hierarchy.addNode(parent);
                // store the index in the BoneMap
                boneMap[node->mName] = currentIndex;
			
				// store the transform for the bone
				Ra::Core::Transform tr;
				assimpToCore(currentTransform, tr);
				data.pose.push_back(tr);
				
				// initialize the transform for the child bones
				currentTransform = aiMatrix4x4();
			}
			
			for (int i = 0; i < node->mNumChildren; i++)
				recursiveSkeletonRead(node->mChildren[i], currentTransform, boneMap, data, currentIndex);
		}
        
        void getTransformFromKey(const aiNodeAnim* key, int i, Ra::Core::Transform& keyTransform)
        {
            aiVector3D keyPosition = key->mPositionKeys[i].mValue;
            aiQuaternion keyRotation = key->mRotationKeys[i].mValue;
            aiVector3D keyScaling = key->mScalingKeys[i].mValue;

            // convert the key to a transform matrix
            assimpToCore(keyPosition, keyRotation, keyScaling, keyTransform);
        }
		
        void getUniqueKeyTimes(aiAnimation* animation, std::vector<double>& times)
        {
            int channelCount = animation->mNumChannels;
            std::set<double> timeSet;
            for (int i = 0; i < channelCount; i++)
            {
                aiNodeAnim* currentNodeAnim = animation->mChannels[i];
                
                int channelKeyCount = currentNodeAnim->mNumRotationKeys;
                for (int j = 0; j < channelKeyCount; j++)
                {
                    const aiVectorKey& positionKey = currentNodeAnim->mPositionKeys[j];
                    const aiQuatKey& rotationKey = currentNodeAnim->mRotationKeys[j];
                    const aiVectorKey& scalingKey = currentNodeAnim->mScalingKeys[j];
                    
                    CORE_ASSERT(positionKey.mTime == rotationKey.mTime && positionKey.mTime == scalingKey.mTime, "Invalid key times");
                    
                    timeSet.insert(positionKey.mTime);
                }
            }
            std::copy(timeSet.begin(), timeSet.end(), std::back_inserter(times));
        }
        
		void assimpToCore( const aiMatrix4x4& inMatrix, Ra::Core::Transform& outMatrix )
        {
            for ( uint i = 0; i < 4; ++i )
            {
                for ( uint j = 0; j < 4; ++j )
                {
                    outMatrix(i, j) = inMatrix[i][j];
                }
            }
        }
        
        void assimpToCore(const aiQuaternion& inQuat, Ra::Core::Quaternion& outQuat)
        {
            outQuat = Ra::Core::Quaternion(inQuat.w, inQuat.x, inQuat.y, inQuat.z);
        }
        
        void assimpToCore(const aiVector3D& inVec, Ra::Core::Vector3& outVec)
        {
            outVec = Ra::Core::Vector3(inVec.x, inVec.y, inVec.z);
        }
        
        void assimpToCore(const aiVector3D &inTranslation, const aiQuaternion &inRotation, const aiVector3D &inScaling, Ra::Core::Transform& outTransform)
        {
            Ra::Core::Vector3 translation;
            Ra::Core::Vector3 scaling;
            Ra::Core::Quaternion rotation;
            assimpToCore(inTranslation, translation);
            assimpToCore(inScaling, scaling);
            assimpToCore(inRotation, rotation);
            outTransform.fromPositionOrientationScale(translation, rotation, scaling);
        }
	}
}
