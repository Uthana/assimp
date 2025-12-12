/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2025, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the following
conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
----------------------------------------------------------------------
*/

#if !defined(ASSIMP_BUILD_NO_EXPORT) && !defined(ASSIMP_BUILD_NO_BVH_EXPORTER)

#include "BVHExporter.h"
#include <assimp/Exceptional.h>
#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <cmath>
#include <memory>

using namespace Assimp;

namespace Assimp {

// ------------------------------------------------------------------------------------------------
// Worker function for exporting a scene to BVH. Prototyped and registered in Exporter.cpp
void ExportSceneBVH(const char *pFile, IOSystem *pIOSystem, const aiScene *pScene, const ExportProperties * /*pProperties*/) {
    // invoke the exporter
    BVHExporter exporter(pFile, pScene);

    if (exporter.mOutput.fail()) {
        throw DeadlyExportError("output data creation failed. Most likely the file became too large: " + std::string(pFile));
    }

    // we're still here - export successfully completed. Write the file.
    std::unique_ptr<IOStream> outfile(pIOSystem->Open(pFile, "wt"));
    if (outfile == nullptr) {
        throw DeadlyExportError("could not open output .bvh file: " + std::string(pFile));
    }

    outfile->Write(exporter.mOutput.str().c_str(), static_cast<size_t>(exporter.mOutput.tellp()), 1);
}

} // namespace Assimp

namespace {

// Helper function to convert quaternion to Euler angles (ZXY order, which is common in BVH)
void QuaternionToEulerZXY(const aiQuaternion &q, float &rotX, float &rotY, float &rotZ) {
    // Convert quaternion to rotation matrix
    aiMatrix3x3 mat = q.GetMatrix();

    // Extract Euler angles in ZXY order
    // This matches the common BVH convention: Zrotation Xrotation Yrotation
    float sinX = mat.b3;
    if (sinX >= 1.0f) {
        rotX = static_cast<float>(AI_MATH_PI / 2.0);
        rotY = 0.0f;
        rotZ = std::atan2(mat.a2, mat.a1);
    } else if (sinX <= -1.0f) {
        rotX = static_cast<float>(-AI_MATH_PI / 2.0);
        rotY = 0.0f;
        rotZ = std::atan2(-mat.a2, mat.a1);
    } else {
        rotX = std::asin(sinX);
        rotY = std::atan2(-mat.b1, mat.b2);
        rotZ = std::atan2(-mat.a3, mat.c3);
    }

    // Convert from radians to degrees
    rotX *= static_cast<float>(180.0 / AI_MATH_PI);
    rotY *= static_cast<float>(180.0 / AI_MATH_PI);
    rotZ *= static_cast<float>(180.0 / AI_MATH_PI);
}

} // anonymous namespace

// ------------------------------------------------------------------------------------------------
BVHExporter::BVHExporter(const char *filename, const aiScene *pScene)
    : mFilename(filename), mScene(pScene), mAnim(nullptr) {
    // make sure that all formatting happens using the standard, C locale
    const std::locale &l = std::locale("C");
    mOutput.imbue(l);
    mOutput.precision(6);

    // Get the first animation if available
    if (pScene->mNumAnimations > 0) {
        mAnim = pScene->mAnimations[0];
    }

    // Write the BVH file
    WriteHierarchy();
    WriteMotion();
}

// ------------------------------------------------------------------------------------------------
void BVHExporter::WriteHierarchy() {
    mOutput << "HIERARCHY\n";

    if (mScene->mRootNode) {
        WriteNode(mScene->mRootNode, 0);
    }
}

// ------------------------------------------------------------------------------------------------
void BVHExporter::WriteNode(const aiNode *node, int depth) {
    std::string indent(depth * 2, ' ');
    bool isRoot = (depth == 0);
    bool isEndSite = (node->mNumChildren == 0);

    // Get the offset from the node's transformation matrix
    aiVector3D offset(node->mTransformation.a4,
                      node->mTransformation.b4,
                      node->mTransformation.c4);

    if (isEndSite) {
        // End Site nodes have no channels
        mOutput << indent << "End Site\n";
        mOutput << indent << "{\n";
        mOutput << indent << "  OFFSET " << offset.x << " " << offset.y << " " << offset.z << "\n";
        mOutput << indent << "}\n";
    } else {
        // ROOT or JOINT
        if (isRoot) {
            mOutput << "ROOT " << node->mName.C_Str() << "\n";
        } else {
            mOutput << indent << "JOINT " << node->mName.C_Str() << "\n";
        }

        mOutput << indent << "{\n";
        mOutput << indent << "  OFFSET " << offset.x << " " << offset.y << " " << offset.z << "\n";

        // Root nodes get 6 channels (position + rotation), joints get 3 (rotation only)
        if (isRoot) {
            mOutput << indent << "  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n";
        } else {
            mOutput << indent << "  CHANNELS 3 Zrotation Xrotation Yrotation\n";
        }

        // Track node order for motion data output
        mNodeOrder.push_back(node);

        // Write children
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            WriteNode(node->mChildren[i], depth + 1);
        }

        mOutput << indent << "}\n";
    }
}

// ------------------------------------------------------------------------------------------------
void BVHExporter::WriteMotion() {
    mOutput << "MOTION\n";

    unsigned int numFrames = 1;
    double frameTime = 1.0 / 30.0; // Default 30 fps

    if (mAnim) {
        // Use animation duration to determine frame count
        numFrames = static_cast<unsigned int>(mAnim->mDuration) + 1;
        if (mAnim->mTicksPerSecond > 0) {
            frameTime = 1.0 / mAnim->mTicksPerSecond;
        }
    }

    mOutput << "Frames: " << numFrames << "\n";
    mOutput << "Frame Time: " << frameTime << "\n";

    // Write frame data
    for (unsigned int frame = 0; frame < numFrames; ++frame) {
        WriteFrame(frame);
    }
}

// ------------------------------------------------------------------------------------------------
void BVHExporter::WriteFrame(unsigned int frameIndex) {
    bool first = true;

    for (size_t i = 0; i < mNodeOrder.size(); ++i) {
        const aiNode *node = mNodeOrder[i];
        bool isRoot = (i == 0);

        // Default values
        aiVector3D pos(0.0f, 0.0f, 0.0f);
        aiQuaternion rot;
        float rotX = 0.0f, rotY = 0.0f, rotZ = 0.0f;

        // Get the offset from the node's transformation as default position
        if (isRoot) {
            pos.x = node->mTransformation.a4;
            pos.y = node->mTransformation.b4;
            pos.z = node->mTransformation.c4;
        }

        // Try to get animation data for this node
        const aiNodeAnim *nodeAnim = GetNodeAnim(std::string(node->mName.C_Str()));
        if (nodeAnim) {
            // Get position for this frame (only for root)
            if (isRoot && nodeAnim->mNumPositionKeys > 0) {
                unsigned int posKeyIndex = frameIndex;
                if (posKeyIndex >= nodeAnim->mNumPositionKeys) {
                    posKeyIndex = nodeAnim->mNumPositionKeys - 1;
                }
                pos = nodeAnim->mPositionKeys[posKeyIndex].mValue;
            }

            // Get rotation for this frame
            if (nodeAnim->mNumRotationKeys > 0) {
                unsigned int rotKeyIndex = frameIndex;
                if (rotKeyIndex >= nodeAnim->mNumRotationKeys) {
                    rotKeyIndex = nodeAnim->mNumRotationKeys - 1;
                }
                rot = nodeAnim->mRotationKeys[rotKeyIndex].mValue;
                QuaternionToEulerZXY(rot, rotX, rotY, rotZ);
            }
        }

        // Write position (only for root)
        if (isRoot) {
            if (!first) mOutput << " ";
            mOutput << pos.x << " " << pos.y << " " << pos.z;
            first = false;
        }

        // Write rotation (ZXY order)
        if (!first) mOutput << " ";
        mOutput << rotZ << " " << rotX << " " << rotY;
        first = false;
    }

    mOutput << "\n";
}

// ------------------------------------------------------------------------------------------------
const aiNodeAnim *BVHExporter::GetNodeAnim(const std::string &nodeName) const {
    if (!mAnim) {
        return nullptr;
    }

    for (unsigned int i = 0; i < mAnim->mNumChannels; ++i) {
        if (nodeName == mAnim->mChannels[i]->mNodeName.C_Str()) {
            return mAnim->mChannels[i];
        }
    }

    return nullptr;
}

#endif // !ASSIMP_BUILD_NO_EXPORT && !ASSIMP_BUILD_NO_BVH_EXPORTER
