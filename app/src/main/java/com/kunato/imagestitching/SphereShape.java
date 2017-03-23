/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.kunato.imagestitching;
import android.util.Log;

import java.lang.Math;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
/*
 * Class for generating a sphere model for given input params
 * The generated class will have vertices and indices
 * Vertices data is composed of vertex coordinates in x, y, z followed by
 *  texture coordinates s, t for each vertex
 * Indices store vertex indices for the whole sphere.
 * Formula for generating sphere is originally coming from source code of
 * OpenGL ES2.0 Programming guide
 * which is available from http://code.google.com/p/opengles-book-samples/,
 * but some changes were made to make texture look right.
 */
public class SphereShape {
    public static final int FLOAT_SIZE = 4;
    public static final int SHORT_SIZE = 2;
    private FloatBuffer mVertices;
    private ShortBuffer[] mIndices;
    private int[] mNumIndices;
    private int mTotalIndices;
    /*
     * @param nSlices how many slice in horizontal direction.
     *                The same slice for vertical direction is applied.
     *                nSlices should be > 1 and should be <= 180
     * @param x,y,z the origin of the sphere
     * @param r the radius of the sphere
     */
    public SphereShape(int nSlices, float r, int numIndexBuffers) {

        int nVertices = (nSlices+1) * (nSlices+1);
        if (nVertices > Short.MAX_VALUE) {
            // this cannot be handled in one vertices / indices pair
            throw new RuntimeException("nSlices " + nSlices + " too big for vertex");
        }
        mTotalIndices = nSlices * nSlices * 6;
        // 3 vertex coords + 2 texture coords
        mVertices = ByteBuffer.allocateDirect(nVertices * 5 * FLOAT_SIZE)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        mIndices = new ShortBuffer[numIndexBuffers];
        mNumIndices = new int[numIndexBuffers];
        // first evenly distribute to n-1 buffers, then put remaining ones to the last one.
        int noIndicesPerBuffer = (mTotalIndices / numIndexBuffers / 6) * 6;
        for (int i = 0; i < numIndexBuffers - 1; i++) {
            mNumIndices[i] = noIndicesPerBuffer;
        }
        mNumIndices[numIndexBuffers - 1] = mTotalIndices - noIndicesPerBuffer *
                (numIndexBuffers - 1);
        for (int i = 0; i < numIndexBuffers; i++) {
            mIndices[i] = ByteBuffer.allocateDirect(mNumIndices[i] * SHORT_SIZE)
                    .order(ByteOrder.nativeOrder()).asShortBuffer();
        }
        // calling put for each float took too much CPU time, so put by line instead
        float[] vLineBuffer = new float[(nSlices+1) * 5];

        for (int i = 0; i <= nSlices; i++) {
            for (int j = 0; j <= nSlices; j++) {

                int vertexBase = j * 5;
                //mathPi / 1/(nslide * i)
                //1 * ratio // 1 / ratio
                float sini = (float) Math.sin((Math.PI * i/(float)nSlices));
                //mathpi + mathpi+ (2*mathpi)/1/(ratio j)
                float cosi = (float) Math.cos((Math.PI * i/(float)nSlices));
                float sinj = (float) Math.sin(Math.PI + (2 * Math.PI) * (j / (float) nSlices));
                float cosj = (float) Math.cos(Math.PI + (2 * Math.PI) * (j/(float)nSlices));
                // vertex x,y,z
                //x
                vLineBuffer[vertexBase + 0] = r * sini * sinj;
                //y
                vLineBuffer[vertexBase + 1] = r * cosi;
                //z(swap opencv texture and opengl)
                vLineBuffer[vertexBase + 2] = -r * sini * cosj;

//                Log.i("Vertex","("+(x + r * sini * sinj)+","+(y + r * sini * cosj)+","+(z + r * cosi)+")");

                //change u (s)
                // texture u,v
                vLineBuffer[vertexBase + 3] = (float) j/ (float) nSlices;
                vLineBuffer[vertexBase + 4] = (float) i /(float) nSlices;
//                Log.i("Texture","("+((float) j / (float) nSlices)+","+-((1.0f - i) / (float)nSlices)+")");
            }
            mVertices.put(vLineBuffer, 0, vLineBuffer.length);
//            for(int ik = 0 ; ik < vLineBuffer.length; ik+=5){
//                Log.i("SphereCoord,",""+vLineBuffer[ik]+","+vLineBuffer[ik+1]+","+vLineBuffer[ik+2]+","+vLineBuffer[ik+3]+","+vLineBuffer[ik+4]);
//            }
        }

        short[] indexBuffer = new short[max(mNumIndices)];
        int index = 0;
        int iMax = nSlices+1;
        int bufferNum = 0;
        for (int i = 0; i < nSlices; i++) {
            for (int j = 0; j < nSlices; j++) {
                int i1 = i + 1;
                int j1 = j + 1;
                if (index >= mNumIndices[bufferNum]) {
                    // buffer ready for moving to target
                    mIndices[bufferNum].put(indexBuffer, 0, mNumIndices[bufferNum]);
                    // move to the next one
                    index = 0;
                    bufferNum++;
                }
                indexBuffer[index++] = (short) (i * iMax + j);
                indexBuffer[index++] = (short) (i1 * iMax + j);
                indexBuffer[index++] = (short) (i1 * iMax + j1);
                indexBuffer[index++] = (short) (i * iMax + j);
                indexBuffer[index++] = (short) (i1 * iMax + j1);
                indexBuffer[index++] = (short) (i * iMax + j1);
            }
        }
        mIndices[bufferNum].put(indexBuffer, 0, mNumIndices[bufferNum]);
        mVertices.position(0);
        for (int i = 0; i < numIndexBuffers; i++) {
            mIndices[i].position(0);
        }
    }
    public FloatBuffer getVertices() {
        return mVertices;
    }
    public int getVeticesStride() {
        return 5*FLOAT_SIZE;
    }
    public ShortBuffer[] getIndices() {
        return mIndices;
    }
    public int[] getNumIndices() {
        return mNumIndices;
    }
    public int getTotalIndices() {
        return mTotalIndices;
    }
    private int max(int[] array) {
        int max = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) max = array[i];
        }
        return max;
    }
}