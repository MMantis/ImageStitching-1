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

import android.content.Context;
import android.graphics.Bitmap;
import android.opengl.GLES31;
import android.opengl.GLUtils;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Arrays;


public class StitchObject {

    private final String vertexShaderCode =
            "#version 310 es\n" +
                    "in vec2 vPosition;\n" +
                    "in vec2 vTexCoord;\n" +
                    "out vec2 texCoord;\n" +
                    "void main() {\n" +
                    "  texCoord = vTexCoord;\n" +
                    "  gl_Position = vec4 ( vPosition.x, vPosition.y, 0.0, 1.0 );\n" +
                    "}";

    private final String fragmentShaderCode = "" +
            "#version 310 es\n" +
            "#define M_PI 3.1415926535897932384626433832795\n" +
            "#define SCALE 1468.803406\n"+
            "precision highp float;\n" +
            "precision highp int;\n" +
            "uniform int length;\n" +
            "uniform sampler2D sTexture[11];\n" +
            "uniform ivec2 size[11];\n" +
            "uniform ivec2 corner[11];\n" +
            "uniform mat3 r_kinv[11];\n" +
            "uniform mat3 k_rinv[11];\n" +
            "uniform ivec2 tl;\n" +
            "uniform ivec2 winSize;\n" +
            "in vec2 texCoord;\n" +
            "out vec4 fragmentColor;\n" +
            "int idx = 0;\n" +
            "vec2 mapForward(in mat3 matrix, in vec3 uv, in float scale){\n" +
            "   vec3 temp = matrix * uv;\n" +
            "   float u = scale * atan(temp.y,temp.x);\n" +
            "   float v = scale * (M_PI - acos(temp.x / sqrt(temp.y * temp.y + temp.x * temp.x + temp.z * temp.z)));\n" +
            "   return vec2(u,v);\n" +
            "}\n" +
            "vec3 mapBackward(in vec3 uv, in float scale, in mat3 matrix){\n" +
            "   uv/=scale;\n" +
            "   float sinv = sin(M_PI - uv.y);\n" +
            "   vec3 temp = vec3(sinv * sin(uv.x),cos(M_PI - uv.y),sinv * cos(uv.x) );\n" +
            "   vec3 ret = matrix * temp;\n" +
            "   if(ret.z < 0.0){\n" +
            "       ret.x = -1.0;ret.y = -1.0;" +
            "   }else{\n" +
            "       ret.x/=ret.z;ret.y/=ret.z;\n" +
            "   }\n" +
            "   return ret;\n" +
            "}\n" +
            "vec4 getSampleFromArray(in int ndx){\n" +
            "   vec4 color;\n" +
            "   if(ndx == 0){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[0]);\n" +
            "       color = texelFetch(sTexture[0], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[0].x && int(map.y) < 0 && int(map.y) >= size[0].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 1){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[1]);" +
            "       color = texelFetch(sTexture[1], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[1].x && int(map.y) < 0 && int(map.y) >= size[1].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 2){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[2]);" +
            "       color = texelFetch(sTexture[2], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[2].x && int(map.y) < 0 && int(map.y) >= size[2].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 3){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[3]);" +
            "       color = texelFetch(sTexture[3], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[3].x && int(map.y) < 0 && int(map.y) >= size[3].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 4){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[4]);" +
            "       color = texelFetch(sTexture[4], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[4].x && int(map.y) < 0 && int(map.y) >= size[4].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 5){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[5]);" +
            "       color = texelFetch(sTexture[5], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[5].x && int(map.y) < 0 && int(map.y) >= size[5].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 6){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[6]);" +
            "       color = texelFetch(sTexture[6], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[6].x && int(map.y) < 0 && int(map.y) >= size[6].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 7){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[7]);" +
            "       color = texelFetch(sTexture[7], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[7].x && int(map.y) < 0 && int(map.y) >= size[7].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 8){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[8]);" +
            "       color = texelFetch(sTexture[8], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[8].x && int(map.y) < 0 && int(map.y) >= size[8].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 9){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[9]);" +
            "       color = texelFetch(sTexture[9], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[9].x && int(map.y) < 0 && int(map.y) >= size[9].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   else if(ndx == 10){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x, winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[10]);" +
            "       color = texelFetch(sTexture[10], ivec2(round(map.x),round(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[10].x && int(map.y) < 0 && int(map.y) >= size[10].y ){" +
            "           color.a = 0.0;" +
            "       }" +
            "       else{" +
            "           int x = int(map.x);" +
            "           if(x > size[ndx].x - int(map.x)){" +
            "               x = size[ndx].x - int(map.x);" +
            "           }" +
            "           int y = int(map.y);" +
            "           if(y > size[ndx].y - int(map.y)){" +
            "               y = size[ndx].y - int(map.y);" +
            "           }" +
            "           if(y > x){" +
            "               color *= float(x)/float(size[ndx].x);" +
            "           }" +
            "           else{" +
            "               color *= float(y)/float(size[ndx].y);" +
            "           }" +
            "       }" +
            "   }\n" +
            "   return color;\n"+
            "}\n" +
            "void main() {\n" +
            "   int idx;" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));" +
            "   vec4 color = vec4(0,0,0,0);" +
            "   for(int i = length-1 ; i >= 0 ;i--){" +
            "       if(corner[i].x < pos.x + tl.x && corner[i].x + size[i].x > pos.x + tl.x && corner[i].y < pos.y + tl.y && corner[i].y + size[i].y > pos.y + tl.y){" +
            "           idx = i;\n" +
            "           color += getSampleFromArray(idx);\n" +
            "       }" +
            "   }" +
            "   if(color.a > 0.0){" +
            "       color/=color.w;" +
            "       fragmentColor = color;" +
            "   }" +
            "   \n" +
            "   \n" +
            "}";
    private final FloatBuffer pVertex;
    private final FloatBuffer pTexCoord;
    private int mTLX;
    private int mTLY;
    private final int mProgram;
    public final int NUMBER_OF_TEXTURE = 11;
    //Only one texture
    private int[] mTextures = new int[NUMBER_OF_TEXTURE];
    private int[] corners = new int[NUMBER_OF_TEXTURE*2];
    private int[] sizes = new int[NUMBER_OF_TEXTURE*2];
    private int[] mFBO = new int[1];
    private int mCurrentSize = 1;
    private int mFBOID;
    private int mFBOTex;
    private static final float SCALE = 1468.803406f;
    private Bitmap[] mCPUBitmap = new Bitmap[NUMBER_OF_TEXTURE];
    int[] cof = new int[NUMBER_OF_TEXTURE*2];
    int[] sof = new int[NUMBER_OF_TEXTURE*2];
    float[] k_rinvData = new float[]{ 1.46880334e+03f, 0.f, 5.56620422e+02f, 0.f, 1.46880334e+03f,  9.87914978e+02f, 0.f, 0.f, 1.f,
            1.56739111e+03f, -3.09704323e+01f, 9.76510925e+01f, 3.19323425e+02f, 1.44898230e+03f, 9.65319824e+02f, 2.95225173e-01f, -1.96384918e-02f, 9.55225825e-01f ,
            1.52318079e+03f, 3.44877005e+00f, -3.83559906e+02f, 5.54351318e+02f, 1.48857654e+03f, 7.81149048e+02f, 5.71894169e-01f, 2.03441400e-02f, 8.20075095e-01f ,
    };
    private ByteBuffer mScreenBuffer;
    private GLRenderer glRenderer;
    private int mWidth = 0;
    private int mHeight = 0;
    private boolean mUpdate = false;
    private Context context;

    public StitchObject(GLRenderer renderer) {
        Context context = renderer.mView.getActivity();
        glRenderer = renderer;
        mProgram = Util.loadShader(vertexShaderCode, fragmentShaderCode);

        float[] vtmp = { 1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f };
        float[] ttmp = { 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f };
        pVertex = ByteBuffer.allocateDirect(8 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        pVertex.put ( vtmp );
        pVertex.position(0);
        pTexCoord = ByteBuffer.allocateDirect(8 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        pTexCoord.put ( ttmp );
        pTexCoord.position(0);
        this.context = context;

        GLES31.glGenTextures(NUMBER_OF_TEXTURE, this.mTextures, 0);


    }

    public int createFBO(){
        //generate fbo id
        GLES31.glGenFramebuffers(1, mFBO, 0);
        mFBOID = mFBO[0];
        //generate texture
        GLES31.glGenTextures(1, mFBO, 0);
        mFBOTex = mFBO[0];
        //generate render buffer
        GLES31.glGenRenderbuffers(1, mFBO, 0);
        int renderBufferId = mFBO[0];
        //Bind Frame buffer
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, mFBOID);
        //Bind texture
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, mFBOTex);
        //Define texture parameters
        GLES31.glTexImage2D(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_RGBA, mWidth, mHeight, 0, GLES31.GL_RGBA, GLES31.GL_UNSIGNED_BYTE, null);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        //Bind render buffer and define buffer dimension
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, renderBufferId);
        GLES31.glRenderbufferStorage(GLES31.GL_RENDERBUFFER, GLES31.GL_DEPTH_COMPONENT16, mWidth, mHeight);
        //Attach texture FBO color attachment
        GLES31.glFramebufferTexture2D(GLES31.GL_FRAMEBUFFER, GLES31.GL_COLOR_ATTACHMENT0, GLES31.GL_TEXTURE_2D, mFBOTex, 0);
        //Attach render buffer to depth attachment
        GLES31.glFramebufferRenderbuffer(GLES31.GL_FRAMEBUFFER, GLES31.GL_DEPTH_ATTACHMENT, GLES31.GL_RENDERBUFFER, renderBufferId);

        int[] wh = new int[2];
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_WIDTH, wh, 0);
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_HEIGHT, wh, 1);
        //we are done, reset
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, 0);
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, 0);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);

        return mFBO[0];
    }

    public void setROI(int[] roi,float[] k_rinv){
        Log.d("Stitch GPU","Set ROI");
        for(int i = 0 ; i < roi.length ;i++){

            if(i%4 == 0){
                cof[i/2] = roi[i];
            }
            if(i%4 == 1){
                cof[(i/2)+1] = roi[i];
            }
            if(i%4 == 2){
                sof[(i-2)/2] = roi[i];
            }
            if(i%4 == 3){
                sof[((i-2)/2)+1] = roi[i];
            }
        }
        k_rinvData = k_rinv;
//        loadBitmap(mCurrentSize,null);
        mCurrentSize+=1;
        mUpdate = true;
    }

    public void loadBitmap(int i){
        GLES31.glActiveTexture(GLES31.GL_TEXTURE0+i);
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, this.mTextures[i]);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S,GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T,GLES31.GL_CLAMP_TO_EDGE);
        GLUtils.texImage2D(GLES31.GL_TEXTURE_2D, 0, mCPUBitmap[i], 0);
        GLES31.glGenerateMipmap(GLES31.GL_TEXTURE_2D);
        mCPUBitmap[i].recycle();
        updateParam();
        createFBO();
    }

    public void bitmapToCPU(Bitmap bitmap,int index){
        mCPUBitmap[index] = bitmap;

    }

    public void updateParam(){

        float min_x = 10000;
        float min_y = 10000;
        for(int i = 0 ; i < mCurrentSize*2 ;i++){
            if(cof[i] < min_x && i%2 == 0){
                min_x = cof[i];
            }
            if(cof[i] < min_y && i%2 == 1) {
                min_y = cof[i];

            }
        }

        mTLX = (int) min_x;
        mTLY = (int) min_y;

//
//        for(int i = 0; i< cof.length ;i++){
//            if(i%2 == 0){
//                cof[i] -= min_x;
//            }
//            if(i%2 == 1){
//                cof[i] -= min_y;
//            }
//        }
        min_x = 10000;
        min_y = 10000;
        float max_x = -10000;
        float max_y = -10000;
        int max_x_index = -1;
        int max_y_index = -1;

        for(int i = 0 ; i < mCurrentSize*2 ;i++){
            if(cof[i] < min_x && i%2 == 0){
                min_x = cof[i];
            }
            if(cof[i] < min_y && i%2 == 1){
                min_y = cof[i];

            }
            if(cof[i] > max_x && i%2 == 0){
                max_x = cof[i];
                max_x_index = i;
            }
            if(cof[i] > max_y && i%2 == 1){
                max_y = cof[i];
                max_y_index = i;
            }
        }
        corners = cof;
        mWidth = (int) (max_x-min_x+sof[max_x_index]);

        mHeight = (int) (max_y-min_y+sof[max_y_index]);
//        mWidth = 1495;
//        mHeight = 1719;
        Log.d("Stitch GPU","Size ("+mWidth+","+mHeight+") , TL ("+mTLX+","+mTLY+")");
        sizes = sof;
    }
    public int getFBOTexture(){
        return mFBOTex;
    }

    public void drawTOFBO() {
        if(!mUpdate)
            return;
        long start = System.currentTimeMillis();
        if(mCurrentSize == 2){
            loadBitmap(0);
            loadBitmap(1);
        }
        else{
            loadBitmap(mCurrentSize-1);
        }
        int count = 0;

        long start2 = System.currentTimeMillis();
        GLES31.glUseProgram(mProgram);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, mFBOID);
        GLES31.glViewport(0, 0, mWidth, mHeight);
        GLES31.glClear(GLES31.GL_COLOR_BUFFER_BIT);
        GLES31.glClear(GLES31.GL_DEPTH_BUFFER_BIT);
        int ph = GLES31.glGetAttribLocation(mProgram,"vPosition");
        int tch = GLES31.glGetAttribLocation(mProgram,"vTexCoord");
        int th = GLES31.glGetUniformLocation ( mProgram, "sTexture" );
        int sh = GLES31.glGetUniformLocation(mProgram,"size");
        int ch = GLES31.glGetUniformLocation(mProgram,"corner");
        int lh = GLES31.glGetUniformLocation(mProgram,"length");
        int r_kinvh = GLES31.glGetUniformLocation(mProgram,"r_kinv");
        int k_rinvh = GLES31.glGetUniformLocation(mProgram,"k_rinv");
        int tlh = GLES31.glGetUniformLocation(mProgram,"tl");
        int wsh = GLES31.glGetUniformLocation(mProgram,"winSize");
        GLES31.glVertexAttribPointer(ph, 2, GLES31.GL_FLOAT, false, 4*2, pVertex);
        GLES31.glVertexAttribPointer(tch, 2, GLES31.GL_FLOAT, false, 4*2, pTexCoord );
        GLES31.glEnableVertexAttribArray(ph);
        GLES31.glEnableVertexAttribArray(tch);
        //START OF RENDERING
        int[] texArray = new int[mCurrentSize];
        Log.d("GPU Stitch","mCurrentSize : "+mCurrentSize);
        for(int i = 0 ; i <mCurrentSize;i++){
            GLES31.glActiveTexture(GLES31.GL_TEXTURE0+i);
            GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, mTextures[i]);
            texArray[i] = i;
        }
        GLES31.glUniform1i(lh,mCurrentSize);

        Log.d("Stitch GPU","COF : "+Arrays.toString(corners));
        Log.d("Stitch GPU","SizeList : "+Arrays.toString(sizes));
        Log.d("Stitch GPU","TexArray : "+Arrays.toString(texArray));
        GLES31.glUniform2iv(sh,mCurrentSize,sizes,0);
        GLES31.glUniform2iv(ch,mCurrentSize,corners,0);
        GLES31.glUniform1iv(th,mCurrentSize,texArray,0);
        GLES31.glUniform2i(tlh,mTLX,mTLY);
        GLES31.glUniform2i(wsh,mWidth,mHeight);
        GLES31.glUniformMatrix3fv(k_rinvh,mCurrentSize,true,k_rinvData,0);
        GLES31.glDrawArrays(GLES31.GL_TRIANGLE_STRIP, 0, 4);
        long end = System.currentTimeMillis();
        Log.d("Stitch GPU","TimeSpend : "+(end-start2)*0.001+ " : "+(start2-start)*0.001 + " ; "+count);
        //END OF RENDERING

        mScreenBuffer = ByteBuffer.allocateDirect(mHeight * mWidth * 4);
        mScreenBuffer.order(ByteOrder.nativeOrder());
        GLES31.glReadPixels(0, 0, mWidth, mHeight, GLES31.GL_RGBA, GLES31.GL_UNSIGNED_BYTE, mScreenBuffer);

        mScreenBuffer.rewind();
        byte pixelsBuffer[] = new byte[4*mHeight*mWidth];
        mScreenBuffer.get(pixelsBuffer);
        Mat mat = new Mat(mHeight, mWidth, CvType.CV_8UC4);
        mat.put(0, 0, pixelsBuffer);
        Mat m = new Mat();
        Imgproc.cvtColor(mat, m, Imgproc.COLOR_RGBA2BGR);
        Core.flip(m, mat, 0);
        Highgui.imwrite("/sdcard/stitch/stitchfbo.jpg",mat);

        //END OF IMWRITE
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);
        mUpdate = false;
    }


}
