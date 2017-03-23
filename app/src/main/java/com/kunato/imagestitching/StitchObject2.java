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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.microedition.khronos.opengles.GL;


public class StitchObject2 {

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
            "#define WINDOW_SIZE 50.0\n" +
            "#define SCALE 1453.8\n"+
            "precision highp float;\n" +
            "precision highp int;\n" +
            "uniform int length;\n" +
            "uniform sampler2D sTexture[2];\n" +
            "uniform ivec2 size[2];\n" +
            "uniform ivec2 corner[2];\n" +
            "uniform mat3 k_rinv[1];\n" +
            "uniform ivec2 winSize;\n" +
            "uniform ivec2 tl;\n" +
            "in vec2 texCoord;\n" +
            "out vec4 fragmentColor;\n" +
            "int idx = 0;\n" +
            "float ssd;\n" +
            "vec3 mapBackward(in vec3 uv, in float scale, in mat3 matrix){\n" +
            "   uv/=scale;\n" +
            "   float sinv = sin(M_PI - uv.y);\n" +
            "   vec3 temp = vec3(sinv * sin(uv.x), cos(M_PI - uv.y), sinv * cos(uv.x) );\n" +
            "   vec3 ret = matrix * temp;\n" +
            "   if(ret.z < 0.0){\n" +
            "       ret.x = -1.0;ret.y = -1.0;" +
            "   }else{\n" +
            "       ret.x/=ret.z;ret.y/=ret.z;\n" +
            "   }\n" +
            "   return ret;\n" +
            "}\n" +
            "vec4 getSampleFromArray(in int ndx,in ivec2 bpos,in ivec2 bpos2,ivec2 corner_i){\n" +
            "   ivec2 pos = bpos - corner_i;\n" +
            "   ivec2 pos2 = bpos2 - corner_i;\n" +
            "   vec4 color;\n" +
            "   if(ndx == 0){\n" +
            "       color = texelFetch(sTexture[0], ivec2(pos2.x,pos2.y) ,0);\n" +
            "       float blend_ratio = 1.0;\n" +
            "       float blend_x = 1.0;\n" +
            "       float blend_y = 1.0;\n" +
            "       if(corner[0].x > corner[1].x){" +
            "           if(float(pos.x) < WINDOW_SIZE){" +
            "               blend_x = float(pos.x)/WINDOW_SIZE;" +
            "           }" +
            "       }" +
            "       if(corner[1].x + size[1].x > corner[0].x + size[0].x){" +
            "           if(float(pos.x) > (float(size[0].x) - (WINDOW_SIZE * 1.5)) && pos.x < size[0].x){" +
            "               blend_x = - (float(pos.x) - float(size[0].x))/(WINDOW_SIZE * 1.5);\n" +
            "           }" +
            "       }" +
            "       if(corner[0].y > corner[1].y){" +
            "           if(float(pos.y) < WINDOW_SIZE){" +
            "               blend_y = float(pos.y)/WINDOW_SIZE;" +
            "           }" +
            "       }" +
            "       if(corner[1].y + size[1].y > corner[0].y + size[0].y){" +
            "           if(float(pos.y) > (float(size[0].y) - (WINDOW_SIZE * 1.5)) && pos.y < size[0].y){" +
            "               blend_y = - (float(pos.y) - float(size[0].y))/(WINDOW_SIZE * 1.5);\n"+
            "           } " +
            "       }" +
            "       if(blend_x > blend_y){" +
            "           blend_ratio = blend_y;" +
            "       }" +
            "       else{" +
            "           blend_ratio = blend_x;" +
            "       }" +
            "       color *= blend_ratio;" +
            "   }"+
            "   else if(ndx == 1){\n" +
            "       ivec2 half_corner = ivec2(float(corner[0].x)/2.0,float(corner[0].y)/2.0);" +
            "       ivec2 diff = bpos + tl;" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[0]);\n" +
            "       color = texelFetch(sTexture[1], ivec2(int(round(map.y)),size[1].x - int(round(map.x))),0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[1].x && int(map.y) < 0 && int(map.y) >= size[1].y ){" +
            "           color.a = 0.0;\n" +
            "       }" +
            "   }\n" +
            "   return color;\n"+
            "}\n" +
            "void main() {\n" +
            "   int idx;" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,int(gl_FragCoord.y));" +
            "   vec4 color = vec4(0,0,0,0);" +
            "   for(int i = 1 ; i >= 0 ; i--){" +
            "       if(corner[i].x < pos.x && corner[i].x + size[i].x > pos.x && corner[i].y < pos.y && corner[i].y + size[i].y > pos.y){" +
            "           ivec2 pos2 = ivec2(gl_FragCoord.x, int(gl_FragCoord.y));" +
            "           ivec2 diff = pos - corner[i];" +
            "           ivec2 diff2 = pos2 - corner[i];" +

            "           if(length == 0){" +
            "               color = getSampleFromArray(i,pos,pos2,corner[i]);\n" +
            "               break;" +
            "           }else{" +
            "               vec4 color_top = getSampleFromArray(i,pos,pos2,corner[i]);\n" +
            "               color *= (1.0-color_top.a);\n" +
            "               color += color_top;\n" +
            "               //if(i == 1){color = vec4(1,1,0,1);}else{color = vec4(1,0,1,1);}\n" +
            "           }" +
            "       }" +
            "   }" +
            "   if(color.a > 0.0){" +
            "       color/=color.a;\n" +
            "       fragmentColor = color;"+
            "   }" +
            "}";


    private final String fss_int =
            "precision mediump float;\n" +
                    "uniform sampler2D sTexture;\n"+
                    "varying vec2 texCoord;\n" +
                    "void main() {\n" +
                    "gl_FragColor = texture2D(sTexture,texCoord);" +
                    "}";
    private int mCount = 0;
    private final FloatBuffer pVertex;
    private final FloatBuffer pTexCoord;
    private int mTLX;
    private int mTLY;
    private final int mProgram;
    private final int mSubProgram;
    public final int NUMBER_OF_TEXTURE = 2;
    private int mLastSSDSum = 0;
    //Only one texture
    private int[] mTextures = new int[NUMBER_OF_TEXTURE];
    private int[] corners = new int[NUMBER_OF_TEXTURE*2];
    private int[] sizes = new int[NUMBER_OF_TEXTURE*2];
    private int[] mFBO = new int[1];
    private int[] mSFBO = new int[1];
    private int[] mLastFBO = new int[3];
    private int[] mLastSFBO = new int[3];
    private int mCurrentSize = 2;
    private int mFBOID = 0;
    private int mFBOTex = 0;
    private int mFBORBuffer = 0;
    private int[] mSFBOID = new int[2];
    private int[] mSFBOTex = new int[2];
    private int[] mSFBORBuffer = new int[2];
    private int mLastSFBOIndex = 0;
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

    public StitchObject2(GLRenderer renderer) {
        Context context = renderer.mView.getActivity();
        glRenderer = renderer;
        mProgram = Util.loadShader(vertexShaderCode, fragmentShaderCode);
        mSubProgram = Util.loadShader(vertexShaderCode, fss_int);
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
        generateFBO();
        generateSFBO(0);
        generateSFBO(1);



    }
    public void generateFBO(){
        GLES31.glDeleteFramebuffers(1,mLastFBO,0);
        GLES31.glDeleteTextures(1,mLastFBO,1);
        GLES31.glDeleteBuffers(1,mLastFBO,2);
        mLastFBO[0] = mFBOID;
        mLastFBO[1] = mFBOTex;
        mLastFBO[2] = mFBORBuffer;
        GLES31.glGenFramebuffers(1, mFBO, 0);
        mFBOID = mFBO[0];
        GLES31.glGenTextures(1, mFBO, 0);
        mFBOTex = mFBO[0];
        GLES31.glGenRenderbuffers(1, mFBO, 0);
        mFBORBuffer = mFBO[0];
    }
    public void generateSFBO(int i){
        GLES31.glDeleteFramebuffers(1,mLastSFBO,0);
        GLES31.glDeleteTextures(1,mLastSFBO,1);
        GLES31.glDeleteBuffers(1,mLastSFBO,2);
        GLES31.glGenFramebuffers(1, mSFBO, 0);
        mSFBOID[i] = mSFBO[0];
        GLES31.glGenTextures(1, mSFBO, 0);
        mSFBOTex[i] = mSFBO[0];
        GLES31.glGenRenderbuffers(1, mSFBO, 0);
        mSFBORBuffer[i] = mSFBO[0];
    }

    public int createFBO(){
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
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, mFBORBuffer);
        GLES31.glRenderbufferStorage(GLES31.GL_RENDERBUFFER, GLES31.GL_DEPTH_COMPONENT16, mWidth, mHeight);
        //Attach texture FBO color attachment
        GLES31.glFramebufferTexture2D(GLES31.GL_FRAMEBUFFER, GLES31.GL_COLOR_ATTACHMENT0, GLES31.GL_TEXTURE_2D, mFBOTex, 0);
        //Attach render buffer to depth attachment
        GLES31.glFramebufferRenderbuffer(GLES31.GL_FRAMEBUFFER, GLES31.GL_DEPTH_ATTACHMENT, GLES31.GL_RENDERBUFFER, mFBORBuffer);

        int[] wh = new int[2];
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_WIDTH, wh, 0);
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_HEIGHT, wh, 1);
        //we are done, reset
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, 0);
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, 0);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);

        return mFBO[0];
    }
    public int createSFBO(){
        //Bind Frame buffer
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, mSFBOID[mLastSFBOIndex]);
        //Bind texture
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, mSFBOTex[mLastSFBOIndex]);
        //Define texture parameters
        GLES31.glTexImage2D(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_RGBA, mWidth, mHeight, 0, GLES31.GL_RGBA, GLES31.GL_UNSIGNED_BYTE, null);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        //Bind render buffer and define buffer dimension
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, mSFBORBuffer[mLastSFBOIndex]);
        GLES31.glRenderbufferStorage(GLES31.GL_RENDERBUFFER, GLES31.GL_DEPTH_COMPONENT16, mWidth, mHeight);
        //Attach texture FBO color attachment
        GLES31.glFramebufferTexture2D(GLES31.GL_FRAMEBUFFER, GLES31.GL_COLOR_ATTACHMENT0, GLES31.GL_TEXTURE_2D, mSFBOTex[mLastSFBOIndex], 0);
        //Attach render buffer to depth attachment
        GLES31.glFramebufferRenderbuffer(GLES31.GL_FRAMEBUFFER, GLES31.GL_DEPTH_ATTACHMENT, GLES31.GL_RENDERBUFFER, mSFBORBuffer[mLastSFBOIndex]);

        int[] wh = new int[2];
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_WIDTH, wh, 0);
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_HEIGHT, wh, 1);
        //we are done, reset
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, 0);
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, 0);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);

        return mSFBO[0];
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

        mUpdate = true;
    }

    public void loadBitmap(){

        GLES31.glActiveTexture(GLES31.GL_TEXTURE_2D);
        if(mLastSFBOIndex == 1){
            this.mTextures[0] = this.mSFBOTex[0];
        }
        else{

            this.mTextures[0] = this.mSFBOTex[1];
        }
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, this.mTextures[0]);
        GLES31.glActiveTexture(GLES31.GL_TEXTURE0);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S,GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T,GLES31.GL_CLAMP_TO_EDGE);
        int[] wh = new int[2];
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_WIDTH, wh, 0);
        GLES31.glGetTexLevelParameteriv(GLES31.GL_TEXTURE_2D, 0, GLES31.GL_TEXTURE_HEIGHT, wh, 1);
        Log.d("GPU Stitching","Texture Size : ["+wh[0]+","+wh[1]+"]");
        GLES31.glActiveTexture(GLES31.GL_TEXTURE1);
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, this.mTextures[1]);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S,GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T,GLES31.GL_CLAMP_TO_EDGE);
        GLUtils.texImage2D(GLES31.GL_TEXTURE_2D, 0, mCPUBitmap[1], 0);
        GLES31.glGenerateMipmap(GLES31.GL_TEXTURE_2D);
       // mCPUBitmap[1].recycle();
        updateParam();

        createFBO();
        createSFBO();

    }

    public void bitmapToCPU(Bitmap bitmap,int index){
        mCPUBitmap[index] = bitmap;

    }

    public void updateParam(){

        float min_x = 10000;
        float min_y = 10000;
        for(int i = 0 ; i < mCurrentSize*2 ;i++){
            if(cof[i] < min_x && i%2 == 0 && sof[i] != 0) {
                min_x = cof[i];
            }
            if(cof[i] < min_y && i%2 == 1 && sof[i] != 0) {
                min_y = cof[i];

            }
        }
        //OK
        mTLX = (int) min_x;
        mTLY = (int) min_y;

        float max_x = -10000;
        float max_y = -10000;

        for(int i = 0 ; i < mCurrentSize*2 ;i++){
            if(cof[i] + sof[i] > max_x && i%2 == 0 && sof[i] != 0){
                max_x = cof[i] + sof[i];
            }
            if(cof[i] + sof[i] > max_y && i%2 == 1 && sof[i] != 0){
                max_y = cof[i] + sof[i];
            }
        }
        for(int i = 0 ; i < mCurrentSize*2 ;i++){
            if(i%2 == 0 && sof[i] != 0){
                cof[i] -=mTLX;
            }else if(i%2 == 1 && sof[i] != 0){
                cof[i] -=mTLY;
            }
        }
        corners = cof;
        mWidth = (int) (max_x-min_x);

        mHeight = (int) (max_y-min_y);
//        mWidth = 1495;
//        mHeight = 1719;
        Log.d("Stitch GPU","Size ("+mWidth+","+mHeight+") , TL ("+mTLX+","+mTLY+")");
        sizes = sof;
    }
    public int getFBOTexture(){
        if(mLastSFBOIndex == 1){
            return this.mSFBOTex[0];
        }
        else{

            return this.mSFBOTex[1];
        }
    }

    public void drawTOFBO() {
        if(!mUpdate)
            return;
        long start = System.currentTimeMillis();
        loadBitmap();
        int count = 0;

        long start2 = System.currentTimeMillis();
        GLES31.glUseProgram(mProgram);
        GLES31.glBlendFunc(GLES31.GL_SRC_ALPHA, GLES31.GL_ONE_MINUS_SRC_ALPHA);
//        GLES31.glBlendFunc(GLES31.GL_ONE,GLES31.GL_ZERO);
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
        int k_rinvh = GLES31.glGetUniformLocation(mProgram,"k_rinv");
        int wsh = GLES31.glGetUniformLocation(mProgram,"winSize");
        int tlh = GLES31.glGetUniformLocation(mProgram,"tl");
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
        GLES31.glUniform1i(lh,mCount);

        Log.d("Stitch GPU","G_Corners : "+Arrays.toString(corners));
        Log.d("Stitch GPU","G_Sizes : "+Arrays.toString(sizes));
        Log.d("Stitch GPU","G_WinSize : ("+mWidth+","+mHeight+")");
        Log.d("Stitch GPU","G_Length : "+mCount);
        Log.d("Stitch GPU","TexArray : "+Arrays.toString(texArray));
        Log.d("Stitch GPU","k_rinv : "+Arrays.toString(k_rinvData));
        GLES31.glUniform2iv(sh,mCurrentSize,sizes,0);
        GLES31.glUniform2iv(ch,mCurrentSize,corners,0);
        GLES31.glUniform1iv(th,mCurrentSize,texArray,0);
        GLES31.glUniform2i(wsh,mWidth,mHeight);
        GLES31.glUniform2i(tlh,mTLX,mTLY);
        GLES31.glUniformMatrix3fv(k_rinvh,1,true,k_rinvData,0);
        GLES31.glDrawArrays(GLES31.GL_TRIANGLE_STRIP, 0, 4);
        long end = System.currentTimeMillis();
        Log.d("Stitch GPU","TimeSpend : "+(end-start2)*0.001+ " : "+(start2-start)*0.001 + " ; "+count);

        //END OF RENDERING

//        try {
//            bos = new BufferedOutputStream(new FileOutputStream(new File("/sdcard/stitch/test.mat")));
//            bos.write(alphaBuffer);
//            bos.flush();
//            bos.close();
//            Log.d("Stitch GPU","Writing File Success");
//        } catch (Exception e) {
//            Log.d("Stitch GPU","Writing File Error");
//        }

        //Log.d("Stitch GPU","Sum : "+sum+" countI : "+countI +" maxI : " + maxI+" Avg : "+sum/countI +" Avg on last : "+(sum-mLastSSDSum)/(countI-mLastSSDCount));


        //END OF IMWRITE
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);
        mUpdate = false;
        GLES31.glBlendFunc(GLES31.GL_SRC_ALPHA, GLES31.GL_ONE_MINUS_SRC_ALPHA);
        drawTOSFBO();



    }
    public void drawTOSFBO(){
        GLES31.glUseProgram(mSubProgram);
        GLES31.glViewport(0,0,mWidth,mHeight);
        GLES31.glClear(GLES31.GL_COLOR_BUFFER_BIT);
        GLES31.glClear(GLES31.GL_DEPTH_BUFFER_BIT);
        pVertex.position(0);
        pTexCoord.position(0);
        int ph = GLES31.glGetAttribLocation(mSubProgram,"vPosition");
        int tch = GLES31.glGetAttribLocation(mSubProgram,"vTexCoord");
        int th = GLES31.glGetUniformLocation ( mSubProgram, "sTexture" );
        GLES31.glVertexAttribPointer(ph, 2, GLES31.GL_FLOAT, false, 4*2, pVertex);
        GLES31.glVertexAttribPointer(tch, 2, GLES31.GL_FLOAT, false, 4*2, pTexCoord );
        GLES31.glEnableVertexAttribArray(ph);
        GLES31.glEnableVertexAttribArray(tch);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, mSFBOID[mLastSFBOIndex]);
        GLES31.glActiveTexture(GLES31.GL_TEXTURE3);
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, mFBOTex );
        GLES31.glUniform1i(th, 0);

        GLES31.glDrawArrays(GLES31.GL_TRIANGLE_STRIP, 0, 4);
        mCount++;

//        if(mCount == 1) {
//            mScreenBuffer = ByteBuffer.allocateDirect(mHeight * mWidth * 4);
//            mScreenBuffer.order(ByteOrder.nativeOrder());
//            GLES31.glReadPixels(0, 0, mWidth, mHeight, GLES31.GL_RGBA, GLES31.GL_UNSIGNED_BYTE, mScreenBuffer);
//
//            mScreenBuffer.rewind();
//            byte pixelsBuffer[] = new byte[4 * mHeight * mWidth];
//            mScreenBuffer.get(pixelsBuffer);
//            Mat mat = new Mat(mHeight, mWidth, CvType.CV_8UC4);
//            mat.put(0, 0, pixelsBuffer);
//            Mat bgr = new Mat();
//            Imgproc.cvtColor(mat,bgr,Imgproc.COLOR_RGBA2BGRA);
//            Imgcodecs.imwrite("/sdcard/stitch/stitchResult1.jpg", bgr);
//        }
//
//        if(mCount >= 2) {
//            mScreenBuffer = ByteBuffer.allocateDirect(mHeight * mWidth * 4);
//            mScreenBuffer.order(ByteOrder.nativeOrder());
//            GLES31.glReadPixels(0, 0, mWidth, mHeight, GLES31.GL_RGBA, GLES31.GL_UNSIGNED_BYTE, mScreenBuffer);
//
//            mScreenBuffer.rewind();
//            byte pixelsBuffer[] = new byte[4 * mHeight * mWidth];
//            mScreenBuffer.get(pixelsBuffer);
//            Mat mat = new Mat(mHeight, mWidth, CvType.CV_8UC4);
//            mat.put(0, 0, pixelsBuffer);
//
//            Mat bgr = new Mat();
//            Imgproc.cvtColor(mat,bgr,Imgproc.COLOR_RGBA2BGRA);
//            Imgcodecs.imwrite("/sdcard/stitch/stitchResult"+mCount+".jpg", bgr);
//        }

        Log.d("GPU Stitching","Stitch Complete for "+mCount+" Images.");
        glRenderer.getSphere().mReadyToUpdate = true;
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);
        if(mLastSFBOIndex == 1){
            mLastSFBOIndex = 0;
        }
        else{
            mLastSFBOIndex = 1;
        }
    }


}
