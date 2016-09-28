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
import android.graphics.BitmapFactory;
import android.opengl.GLES31;
import android.opengl.GLUtils;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.microedition.khronos.opengles.GL;

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
            "uniform int length;\n" +
            "uniform sampler2D sTexture[7];\n" +
            "uniform ivec2 size[7];\n" +
            "uniform ivec2 corner[7];\n" +
            "uniform mat3 r_kinv[7];\n" +
            "uniform mat3 k_rinv[7];\n" +
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
            "" +
            "   if(ndx == 0){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[0]);\n" +
            "       color = texelFetch(sTexture[0], ivec2(map.x,int(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[0].x && int(map.y) < 0 && int(map.y) >= size[0].y ){" +
            "           color.a = 0.0;" +
            "       }" +
//            "color = vec4(0,1,1,1);" +
            "   }\n" +
            "   else if(ndx == 1){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[1]);" +
            "       color = texelFetch(sTexture[1], ivec2(map.x,int(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[1].x && int(map.y) < 0 && int(map.y) >= size[1].y ){" +
            "           color.a = 0.0;" +
            "       }" +
//            "color = vec4(1,1,0,1);" +
            "   }\n" +
            "   else if(ndx == 2){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));\n" +
            "       ivec2 diff = pos + tl;\n" +
            "       vec3 map = mapBackward(vec3(diff.x,diff.y,1),SCALE,k_rinv[2]);" +
            "       color = texelFetch(sTexture[2], ivec2(map.x,int(map.y)) ,0);\n" +
            "       if(int(map.x) < 0 && int(map.x) >= size[2].x && int(map.y) < 0 && int(map.y) >= size[2].y ){" +
            "           color.a = 0.0;" +
            "       }" +
//            "color = vec4(1,0,1,1);" +
            "   }\n" +
            "   else if(ndx == 3){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,gl_FragCoord.y);\n" +
            "       ivec2 diff = pos - corner[3];\n" +
            "       color = texelFetch(sTexture[3], ivec2(diff.x,size[3].y - diff.y) ,0);\n" +
            "   }\n" +
            "   else if(ndx == 4){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,gl_FragCoord.y);\n" +
            "       ivec2 diff = pos - corner[4];\n" +
            "       color = texelFetch(sTexture[4], ivec2(diff.x,size[4].y - diff.y) ,0);\n" +
            "   }\n" +
            "   else if(ndx == 5){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,gl_FragCoord.y);\n" +
            "       ivec2 diff = pos - corner[5];\n" +
            "       color = texelFetch(sTexture[5], ivec2(diff.x,size[5].y - diff.y) ,0);\n" +
            "   }\n" +
            "   else if(ndx == 6){\n" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,gl_FragCoord.y);\n" +
            "       ivec2 diff = pos - corner[6];\n" +
            "       color = texelFetch(sTexture[6], ivec2(diff.x,size[6].y - diff.y) ,0);\n" +
            "   }\n" +
            "   return color;\n"+
            "}\n" +
            "void main() {\n" +
            "   int idx;" +
            "   ivec2 pos = ivec2(gl_FragCoord.x,winSize.y - int(gl_FragCoord.y));" +
            "   for(int i = length-1 ; i >= 0 ;i--){" +
            "       if(corner[i].x < pos.x + tl.x && corner[i].x + size[i].x > pos.x + tl.x && corner[i].y < pos.y + tl.y && corner[i].y + size[i].y > pos.y + tl.y){" +
            "           idx = i;\n" +
            "           vec4 color = getSampleFromArray(idx);\n" +
            "           if(color.a != 0.0){" +
            "               fragmentColor = color;" +
            "               break;" +
            "           }" +
            "       }" +
            "   }" +
            "   \n" +
            "   \n" +
            "}";
    private final FloatBuffer pVertex;
    private final FloatBuffer pTexCoord;
    private int mTLX;
    private int mTLY;
    public boolean mRealRender = false;
    private final int mProgram;
    public final int NUMBER_OF_TEXTURE = 3;
    //Only one texture
    private int[] mTextures = new int[NUMBER_OF_TEXTURE];
    private int[] corners = new int[NUMBER_OF_TEXTURE*2];
    private int[] sizes = new int[NUMBER_OF_TEXTURE*2];
    private int[] mFBO = new int[1];
    private int mFBOID;
    private int mFBOTex;

    private ByteBuffer mScreenBuffer;
    private GLRenderer glRenderer;
    private int mWidth = 0;
    private int mHeight = 0;
    public float[] mArea = {0,0,0,0};
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

        loadGLTexture(context);
        createFBO();

    }

    public void createFBO(){
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
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S, GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T, GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameteri(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST);
        //Bind render buffer and define buffer dimension
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, renderBufferId);
        GLES31.glRenderbufferStorage(GLES31.GL_RENDERBUFFER, GLES31.GL_DEPTH_COMPONENT16, mWidth, mHeight);
        //Attach texture FBO color attachment
        GLES31.glFramebufferTexture2D(GLES31.GL_FRAMEBUFFER, GLES31.GL_COLOR_ATTACHMENT0, GLES31.GL_TEXTURE_2D, mFBOTex, 0);
        //Attach render buffer to depth attachment
        GLES31.glFramebufferRenderbuffer(GLES31.GL_FRAMEBUFFER, GLES31.GL_DEPTH_ATTACHMENT, GLES31.GL_RENDERBUFFER, renderBufferId);
        //we are done, reset
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, 0);
        GLES31.glBindRenderbuffer(GLES31.GL_RENDERBUFFER, 0);
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);
    }

    public void loadBitmap(int i, Bitmap bitmap){

        GLES31.glActiveTexture(GLES31.GL_TEXTURE0+i);
        GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, this.mTextures[i]);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_NEAREST_MIPMAP_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_NEAREST);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_S,GLES31.GL_CLAMP_TO_EDGE);
        GLES31.glTexParameterf(GLES31.GL_TEXTURE_2D, GLES31.GL_TEXTURE_WRAP_T,GLES31.GL_CLAMP_TO_EDGE);
        GLUtils.texImage2D(GLES31.GL_TEXTURE_2D, 0, bitmap, 0);
        GLES31.glGenerateMipmap(GLES31.GL_TEXTURE_2D);

    }
    public void loadGLTexture(final Context context) {
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 1;
        GLES31.glGenTextures(2, this.mTextures, 0);

        for(int i = 0 ; i < mTextures.length ;i++){
            Bitmap bitmap = null;
            if(i == 0){
                try {
                    InputStream is = context.getAssets().open("img0.jpg");
                    bitmap = BitmapFactory.decodeStream(is);

                } catch (IOException e) {
                    Log.d("Stitch GPU","Loadfile Error");
                }
            }
            else if(i == 1){

                try {
                    InputStream is = context.getAssets().open("img1.jpg");
                    bitmap = BitmapFactory.decodeStream(is);
                } catch (IOException e) {
                    Log.d("Stitch GPU","Loadfile Error");
                }
            }
            else if(i == 2){
                try {
                    InputStream is = context.getAssets().open("img2.jpg");
                    bitmap = BitmapFactory.decodeStream(is);
                } catch (IOException e) {
                    Log.d("Stitch GPU","Loadfile Error");
                }
            }
            loadBitmap( i, bitmap);
//            mWidth += bitmap.getWidth();
//            if(mHeight < bitmap.getHeight()){
//                mHeight = bitmap.getHeight();
//            }
            bitmap.recycle();
            float[] size = new float[2];

            GLES31.glGetTexLevelParameterfv(GLES31.GL_TEXTURE_2D,0,GLES31.GL_TEXTURE_WIDTH,size,0);
            GLES31.glGetTexLevelParameterfv(GLES31.GL_TEXTURE_2D,0,GLES31.GL_TEXTURE_HEIGHT,size,1);
            Log.d("Stitch GPU","Loaded Texture Size :"+ Arrays.toString(size));
        }
        int[] cof = new int[]{-532,1437,-110,1408,360,1467};
        int[] sof = new int[]{1034,1701,1058,1701,1047,1700};
        float min_x = 10000;
        float min_y = 10000;
        for(int i = 0 ; i < cof.length ;i++){
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

        for(int i = 0 ; i < cof.length ;i++){
            Log.d("TEST","a"+i%2);
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



    public void draw() {
        long start = System.currentTimeMillis();
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
        int[] texArray = new int[mTextures.length];
        for(int i = 0 ; i < mTextures.length;i++){
            GLES31.glActiveTexture(GLES31.GL_TEXTURE0+i);
            GLES31.glBindTexture(GLES31.GL_TEXTURE_2D, mTextures[i]);
            texArray[i] = i;
        }
        GLES31.glUniform1i(lh,mTextures.length);

        Log.d("Stitch GPU","COF : "+Arrays.toString(corners));
        Log.d("Stitch GPU","SizeList : "+Arrays.toString(sizes));
        GLES31.glUniform2iv(sh,mTextures.length,sizes,0);
        GLES31.glUniform2iv(ch,mTextures.length,corners,0);
        GLES31.glUniform1iv(th,mTextures.length,texArray,0);
        GLES31.glUniform2i(tlh,mTLX,mTLY);
        GLES31.glUniform2i(wsh,mWidth,mHeight);
        float[] r_kinvData = new float[]{1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,};
        float[] k_rinvData = new float[]{ 1.46880334e+03f, 0.f, 5.56620422e+02f, 0.f, 1.46880334e+03f,  9.87914978e+02f, 0.f, 0.f, 1.f,
                1.56739111e+03f, -3.09704323e+01f, 9.76510925e+01f, 3.19323425e+02f, 1.44898230e+03f, 9.65319824e+02f, 2.95225173e-01f, -1.96384918e-02f, 9.55225825e-01f ,
                1.52318079e+03f, 3.44877005e+00f, -3.83559906e+02f, 5.54351318e+02f, 1.48857654e+03f, 7.81149048e+02f, 5.71894169e-01f, 2.03441400e-02f, 8.20075095e-01f ,
                 };
        GLES31.glUniformMatrix3fv(r_kinvh,mTextures.length,false,r_kinvData,0);
        GLES31.glUniformMatrix3fv(k_rinvh,mTextures.length,true,k_rinvData,0);
        GLES31.glDrawArrays(GLES31.GL_TRIANGLE_STRIP, 0, 4);

        long end = System.currentTimeMillis();
        Log.d("Stitching GPU","TimeSpend : "+(end-start)*0.001);
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
        GLES31.glBindFramebuffer(GLES31.GL_FRAMEBUFFER, 0);
    }


}
