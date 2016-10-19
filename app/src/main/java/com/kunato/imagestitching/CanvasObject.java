package com.kunato.imagestitching;

import android.content.Context;
import android.opengl.GLES11Ext;
import android.opengl.GLES31;
import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Arrays;

/**
 * Created by kunato on 1/6/16 AD.
 */
public class CanvasObject {
    private final String vss =
            "uniform mat4 uMVPMatrix;\n" +
                    "attribute vec3 vPosition;\n" +
                    "attribute vec2 vTexCoord;\n" +
                    "varying vec2 texCoord;\n" +
                    "void main() {\n" +
                    "  texCoord = vTexCoord;\n" +
                    "  gl_Position = vec4 ( vPosition.x, vPosition.y, vPosition.z, 1.0 );\n" +
                    "}";
    private final String fss_ext =
            "#extension GL_OES_EGL_image_external : require\n" +
                    "precision mediump float;\n" +
                    "uniform samplerExternalOES sTexture;\n" +
                    "uniform mat3 homography;" +
                    "uniform float width;" +
                    "uniform float height;\n" +
                    "varying vec2 texCoord;\n" +
                    "float determinant(mat3 m) {\n" +
                    "  return   m[0][0]*( m[1][1]*m[2][2] - m[2][1]*m[1][2])\n" +
                    "         - m[1][0]*( m[0][1]*m[2][2] - m[2][1]*m[0][2])\n" +
                    "         + m[2][0]*( m[0][1]*m[1][2] - m[1][1]*m[0][2]) ;\n" +
                    "  }" +
                    "mat3 inverse(mat3 m) {\n" +
                    "  float d = 1.0 / determinant(m) ;\n" +
                    "  return d * mat3( m[2][2]*m[1][1] - m[1][2]*m[2][1],\n" +
                    "                    m[1][2]*m[2][0] - m[2][2]*m[1][0],\n" +
                    "                     m[2][1]*m[1][0] - m[1][1]*m[2][0] ,\n" +
                    "\n" +
                    "                   m[0][2]*m[2][1] - m[2][2]*m[0][1],\n" +
                    "                    m[2][2]*m[0][0] - m[0][2]*m[2][0],\n" +
                    "                     m[0][1]*m[2][0] - m[2][1]*m[0][0],\n" +
                    "   \n" +
                    "                   m[1][2]*m[0][1] - m[0][2]*m[1][1],\n" +
                    "                    m[0][2]*m[1][0] - m[1][2]*m[0][0],\n" +
                    "                     m[1][1]*m[0][0] - m[0][1]*m[1][0]\n" +
                    "                 );\n" +
                    "  }" +
                    "vec2 convertToTexCoord(vec3 pixelCoords){" +
                    "pixelCoords /= pixelCoords.z;" +
                    "pixelCoords /= vec3(width,height,1.0);" +
                    "" +
                    "return pixelCoords.xy;}" +
                    "void main() {\n" +
                    "vec3 coord = vec3(texCoord.x,texCoord.y,1.0);" +
                    "vec4 color = texture2D(sTexture,convertToTexCoord(inverse(homography)*coord));\n" +
                    "float grayScale = dot(color.rgb, vec3(0.299, 0.587, 0.114));\n" +
                    "gl_FragColor = vec4(grayScale,grayScale,grayScale,1.0);\n" +
                    "//gl_FragColor = color;\n" +
                    "}";
    private final String fss_int =
            "precision mediump float;\n" +
                    "uniform sampler2D sTexture;\n"+
                    "varying vec2 texCoord;\n" +
                    "void main() {\n" +
                    "gl_FragColor = texture2D(sTexture,texCoord);" +
                    "gl_FragColor.a = 1.0;" +
                    "}";

//    public float[] mHomography = {0.70710678118f,-0.70710678118f,0, 0.70710678118f,0.70710678118f,0f, 0,0,1};
//    public float[] mHomography = {9.284416f,0.329349f,-589.334961f, 1.089121f,10.698853f,-309.189026f, 0.001062f,0.000253f,1.000000f};
//    public float[] mHomography = {1.455513f,-0.037048f,-506.760040f,0.061768f,1.757028f,-136.568909f,0.000019f,0.000011f,1.000000f};
//public float[] mHomography = {0.824879f, -0.02256f, 118.489403f/1080f, -0.013021f,0.979792f,156.191864f/1440f ,-0.000028f ,-0.000031f, 1.000000f};
    private FloatBuffer mVertexCoord;
    private FloatBuffer mTextureCoord;
    private int[] mTexture;
    private int mProgram;
    public static final int COORD_PER_VERTEX = 3;
    public static final int COORD_PER_TEXTURE = 2;
    public CanvasObject(float[] vertices, float[] textures, Context context) {
        this.mTexture = new int[1];
        mVertexCoord = ByteBuffer.allocateDirect(4 * vertices.length).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mVertexCoord.put(vertices);
        mVertexCoord.position(0);
        mTextureCoord = ByteBuffer.allocateDirect(4 * textures.length).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mTextureCoord.put(textures);
        mTextureCoord.position(0);
        mProgram = Util.loadShader(vss, fss_ext);
        GLES31.glGenTextures(1, mTexture, 0);
        GLES31.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, mTexture[0]);
        GLES31.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES31.GL_TEXTURE_WRAP_S, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES31.GL_TEXTURE_WRAP_T, GLES31.GL_MIRRORED_REPEAT);
        GLES31.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES31.GL_TEXTURE_MIN_FILTER, GLES31.GL_LINEAR);
        GLES31.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES31.GL_TEXTURE_MAG_FILTER, GLES31.GL_LINEAR);
    }


    public void draw(float[] mMVPMatrix,float[] mHomography){
        GLES31.glUseProgram(mProgram);
        int ph = GLES31.glGetAttribLocation(mProgram, "vPosition");
        int tch = GLES31.glGetAttribLocation (mProgram, "vTexCoord" );
        int th = GLES31.glGetUniformLocation(mProgram, "sTexture");
        int homoh = GLES31.glGetUniformLocation(mProgram,"homography");
        int widthh = GLES31.glGetUniformLocation(mProgram,"width");
        int heighth = GLES31.glGetUniformLocation(mProgram,"height");
        int mMVPMatrixHandle = GLES31.glGetUniformLocation(mProgram, "uMVPMatrix");

        GLES31.glActiveTexture(GLES31.GL_TEXTURE0);
        GLES31.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,mTexture[0]);
        GLES31.glUniform1i(th, 0);
        GLES31.glUniformMatrix4fv(mMVPMatrixHandle, 1, false, mMVPMatrix, 0);
        GLES31.glVertexAttribPointer(ph, COORD_PER_VERTEX, GLES31.GL_FLOAT, false, 4 * COORD_PER_VERTEX, mVertexCoord);
        GLES31.glVertexAttribPointer(tch, COORD_PER_TEXTURE, GLES31.GL_FLOAT,false,4*COORD_PER_TEXTURE,mTextureCoord);
        GLES31.glUniformMatrix3fv(homoh, 1, false, mHomography, 0);
        GLES31.glUniform1f(heighth, 1);
        GLES31.glUniform1f(widthh, 1);
        GLES31.glEnableVertexAttribArray(ph);
        GLES31.glEnableVertexAttribArray(tch);

        GLES31.glDrawArrays(GLES31.GL_TRIANGLE_STRIP, 0, 4);
        GLES31.glDisableVertexAttribArray(ph);
        GLES31.glDisableVertexAttribArray(tch);
        GLES31.glDisableVertexAttribArray(th);
    }

    public void deleteTex() {
        GLES31.glDeleteTextures(1, mTexture, 0);
    }
    public int[] getTexturePos(){
        return mTexture;
    }

}
