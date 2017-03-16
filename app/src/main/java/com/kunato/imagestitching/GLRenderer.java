package com.kunato.imagestitching;

import android.graphics.SurfaceTexture;
import android.location.Location;
import android.opengl.GLES11Ext;
import android.opengl.GLES31;
import android.opengl.GLSurfaceView;
import android.util.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// Renderer
public class GLRenderer implements GLSurfaceView.Renderer, SurfaceTexture.OnFrameAvailableListener {
    private long mStartTime;
    private int mFrame;
    private SurfaceTexture mTextureNormal;
    private SurfaceTexture mTextureProcessed;
    private boolean mUpdateST = false;
    protected MainController mView;
    public boolean mUsingOldMatrix = false;
    public float mFadeAlpha = 1.0f;
    private final float[] mMVPMatrix = new float[16];
    public float[] mProjectionMatrix = new float[16];
    private final float[] mViewCanvasMatrix = new float[16];
    private final float SCREEN_RATIO = 0.6239168f;
    public static final float ZOOM_RATIO = 1.7f;
    private final float CANVAS_SIZE = 1f * ZOOM_RATIO;
    private final float HEIGHT_WIDTH_RATIO = 1f;
    private boolean mFirstFrame = true;
    private List<ARObject> mARObject = new ArrayList<>();
    public float[] mRotationMatrix = {1f,0,0,0
            ,0,1f,0,0
            ,0,0,1f,0
            ,0,0,0,1f};
    public float[] mPreviousRotMatrix = {1f,0,0,0
            ,0,1f,0,0
            ,0,0,1f,0
            ,0,0,0,1f};
    private float[] mSpericalModelMatrix = {1f,0,0,0
            ,0,1f,0,0
            ,0,0,1f,0
            ,0,0,0,1f};
    private float[] mWorldModelMatrix = {1f,0,0,0
            ,0,1f,0,0
            ,0,0,1f,0
            ,0,0,0,1f};
    private int[] mTexturePool = new int[2];
    private float[] mModelViewMatrix = new float[16];
    private CanvasObject mCanvasObject;
    private CanvasObject mCanvasObjectProcessed;
    public int mWidth;
    public int mHeight;
    public float[] mHomography = {1,0,0,0,1,0,0,0,1};
    private boolean readInProgress = false;
    private StitchObject mStitch;

    GLRenderer(MainController view) {
        mView = view;
    }
    public void captureScreen(){
        readInProgress = true;
    }
    public void onSurfaceCreated ( GL10 unused, EGLConfig config ) {
        mStartTime = System.nanoTime();
        //Note10.1
//        float[] vertices = {-CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                -CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f};
        //Nexus5x
//        float[] vertices = {-CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                -CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
//                CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f};

        float[] vertices = {CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
                -CANVAS_SIZE, CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
                CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f,
                -CANVAS_SIZE, -CANVAS_SIZE * HEIGHT_WIDTH_RATIO, -1.0f};

        float[] textures = {0.0f, 0.0f,
                0.0f, 1.0f,
                1.0f, 0.0f,
                1.0f, 1.0f};
        mStitch = new StitchObject(this);
        mCanvasObject = new CanvasObject(vertices,textures, mView.getActivity());
        mCanvasObjectProcessed = new CanvasObject(vertices,textures,mView.getActivity());


        mTextureNormal = new SurfaceTexture (mCanvasObject.getTexturePos()[0]);
        mTextureProcessed = new SurfaceTexture(mCanvasObjectProcessed.getTexturePos()[0]);
        mTextureNormal.setOnFrameAvailableListener(this);
        mTextureProcessed.setOnFrameAvailableListener(this);

        GLES31.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        //(x (vertical),(horizontal)y,z)
        GLES31.glEnable(GLES31.GL_BLEND);
        GLES31.glBlendFunc(GLES31.GL_SRC_ALPHA, GLES31.GL_ONE_MINUS_SRC_ALPHA);
        prepareARObject();
    }
    public void prepareARObject(){
        mARObject.add(new ARObject(this,1,"Sentan",34.732764, 135.734837));
        mARObject.add(new ARObject(this,2,"IS",34.732118, 135.734693));
        mARObject.add(new ARObject(this,3,"Dormitory",34.732039, 135.735305));
    }
    public void initARObject(int angle, Location location, float adjustment){
        Log.d("GLRenderer","AddARObject");
        for(int i =  0 ; i < mARObject.size() ;i++){
            mARObject.get(i).setCameraRotation(angle, location, adjustment);
        }

    }
    public void setAdjustment(float adjustment){
        Log.d("GLRenderer","AddARObject");
        for(int i =  0 ; i < mARObject.size() ;i++){
            mARObject.get(i).setAdjustment(adjustment);
        }

    }
    //Core function
    public void onDrawFrame ( GL10 unused ) {
        mFrame++;
        if(System.nanoTime() - mStartTime >= 1000000000){
            Log.v("FPS","fps : "+mFrame);
            mFrame = 0;
            mStartTime = System.nanoTime();
        }

        GLES31.glClear(GLES31.GL_COLOR_BUFFER_BIT);
        GLES31.glClear(GLES31.GL_DEPTH_BUFFER_BIT);

//        Matrix.multiplyMM(mModelViewMatrix,0, mRotationMatrix,0, mSpericalModelMatrix,0);
//        Matrix.multiplyMM(mMVPMatrix, 0, mProjectionMatrix,0, mModelViewMatrix,0);


//        float[] out = new float[3];
//        int[] viewport = {0,0,mWidth,mHeight};
//        GLU.gluProject(123.4349f, 0, -169.89357f, mRotationMatrix, 0, mProjectionMatrix, 0, viewport,0,out,0);
//        Log.d("gluProject", "" + Arrays.toString(out));
        //draws
        mStitch.drawTOFBO();

        synchronized(this) {
            if ( mUpdateST ) {
                //choose whice texture to update
                //maybe no need for mTextureNormal
                mTextureProcessed.updateTexImage();
                mUpdateST = false;
            }
        }

        //multiply MM(retMat, retMatOffset, mat1 * mat2 (includeOffset))
//        mCanvasObject.draw(mMVPMatrix);
        //Test mRotation
        mCanvasObjectProcessed.draw(mViewCanvasMatrix,mHomography);
        for(int i = 0 ; i < mARObject.size() ; i++)
            mARObject.get(i).draw(mRotationMatrix,mProjectionMatrix);
        GLES31.glFlush();
    }

    public void onSurfaceChanged ( GL10 unused, int width, int height ) {
        mWidth = width;
        mHeight = height;
        GLES31.glViewport(0, 0, mWidth, mHeight);

        GLES31.glRenderbufferStorage(GLES31.GL_RENDERBUFFER, GLES11Ext.GL_RGBA8_OES,mWidth,mHeight);
        Log.v("GL", "Screen" + String.format("(Width:Height)[%d,%d]", mWidth,mHeight));

        //48=zoom1.5//72=zoom1
        //52 for height
        //65 default
        //Matrix.perspectiveM(mProjectionMatrix, 0, 50 / ZOOM_RATIO, ratio, 0.1f, 1000f);//48 for 4/3 64 for 1920/1080

        mProjectionMatrix = Util.glProjectionMatrix();
        for(int i = 0 ; i < mProjectionMatrix.length ;i++){
            Log.d("Matrix",""+mProjectionMatrix[i]);
        }

    }
    public void setHomography(float[] input){
        mHomography = input;
        Log.v("GL","SetHomography" + Arrays.toString(input));
    }
    public synchronized void onFrameAvailable ( SurfaceTexture st ) {
        mUpdateST = true;
        mView.requestRender();
    }

    public void close() {
        mUpdateST = false;
        mTextureNormal.release();
        mCanvasObject.deleteTex();
    }

    public void setRotationMatrix(float[] rotationMatrix){
        //mPreviousRotMatrix = mRotationMatrix;
        mRotationMatrix = rotationMatrix;
    }
    public SphereObject getSphere(){
        return null;
    }
    public CanvasObject getCanvas() { return mCanvasObject; }
    public SurfaceTexture getSurfaceTexture(){
        return mTextureNormal;
    }
    public SurfaceTexture getProcessSurfaceTexture() {
        return mTextureProcessed;
    }
    public StitchObject getStitch() { return mStitch; }

}