package com.kunato.imagestitching;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

/**
 * Created by kunato on 12/21/15 AD.
 */
public class ImageStitchingNative {
    private static ImageStitchingNative instance = null;
    private Context context;
    private Bitmap mUploadingBitmap = null;
    private float[] mBitmapArea;
    private boolean mStop = false;
    private ImageStitchingNative(){
    }

    public native int nativeKeyFrameSelection(float[] rotMat);
    public native void nativeAligning(long imgAddr,long glRotAddr,long retMatAddr);
    public native int nativeStitch(long retAddr,long areaAddr,long rotAddr,long refineRotAddr,long roiAddr,long k_rinvAddr);
    public native void nativeAddStitch(long imgAddr,long rotAddr);
    public int keyFrameSelection(float[] rotMat) {
        if(mStop)
            return 0;
        return nativeKeyFrameSelection(rotMat);
    }
    public int addToPano(Mat imageMat, Mat rotMat,int mPictureSize){
        Log.d("Java Stitch","Current Picture Size : "+mPictureSize);
        //Load Image to GPU

        Mat rgba = new Mat(imageMat.cols(),imageMat.rows(),CvType.CV_8UC4);
        Core.flip(imageMat.t(),imageMat,1);
        Imgproc.cvtColor(imageMat,rgba,Imgproc.COLOR_BGR2RGBA);
        Bitmap iBitmap = Bitmap.createBitmap(rgba.cols(),rgba.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(rgba,iBitmap);

        Factory.getFactory(null).getGlRenderer().getStitch().bitmapToCPU(iBitmap,mPictureSize);

        Factory.mainController.startRecordQuaternion();
        Log.d("Java Stitch", "Image Input Size : "+imageMat.size().width + "*" + imageMat.size().height);
        Mat ret = new Mat();
        Mat roi = new Mat();
        Mat k_rinv = new Mat();

        Mat area = new Mat(1,4,CvType.CV_32F);
        Mat rot = new Mat(3,3,CvType.CV_32F);
        Log.d("Java Stitch", "Image Rotation Input : "+rotMat.dump());
        nativeAddStitch(imageMat.getNativeObjAddr(), rotMat.getNativeObjAddr());
        Mat refinedMat = new Mat(4,4,CvType.CV_32F);
        int rtCode = nativeStitch(ret.getNativeObjAddr(), area.getNativeObjAddr(),rot.getNativeObjAddr(),refinedMat.getNativeObjAddr(),roi.getNativeObjAddr(),k_rinv.getNativeObjAddr());
        Log.d("Java Stitch", "JNI Return Code : "+rtCode + "");
        float[] areaFloat = new float[4];
        area.get(0, 0, areaFloat);

        Log.d("Java Stitch", "Return Area [" + Arrays.toString(areaFloat)+"]");

        if(rtCode == -1){
            return 1;
        }
        if(rtCode != 1) {
            return rtCode;
        }
        int[] roiData = new int[roi.rows()*roi.cols()];
        float[] k_rinvData = new float[k_rinv.rows()*k_rinv.cols()];
        roi.get(0, 0, roiData);
        k_rinv.get(0,0,k_rinvData);
        Log.d("Java Stitch", "ROI : "+Arrays.toString(roiData));
        Log.d("Java Stitch", "K_RINV : "+Arrays.toString(k_rinvData));

        //Bitmap bitmap = Bitmap.createBitmap(ret.cols(), ret.rows(), Bitmap.Config.ARGB_8888);
        //Mat test = new Mat(ret.height(),ret.width(),CvType.CV_8UC3);
        //Imgproc.cvtColor(ret, test, Imgproc.COLOR_BGRA2RGB);
        //Highgui.imwrite("/sdcard/stitch/pano"+mPictureSize+".jpg",test);

        //Utils.matToBitmap(ret, bitmap);
        //Log.d("JAVA Stitch", "Add Panorama Finished, Size :" + ret.size().width + "," + ret.size().height
        float[] refinedMatArray = new float[16];
        refinedMat.get(0, 0, refinedMatArray);
        float[] refinedQuad = Util.matrixToQuad(refinedMatArray);
        Factory.mainController.updateQuaternion(refinedQuad, Factory.mainController.mDeltaQuaternion);
        //Stop on click
        if(mStop) {
            //Send ROI to GPU
            Factory.getFactory(null).getGlRenderer().getStitch().setROI(roiData, k_rinvData,mPictureSize);

            Log.d("Java Stitch", "Refined Matrix : " + Arrays.toString(refinedMatArray));
            Log.d("Java Stitch", "Refined Quad : " + Arrays.toString(refinedQuad));

            mBitmapArea = areaFloat;


            Factory.getFactory(null).getGlRenderer().getSphere().updateArea(mBitmapArea);
            Factory.getFactory(null).getGlRenderer().mDisplayAR = true;
        }
        return rtCode;
    }

    public void setContext(Context context){

        this.context = context;
    }
    public void stop(){
        mStop = true;
    }

    public static ImageStitchingNative getNativeInstance(){
        if(instance == null){
            instance = new ImageStitchingNative();
        }
        return instance;
    }
    static {
        System.loadLibrary("native");
    }
}
