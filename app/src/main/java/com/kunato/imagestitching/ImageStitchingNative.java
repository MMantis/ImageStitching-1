package com.kunato.imagestitching;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
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

    private ImageStitchingNative(){
    }

    public native int nativeKeyFrameSelection(float[] rotMat);
    public native void nativeAligning(long imgAddr,long glRotAddr,long retMatAddr);
    public native int nativeStitch(long retAddr,long areaAddr,long rotAddr,long refineRotAddr,long roiAddr,long k_rinvAddr);
    public native void nativeAddStitch(long imgAddr,long rotAddr);
    public native int track(long imgAddr,long retAddr,long areaAddr,long rotAddr,long refindRotAddr, long roiAddr,long k_rinvAddr);
    public int keyFrameSelection(float[] rotMat) {
        return nativeKeyFrameSelection(rotMat);
    }
    public int addToPano(Mat imageMat, Mat rotMat,int mPictureSize){
//        Log.d("Java Stitch","Current Picture Size : "+mPictureSize);
//        Mat ppImageMat = new Mat();
//        Core.flip(imageMat.t(),ppImageMat,0);
//        Bitmap iBitmap = Bitmap.createBitmap(ppImageMat.cols(),ppImageMat.rows(),Bitmap.Config.ARGB_8888);
//        Utils.matToBitmap(ppImageMat,iBitmap);
//        Mat tst = new Mat();
//        Imgproc.cvtColor(ppImageMat,tst,Imgproc.COLOR_RGBA2BGR);
//        Factory.getFactory(null).getGlRenderer().getStitch().bitmapToCPU(iBitmap,mPictureSize);
//
//        Factory.mainController.startRecordQuaternion();
//        Log.d("JAVA Stitch", "Image Input Size : "+ppImageMat.size().width + "*" + ppImageMat.size().height);
//        Mat ret = new Mat();
//        Mat roi = new Mat();
//        Mat k_rinv = new Mat();
//        Mat area = new Mat(1,4,CvType.CV_32F);
//        Mat rot = new Mat(3,3,CvType.CV_32F);
//        Log.d("JAVA Stitch", "Image Rotation Input : "+rotMat.dump());
//        nativeAddStitch(tst.getNativeObjAddr(), rotMat.getNativeObjAddr());
//        Mat refinedMat = new Mat(4,4,CvType.CV_32F);
//        int rtCode = nativeStitch(ret.getNativeObjAddr(), area.getNativeObjAddr(),rot.getNativeObjAddr(),refinedMat.getNativeObjAddr(),roi.getNativeObjAddr(),k_rinv.getNativeObjAddr());
//        Log.d("JAVA Stitch", "JNI Return Code : "+rtCode + "");
//        float[] areaFloat = new float[4];
//        area.get(0, 0, areaFloat);
//
//
//        //areaFloat[0]+=0;
//        //areaFloat[1]-=150;
//        Log.d("JAVA Stitch", "Return Area [" + Arrays.toString(areaFloat)+"]");
//
//        if(rtCode == -1){
//            return 1;
//        }
//        if(rtCode != 1) {
//            return rtCode;
//        }
//        int[] roiData = new int[roi.rows()*roi.cols()];
//        float[] k_rinvData = new float[k_rinv.rows()*k_rinv.cols()];
//        roi.get(0, 0, roiData);
//        k_rinv.get(0,0,k_rinvData);
//        Log.d("JAVA Stitch", "ROI : "+Arrays.toString(roiData));
//        Log.d("JAVA Stitch", "K_RINV : "+Arrays.toString(k_rinvData));
//
//        //Send ROI to GPU
//        Factory.getFactory(null).getGlRenderer().getStitch().setROI(roiData,k_rinvData);
//
//        //Bitmap bitmap = Bitmap.createBitmap(ret.cols(), ret.rows(), Bitmap.Config.ARGB_8888);
//        //Mat test = new Mat(ret.height(),ret.width(),CvType.CV_8UC3);
//        //Imgproc.cvtColor(ret, test, Imgproc.COLOR_BGRA2RGB);
//        //Highgui.imwrite("/sdcard/stitch/pano"+mPictureSize+".jpg",test);
//
//        //Utils.matToBitmap(ret, bitmap);
//        //Log.d("JAVA Stitch", "Add Panorama Finished, Size :" + ret.size().width + "," + ret.size().height);
//
//        float[] refinedMatArray = new float[16];
//        refinedMat.get(0,0,refinedMatArray);
//        float[] refinedQuad = Util.matrixToQuad(refinedMatArray);
//        Log.d("Java Stitch","Refined Matrix : "+Arrays.toString(refinedMatArray));
//        Log.d("Java Stitch","Refined Quad : "+Arrays.toString(refinedQuad));
//
//        Log.d("JAVA Stitch","Before Align Quad"+Arrays.toString(Factory.mainController.mQuaternion));
//        Factory.mainController.updateQuaternion(refinedQuad,Factory.mainController.mDeltaQuaternion);
//        Log.d("JAVA Stitch", "After Align Quad :"+Arrays.toString(Factory.mainController.mQuaternion));
//
//        mBitmapArea = areaFloat;
//
////        Factory.getFactory(null).getRSProcessor(null, null).requestAligning();;
////        Factory.mainController.requireAlign();
//
//        Factory.getFactory(null).getGlRenderer().getSphere().updateArea(mBitmapArea);
//
//        return rtCode;


//        Factory.mainController.startRecordQuaternion();
        Mat ret = new Mat();
        Mat area = new Mat(1,4,CvType.CV_32F);
        Mat refine = new Mat(4,4,CvType.CV_32F);
        Mat roi = new Mat(2,4,CvType.CV_32S);
        Bitmap iBitmap = Bitmap.createBitmap(imageMat.cols(),imageMat.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(imageMat,iBitmap);
        Factory.getFactory(null).getGlRenderer().getStitch().bitmapToCPU(iBitmap,1);
        Mat k_rinv = new Mat();
        int rtCode = track(imageMat.getNativeObjAddr(),ret.getNativeObjAddr(),area.getNativeObjAddr(),rotMat.getNativeObjAddr(),refine.getNativeObjAddr(),roi.getNativeObjAddr(),k_rinv.getNativeObjAddr());
        Log.d("Java Stitch","RtCode : "+rtCode);
        if(rtCode == -1){
            return 1;
        }
        if(rtCode != 1){
            return rtCode;
        }
        float[] areaFloat = new float[4];
        area.get(0, 0, areaFloat);


        //areaFloat[0]+=0;
        //areaFloat[1]-=150;
        Log.d("Java Stitch", "Return Area [" + Arrays.toString(areaFloat)+"]");

        int[] roiData = new int[roi.rows()*roi.cols()];
        float[] k_rinvData = new float[k_rinv.rows()*k_rinv.cols()];
        roi.get(0, 0, roiData);
        k_rinv.get(0,0,k_rinvData);
        Log.d("Java Stitch", "ROI : "+Arrays.toString(roiData));
        Log.d("Java Stitch", "K_RINV : "+Arrays.toString(k_rinvData));

        //Send ROI to GPU
        Factory.getFactory(null).getGlRenderer().getStitch().setROI(roiData,k_rinvData);
        ///Placeholder

        float[] refinedMatArray = new float[16];
        refine.get(0,0,refinedMatArray);
        float[] refinedQuad = Util.matrixToQuad(refinedMatArray);
//        Factory.mainController.updateQuaternion(refinedQuad,Factory.mainController.mDeltaQuaternion);

        mBitmapArea = areaFloat;
        Factory.getFactory(null).getGlRenderer().getSphere().updateArea(mBitmapArea);



        return 1;
    }

    public void setContext(Context context){

        this.context = context;
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
