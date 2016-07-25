/*
 * Copyright (C) 2014 The Android Open Source Project
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

import android.graphics.ImageFormat;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.Type;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Renderscript-based for yuv -> rgb convert
 */
public class RSProcessor {

    private Allocation mInputAllocation;

    private HandlerThread mProcessingThread;
    private Handler mProcessingHandler;
    public ProcessingTask mTask;
    private Size mSize;
    private MainController mController;
    private boolean alignRequest = false;
    public RSProcessor(RenderScript rs, Size dimensions, MainController controller) {
        mSize = dimensions;
        mController = controller;
        Type.Builder yuvTypeBuilder = new Type.Builder(rs, Element.YUV(rs));
        yuvTypeBuilder.setX(dimensions.getWidth());
        yuvTypeBuilder.setY(dimensions.getHeight());
        yuvTypeBuilder.setYuvFormat(ImageFormat.YV12);
        mInputAllocation = Allocation.createTyped(rs, yuvTypeBuilder.create(),
                Allocation.USAGE_IO_INPUT);
        mProcessingThread = new HandlerThread("RSProcessor");
        mProcessingThread.start();
        mProcessingHandler = new Handler(mProcessingThread.getLooper());
        mTask = new ProcessingTask(mInputAllocation);
        Log.d("RS","RS Processor init");

    }
    public void requestAligning(){
        alignRequest = true;
    }
    public Surface getInputHdrSurface() {
        return mInputAllocation.getSurface();
    }


    /**
     * Simple class to keep track of incoming frame count,
     * and to process the newest one in the processing thread
     */
    class ProcessingTask implements Runnable, Allocation.OnBufferAvailableListener {
        private int mPendingFrames = 0;

        private Allocation mInputAllocation;

        public ProcessingTask(Allocation input) {
            mInputAllocation = input;
            mInputAllocation.setOnBufferAvailableListener(this);
            Log.d("RS","processing task init");
        }

        @Override
        public void onBufferAvailable(Allocation a) {
                mPendingFrames++;
                mProcessingHandler.post(this);
                Log.d("RS","BufferAvailable");

        }

        @Override
        public void run() {
            Log.d("RS","Running");
            // Find out how many frames have arrived
            int pendingFrames;
            synchronized (this) {
                pendingFrames = mPendingFrames;
                mPendingFrames = 0;

                // Discard extra messages in case processing is slower than frame rate
                mProcessingHandler.removeCallbacks(this);
            }

            // Get to newest input
            for (int i = 0; i < pendingFrames; i++) {
                mInputAllocation.ioReceive();
            }

            if(alignRequest){
                alignAction();
                alignRequest = false;
            }


//            if(!mController.mAsyncRunning && mController.mRunning){
//                mController.mAsyncRunning = true;
//                mController.mRunning = false;
//                Log.d("RS", "Running");
//                mInputAllocation.copyTo(mController.mFrameByte);
//                mController.doStitching();
//            }


            //WORK
//            if (write && mFrameByte != null) {
//                Log.d("RS","Write Mat");
//                Mat mat = new Mat(1080, 1440, CvType.CV_8UC4);
//                mat.put(0, 0, mFrameByte);
//                Highgui.imwrite("/sdcard/rs.jpeg", mat);
//
//            }
        }

        private void alignAction() {
//            Log.d("RSProcessing",mSize.getWidth()+","+mSize.getHeight());
            //Swap width and height because of camera array.
            Mat mat = new Mat(mSize.getHeight(), mSize.getWidth(), CvType.CV_8UC4);
            byte[] frameByte = new byte[mSize.getWidth()*mSize.getHeight()*4];
            mInputAllocation.copyTo(frameByte);
            mat.put(0, 0, frameByte);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);
            float[] rotMat = new float[16];
            SensorManager.getRotationMatrixFromVector(rotMat, mController.mQuaternion);
            //ImageStitchingNative.getNativeInstance().aligning(mat, rotMat, mController.mGLRenderer.mProjectionMatrix);
//            alignRequest = false;
        }
    }

}
