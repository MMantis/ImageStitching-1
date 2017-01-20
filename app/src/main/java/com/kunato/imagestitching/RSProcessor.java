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

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.ScriptIntrinsicYuvToRGB;
import android.renderscript.Type;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/**
 * Renderscript-based for yuv -> rgb convert
 */
public class RSProcessor {

    private final Bitmap bitmap;
    private Allocation mInputAllocation;
    private Allocation mOutputAllocation;
    private Size mDimension;
    private HandlerThread mProcessingThread;
    private Handler mProcessingHandler;
    private ScriptC_processing mergeScript;
    public ProcessingTask mTask;
    private MainController mController;
    private ScriptIntrinsicYuvToRGB intrinsic;
    private byte[] readbytes;
    public RSProcessor(RenderScript rs, Size dimensions, MainController controller, Surface surface) {
        mController = controller;
        mDimension = dimensions;
        Type.Builder yuvTypeBuilder = new Type.Builder(rs, Element.YUV(rs));
        yuvTypeBuilder.setX(dimensions.getWidth());
        yuvTypeBuilder.setY(dimensions.getHeight());
        yuvTypeBuilder.setYuvFormat(ImageFormat.YUV_420_888);
        mInputAllocation = Allocation.createTyped(rs, yuvTypeBuilder.create(),
                Allocation.USAGE_IO_INPUT | Allocation.USAGE_SCRIPT);
        Type.Builder rgbTypeBuilder = new Type.Builder(rs, Element.RGBA_8888(rs));
        rgbTypeBuilder.setX(dimensions.getWidth());
        rgbTypeBuilder.setY(dimensions.getHeight());
        mOutputAllocation = Allocation.createTyped(rs, rgbTypeBuilder.create(),
                Allocation.USAGE_IO_OUTPUT | Allocation.USAGE_SCRIPT);
        mOutputAllocation.setSurface(surface);
        bitmap = Bitmap.createBitmap(dimensions.getWidth(), dimensions.getHeight(), Bitmap.Config.ARGB_8888);
//        mOutputAllocation = Allocation.createTyped(rs,mInputAllocation.getType());
        mProcessingThread = new HandlerThread("ViewfinderProcessor");
        mProcessingThread.start();
        mProcessingHandler = new Handler(mProcessingThread.getLooper());
        intrinsic = ScriptIntrinsicYuvToRGB.create(rs,Element.U8_4(rs));
//        intrinsic.setInput(mInputAllocation);
//        intrinsic.forEach(mOutputAllocation);
//        mOutputAllocation.copyTo(bitmap);
//        mergeScript = new ScriptC_processing(rs);
        mInputAllocation.setOnBufferAvailableListener(new BufferListener());
//        mTask = new ProcessingTask(mInputAllocation);
        Log.d("RS","RS Processor init");

    }
    public Surface getInputSurface() {
        return mInputAllocation.getSurface();
    }
    public void setOutputSurface(Surface surface){
        mOutputAllocation.setSurface(surface);
    }
    class OutBufferListener implements  Allocation.OnBufferAvailableListener{

        private boolean write = true;

        @Override
        public void onBufferAvailable(Allocation allocation) {

            //WORK
            if(write) {

                Log.d("RS", "Write Mat");

                write = false;
            }
            mController.requestStitch();
        }

    }
    class BufferListener implements  Allocation.OnBufferAvailableListener {

        @Override
        public void onBufferAvailable(Allocation allocation) {
            Log.d("RS", "Received Buffer");
            mInputAllocation.ioReceive();
            intrinsic.setInput(mInputAllocation);
            intrinsic.forEach(mOutputAllocation);
            mOutputAllocation.copyTo(bitmap);
            mOutputAllocation.ioSend();
            Log.d("RS","Output");
            if(mController.mFrame == null)
                mController.mFrame = new Mat(mDimension.getWidth(), mDimension.getHeight() ,CvType.CV_8UC4);
            Utils.bitmapToMat(bitmap,mController.mFrame);
//            Imgcodecs.imwrite("/sdcard/stitch/rs.jpeg", mat);
            if (mController.mFrameByte == null){
                mController.mFrameByte = new byte[mDimension.getWidth() * mDimension.getHeight() * ImageFormat.getBitsPerPixel(ImageFormat.FLEX_RGBA_8888) / 8];
             }
            mController.requestStitch();
        }
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

            Log.d("RS", "processing task init");
        }

        @Override
        public void onBufferAvailable(Allocation a) {

            synchronized (this) {
                mPendingFrames++;
                mProcessingHandler.post(this);
            }
        }

        @Override
        public void run() {
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
            mergeScript.set_gCurrentFrame(mInputAllocation);

            // Run processing pass
            mergeScript.forEach_convertFrames(mOutputAllocation);
            mOutputAllocation.ioSend();

            //WORK
//            if (write && mFrameByte != null) {
//                Log.d("RS","Write Mat");
//                Mat mat = new Mat(1080, 1440, CvType.CV_8UC4);
//                mat.put(0, 0, mFrameByte);
//                Highgui.imwrite("/sdcard/rs.jpeg", mat);
//
//            }
        }


    }

}
