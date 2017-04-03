package com.kunato.imagestitching;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.location.Location;
import android.media.Image;
import android.media.ImageReader;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.AsyncTask;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.v4.view.VelocityTrackerCompat;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.VelocityTracker;
import android.view.View;
import android.widget.Toast;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import static android.hardware.camera2.CaptureRequest.*;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

import static android.hardware.camera2.CameraCharacteristics.*;

public class MainController extends GLSurfaceView {
    private Activity mActivity;
    static {
        System.loadLibrary("native");
    };


    private static final String TAG = MainController.class.getName();
    private final CameraCaptureSession.CaptureCallback mCaptureCallback = new CameraCaptureSession.CaptureCallback() {
        private void process(CaptureResult capture) {
            //Nexus5x
            mLastSensorISO = capture.get(CaptureResult.SENSOR_SENSITIVITY);
            mLastShutterSpeed = capture.get(CaptureResult.SENSOR_EXPOSURE_TIME);
            if(mAEReport && mLastShutterSpeed < MAX_SHUTTER_SPEED){
                Log.i("CaptureRequest","ISO : "+mLastSensorISO+", ShutterSpeed : "+mLastShutterSpeed);
               mAEReport = false;
            }
        }

        @Override
        public void onCaptureProgressed(CameraCaptureSession session,CaptureRequest request,
                                        CaptureResult partialResult) {
           // process(partialResult);
        }

        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                       TotalCaptureResult result) {
            process(result);
        }

    };


    public static long MAX_SHUTTER_SPEED = 5000000;
    //Nexus5x = 1080,1920
    //Note10.1 = 1080,1440
//    private Size mReadSize = new Size(1080,1440);
//    private Size mReadSize = new Size(1920,1080);
    private Size mReadSize = new Size(2688,1512);
    private Size mPreviewSize = new Size(1920,1080);
    private int mConvertType = Imgproc.COLOR_YUV2BGR_I420;
//    private int mConvertType = Imgproc.COLOR_YUV2BGR_NV12;
    //Using in OnImageAvailableListener
    private int mLastSensorISO;
    private long mLastShutterSpeed;
    private boolean mAEReport = false;
    public byte[] mFrameByte;
    public boolean mAsyncRunning = false;
    public boolean mAlign = false;
    public boolean mRunning = false;
    private boolean mFirstRun = true;
    public float[] mQuaternion = new float[4];
    public float[] mDeltaQuaternion = new float[4];
    private boolean mRecordQuaternion = false;
    public int mNumPicture = 0;

    private float[] lastQuaternion = new float[4];
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener = new ImageReader.OnImageAvailableListener() {

        @Override
        public void onImageAvailable(ImageReader reader) {
            Image image = reader.acquireNextImage();
            if(!mAsyncRunning && mRunning){
                mAsyncRunning = true;
                mRunning = false;
                Log.d("ImageReader","Start!");
                //Note10.1
//                Log.d("ImageReader","length : "+planes.length);
//                Image.Plane Y = image.getPlanes()[0];
//                Image.Plane U = image.getPlanes()[1];
//                Image.Plane V = image.getPlanes()[2];
//                byte[] yBytes = new byte[Y.getBuffer().remaining()];
//                Y.getBuffer().rewind();
//                Y.getBuffer().get(yBytes);
//                byte[] uBytes = new byte[U.getBuffer().remaining()];
//                U.getBuffer().rewind();
//                U.getBuffer().get(uBytes);
//                byte[] vBytes = new byte[V.getBuffer().remaining()];
//                V.getBuffer().rewind();
//                V.getBuffer().get(vBytes);
//
//                int Yb = Y.getBuffer().remaining();
//                int Ub = U.getBuffer().remaining();
//                int Vb = V.getBuffer().remaining();
//                if(mFrameByte == null) {
//                    mFrameByte = new byte[Yb + Ub + Vb];
//                }
//                Y.getBuffer().get(mFrameByte, 0, Yb);
//                U.getBuffer().get(mFrameByte, Yb, Ub);
//                V.getBuffer().get(mFrameByte, Yb+ Ub, Vb);

                //Nexus5x
                mFrameByte = Util.readImage(image);

                doStitching();
            }
            else if(mAlign){
                mAlign = false;
                Log.d("ImageReader","Align Start!");
                //Note10.1
//                Log.d("ImageReader","length : "+planes.length);
//                Image.Plane Y = image.getPlanes()[0];
//                Image.Plane U = image.getPlanes()[1];
//                Image.Plane V = image.getPlanes()[2];
//
//                int Yb = Y.getBuffer().remaining();
//                int Ub = U.getBuffer().remaining();
//                int Vb = V.getBuffer().remaining();
//                if(mFrameByte == null)
//                    mFrameByte = new byte[Yb + Ub + Vb];
//
//                Y.getBuffer().get(mFrameByte, 0, Yb);
//                U.getBuffer().get(mFrameByte, Yb, Ub);
//                V.getBuffer().get(mFrameByte, Yb+ Ub, Vb);

                //Nexus5x
                mFrameByte = Util.readImage(image);
                doAlign();
            }
            image.close();

        }

    };
    public static boolean RESTORE_LOCATION = true;





    public void requireAlign(){
        mAlign = true;
    }



    private void doAlign() {
        AsyncTask<Mat, Integer, Boolean> aligningTask = new ImageAligningTask();
        aligningTask.execute();
    }



    private VelocityTracker mVelocityTracker = null;





    private CameraCaptureSession mCaptureSession;
    private CameraDevice mCameraDevice;
    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            mCameraDevice = cameraDevice;
            createCameraPreviewSession();
        }

        @Override
        public void onDisconnected(CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(CameraDevice cameraDevice, int error) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
            mActivity.onBackPressed();
        }

    };

    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;
    private ImageReader mImageReader;
    private CaptureRequest.Builder mPreviewRequestBuilder;
    private Semaphore mCameraOpenCloseLock = new Semaphore(1);
    private SensorListener mSensorListener;
    private SensorManager mSensorManager;
    private CameraCharacteristics mCharacteristics;
    public GLRenderer mGLRenderer;
    private String mCameraId;
    private float TARGET_ASPECT = 3.f / 4.f;
    private float ASPECT_TOLERANCE = 0.1f;
    private Factory mFactory;
    LocationServices mLocationServices;
    private float mAngleAdjustment = 0.0f;
    public MainController(Context context) {
        super(context);
        mFactory = Factory.getFactory(this);
        mActivity = (Activity) context;
        mGLRenderer = mFactory.getGlRenderer();
        setEGLContextClientVersion(3);
        setEGLConfigChooser(8,8,8,8,16,8);
        setRenderer(mGLRenderer);
        setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        mLocationServices = new LocationServices(this);
        mLocationServices.start();

    }

    public void surfaceCreated ( SurfaceHolder holder ) {
        super.surfaceCreated(holder);
        Resume();
    }

    public void surfaceDestroyed ( SurfaceHolder holder ) {
        mGLRenderer.close();
        super.surfaceDestroyed(holder);
    }

    public float[] mRotmat = new float[16];
    public void writeLocation(Location location,int angle){
        double latitude = location.getLatitude();
        double longitude = location.getLongitude();
        try {
            FileOutputStream fos = new FileOutputStream("/sdcard/stitch/location.bin");

            DataOutputStream dos = new DataOutputStream(fos);

            dos.writeDouble(latitude);
            dos.writeDouble(longitude);
            dos.writeInt(angle);
            dos.flush();
            dos.close();
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    public Object[] tryReadLocation(){
        try {
            File file = new File("/sdcard/stitch/location.bin");
            if(!file.exists()){
                return null;
            }
            FileInputStream fis = new FileInputStream(file);
            DataInputStream dis = new DataInputStream(fis);
            double lat2 = dis.readDouble();
            double lon2 = dis.readDouble();
            int ang2 = dis.readInt();
            fis.close();
            dis.close();
            Object[] obj = new Object[2];
            Location l = new Location("");
            l.setLatitude(lat2);
            l.setLongitude(lon2);
            obj[0] = l;
            obj[1] = ang2;
            return obj;
        } catch (IOException e) {
            return null;
        }
    }
    public void doStitching(){
        SensorManager.getRotationMatrixFromVector(mRotmat,mQuaternion);
        AsyncTask<Mat, Integer, Boolean> imageStitchingTask = new ImageStitchingTask();
        if (mFirstRun) {
            Object[] locationAndRotation = mLocationServices.getLocation();
            Location deviceLocation;
            int angle;
            Object[] readLocation = tryReadLocation();
            if(readLocation == null || !RESTORE_LOCATION) {
                Log.d("MainController", "New Location");
                deviceLocation = (Location) locationAndRotation[0];
                angle = (int) locationAndRotation[1];
                writeLocation(deviceLocation,angle);
            }
            else{
                Log.d("MainController","Used Location");
                deviceLocation = (Location) readLocation[0];
                angle = (int) readLocation[1];
            }
            Log.d("MainController","Use Location : "+deviceLocation.getLatitude()+" : "+deviceLocation.getLongitude()+" Angle : "+angle);

            mGLRenderer.initARObject(angle, deviceLocation, mAngleAdjustment);
            mFirstRun = false;
            mQuaternion[0] = 0f;
            mQuaternion[1] = 0f;
            mQuaternion[2] = 0f;
            mQuaternion[3] = 1f;
            mSensorManager.registerListener(mSensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST);
//            mSensorManager.registerListener(mSensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),SensorManager.SENSOR_DELAY_GAME);
            this.setOnTouchListener(new OnTouchListener() {
                @Override
                public boolean onTouch(View v,MotionEvent event) {
                    int index = event.getActionIndex();
                    int action = event.getActionMasked();
                    int pointerId = event.getPointerId(index);

                    switch(action) {
                        case MotionEvent.ACTION_DOWN:
                            if(mVelocityTracker == null) {
                                mVelocityTracker = VelocityTracker.obtain();
                            }
                            else {
                                mVelocityTracker.clear();
                            }
                            mVelocityTracker.addMovement(event);
                            break;
                        case MotionEvent.ACTION_MOVE:
                            mVelocityTracker.addMovement(event);
                            mVelocityTracker.computeCurrentVelocity(1000);

                            if(VelocityTrackerCompat.getXVelocity(mVelocityTracker,pointerId) * VelocityTrackerCompat.getXVelocity(mVelocityTracker,pointerId)
                                    > VelocityTrackerCompat.getYVelocity(mVelocityTracker,pointerId) * VelocityTrackerCompat.getYVelocity(mVelocityTracker,pointerId)){
                                if(VelocityTrackerCompat.getXVelocity(mVelocityTracker,pointerId)* VelocityTrackerCompat.getXVelocity(mVelocityTracker,pointerId)> 500*500){
                                    mAngleAdjustment += 0.00002f * VelocityTrackerCompat.getXVelocity(mVelocityTracker, pointerId);
                                    mGLRenderer.setAdjustment(mAngleAdjustment);
                                }
                            }
                            else{
                                if(VelocityTrackerCompat.getYVelocity(mVelocityTracker,pointerId)>500){

                                }
                            }
                            break;
                        case MotionEvent.ACTION_UP:
                            // Return a VelocityTracker object back to be re-used by others.
                        case MotionEvent.ACTION_CANCEL:
                            mVelocityTracker.recycle();
                            mVelocityTracker = null;
                            break;
                    }
                    return true;
                }
            });

        }
        Mat rotationCVMat = new Mat();
        rotationCVMat.create(3, 3, CvType.CV_32F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
              rotationCVMat.put(i, j, mRotmat[i * 4 + j]);
//                rotationCVMat.put(i, j, Util.UP180[i * 3 + j]);
            }
        }
        imageStitchingTask.execute(rotationCVMat);
    }

    public void runProcess(boolean firstTime){
        if(firstTime){
            if(mLocationServices == null) {
                mLocationServices = new LocationServices(this);
                mLocationServices.start();
            }

            Log.d("MainController","Button Press, AE Lock");
            mPreviewRequestBuilder.set(CONTROL_AF_TRIGGER,CONTROL_AF_TRIGGER_START);
            //mPreviewRequestBuilder.set(CONTROL_AF_MODE, CONTROL_AF_MODE_AUTO);
            mPreviewRequestBuilder.set(CONTROL_AWB_LOCK, Boolean.TRUE);
            //Nexus5x
            if(mLastShutterSpeed > MAX_SHUTTER_SPEED){
                int i = 2;
                for( ; i < 10 ; i+=2 ){
                    if(mLastShutterSpeed/i < MAX_SHUTTER_SPEED){
                        break;
                    }
                }
                mPreviewRequestBuilder.set(CONTROL_AE_MODE,CONTROL_AE_MODE_OFF);
                mPreviewRequestBuilder.set(SENSOR_SENSITIVITY,mLastSensorISO*i);
                mPreviewRequestBuilder.set(SENSOR_EXPOSURE_TIME,mLastShutterSpeed/i);
                mAEReport = true;
            }
            updatePreview();
        }
        else {
            if(!mAsyncRunning)
                mRunning = true;
            Log.d("MainController","Button Press, Still Running");
         }
    }

    public void Resume() {
        permissionRequest();
        if (mSensorManager == null)
            mSensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);
        if (mSensorListener == null) {
            mSensorListener = new SensorListener();
        }
        startBackgroundThread();
        openCamera();

    }

    public void Pause() {
        Log.e(TAG, "onPause");
        if(mLocationServices!= null)
            mLocationServices.stop();
        if(mSensorManager != null)
            mSensorManager.unregisterListener(mSensorListener);
        closeCamera();
        stopBackgroundThread();
    }

    private void openCamera() {
        Log.d("MainController", "openCamera");
        CameraManager manager = (CameraManager) getActivity().getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
                if (characteristics.get(LENS_FACING) == LENS_FACING_FRONT) continue;
                mCharacteristics = characteristics;
                //Note10.1
//                mImageReader = ImageReader.newInstance(mReadSize.getWidth(), mReadSize.getHeight(), ImageFormat.YV12, 1);
                //Nexus5x
                mImageReader = ImageReader.newInstance(mReadSize.getWidth(), mReadSize.getHeight(), ImageFormat.YUV_420_888,1);

                mImageReader.setOnImageAvailableListener(mOnImageAvailableListener, mBackgroundHandler);

                StreamConfigurationMap configs = characteristics.get(
                        CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                Log.d("CameraParam","Support OutputSizes"+Arrays.toString(configs.getOutputSizes(ImageReader.class)));
                Log.d("CameraParam","CaptureResultKey :"+Arrays.toString(characteristics.getAvailableCaptureRequestKeys().toArray()));

                Log.d("CameraParam","Support : "+configs.isOutputSupportedFor(ImageReader.class)+","+configs.isOutputSupportedFor(mImageReader.getSurface()));
                Log.d("CameraParam","Format : "+Arrays.toString(configs.getOutputFormats()));
                Log.d("CameraParam","Size : "+Arrays.toString(configs.getOutputSizes(ImageFormat.YV12)));

                mCameraId = cameraId;
                break;
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();

        }
        try {
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }
            manager.openCamera(mCameraId, mStateCallback, mBackgroundHandler);

        } catch (Exception e) {
            e.printStackTrace();
            Log.e("CameraCharacteristic",e.getLocalizedMessage());
        }
    }

    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            if (mCaptureSession != null) {
                mCaptureSession.close();
                mCaptureSession = null;
            }
            if (mCameraDevice != null) {
                mCameraDevice.close();
                mCameraDevice = null;
            }
            if (mImageReader != null) {
                mImageReader.close();
                mImageReader = null;
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.", e);
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    private void startBackgroundThread() {
        Log.d("BackgroundThread","START");
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    private void stopBackgroundThread() {
        try {
            mBackgroundThread.quitSafely();
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException | NullPointerException e) {
            e.printStackTrace();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        }
    }

    private void createCameraPreviewSession() {
        try {
            Log.d("CameraCharacteristic", "createCameraPreviewSession");

            SurfaceTexture glProcessTexture = mGLRenderer.getProcessSurfaceTexture();

            if (glProcessTexture == null){
                try {
                    Thread.sleep(1000);
                    Log.w("CameraCharacteristic","GLSurface Connector, Texture not ready yet try again in 1 sec");
                    createCameraPreviewSession();
                    return;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            glProcessTexture.setDefaultBufferSize(mPreviewSize.getWidth(),mPreviewSize.getHeight());
            Surface mGLProcessSurface = new Surface(glProcessTexture);

            List<Surface> surfaceList = new ArrayList<>();
            surfaceList.add(mImageReader.getSurface());
            surfaceList.add(mGLProcessSurface);

            mCameraDevice.createCaptureSession(surfaceList,
                    new CameraCaptureSession.StateCallback() {

                        @Override
                        public void onConfigured(CameraCaptureSession cameraCaptureSession) {
                            if (mCameraDevice == null)
                                return;
                            mCaptureSession = cameraCaptureSession;
                            //Note10.1
//                            mPreviewRequestBuilder.set(CONTROL_AF_MODE, CONTROL_AF_MODE_CONTINUOUS_PICTURE);
//                            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER, 1);
                            //Nexus5x
                            Range<Integer>[] ranges = mCharacteristics.get(CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
                            long shutterS = 7666666;
                            mPreviewRequestBuilder.set(SENSOR_EXPOSURE_TIME, shutterS);
                            mPreviewRequestBuilder.set(CONTROL_AE_MODE, CONTROL_AE_MODE_ON);
                            mPreviewRequestBuilder.set(CONTROL_AWB_MODE, CONTROL_AWB_MODE_AUTO);
                            mPreviewRequestBuilder.set(CONTROL_AF_MODE, CONTROL_AF_MODE_AUTO);
                            mPreviewRequestBuilder.set(SENSOR_SENSITIVITY, 200);

                            mPreviewRequestBuilder.set(CONTROL_AE_TARGET_FPS_RANGE,Range.create(30,30));

                            Log.d("Camera","On Configured");
//                            mPreviewRequestBuilder.set(CONTROL_AE_MODE, CONTROL_AE_MODE_OFF);
//                            mPreviewRequestBuilder.set(CaptureRequest.SENSOR_SENSITIVITY, 800);
                            updatePreview();
                        }

                        @Override
                        public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {
                            Toast.makeText(getActivity(), "Failed", Toast.LENGTH_SHORT).show();
                        }
                    }, mBackgroundHandler);
            mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_ZERO_SHUTTER_LAG);
            for(int i = 0 ; i < surfaceList.size() ;i++){
                mPreviewRequestBuilder.addTarget(surfaceList.get(i));
            }

        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private void updatePreview(){
        try {

            mCaptureSession.stopRepeating();
            mCaptureSession.setRepeatingRequest(mPreviewRequestBuilder.build(), mCaptureCallback, mBackgroundHandler);
        } catch (Exception e) {
            e.printStackTrace();
            Log.i("GLSurface Connector", "UpdatePreview, ExceptionExceptionException");
        }
    }

    public void permissionRequest() {
//        if (getActivity().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
//                getActivity().checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
//                getActivity().checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
//                getActivity().checkSelfPermission(Manifest.permission.READ_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
//                getActivity().checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
//            getActivity().requestPermissions(new String[]{Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE,
//                            Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION},
//                    1);
//        }
    }
    public void startRecordQuaternion(){
        mDeltaQuaternion[0] = 0f;
        mDeltaQuaternion[1] = 0f;
        mDeltaQuaternion[2] = 0f;
        mDeltaQuaternion[3] = 1f;
        mRecordQuaternion = true;
    }

    public void updateQuaternion(float[] mainQuaternion ,float[] deltaQuaternion){
        lastQuaternion = mQuaternion.clone();
        mQuaternion = Util.multiplyByQuat(deltaQuaternion, mainQuaternion);
        Log.d("Controller","After Align Quad"+Arrays.toString(mQuaternion));
    }

    public Activity getActivity(){
        return mActivity;
    }

    /**
     * private Class SensorListener + Async
     */
    private class SensorListener implements SensorEventListener {
        private float lastTimeStamp = 0f;

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                mQuaternion = Util.getQuadFromGyro(event.values,lastTimeStamp,event.timestamp, mQuaternion,false,true,false,true);
                if(mRecordQuaternion){
                    mDeltaQuaternion = Util.getQuadFromGyro(event.values,lastTimeStamp,event.timestamp,mDeltaQuaternion,false,true,false,true);
                }
                lastTimeStamp = event.timestamp;
                float[] swapMat = new float[16];
                SensorManager.getRotationMatrixFromVector(swapMat, mQuaternion);
                float[] correctedRotMat = new float[16];
                float[] rotMat = new float[9];
                float[] correctedQuat = {mQuaternion[0],-mQuaternion[1], mQuaternion[2], mQuaternion[3]};
                float[] temp = new float[16];
                SensorManager.getRotationMatrixFromVector(correctedRotMat, correctedQuat);
                SensorManager.getRotationMatrixFromVector(rotMat,mQuaternion);
                Matrix.multiplyMM(temp,0,Util.ROTATE_Y_270,0,swapMat,0);
//                Matrix.multiplyMM(temp, 0, Util.SWAP_X, 0, rotMat, 0);
//                Matrix.multiplyMM(rotMat, 0, Util.SWAP_Z, 0, temp, 0);
                mGLRenderer.setRotationMatrix(correctedRotMat);
                if(!mAsyncRunning)
                    if(!mRunning)
                        if(ImageStitchingNative.getNativeInstance().keyFrameSelection(rotMat) == 1)
                            mRunning = true;
            }
            if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR){
                Log.i("SensorListener","RotationVector"+Arrays.toString(event.values));
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    }

    private class ImageStitchingTask extends AsyncTask<Mat, Integer, Boolean> {
        protected Boolean doInBackground(Mat... objects) {
            Log.d("AsyncTask","doInBackground");
            Mat yv12 = new Mat(mReadSize.getHeight()*3/2, mReadSize.getWidth(), CvType.CV_8UC1);
            yv12.put(0, 0, mFrameByte);
            Mat rgb = new Mat(mReadSize.getWidth(), mReadSize.getHeight(),CvType.CV_8UC3);
            Imgproc.cvtColor(yv12, rgb, mConvertType,3);
            Thread uiThread = new Thread() {

                @Override
                public void run() {
                    getActivity().runOnUiThread(new Runnable() {

                        @Override
                        public void run() {
                            ((MainActivity)getActivity()).getButton().setBackgroundColor(Color.RED);
                        }
                    });
                    super.run();
                }
            };
            uiThread.start();
            int rtCode = ImageStitchingNative.getNativeInstance().addToPano(rgb, (Mat) objects[0] ,mNumPicture);
            if(rtCode == 1){
                mNumPicture++;
            }
            return true;
        }

        protected void onProgressUpdate(Integer... progress) {

        }

        protected void onPostExecute(Boolean bool) {
            Thread uiThread = new Thread() {

                @Override
                public void run() {
                    getActivity().runOnUiThread(new Runnable() {

                        @Override
                        public void run() {
                            ((MainActivity)getActivity()).getButton().setBackgroundColor(Color.GRAY);
                            ((MainActivity)getActivity()).getButton().setText("Capture : " + mNumPicture);
                        }
                    });
                    super.run();
                }
            };
            uiThread.start();
            mAsyncRunning = false;
            Log.i("GLSurface Connector","Stitch Complete, "+mNumPicture+"");

        }
    }



    private class ImageAligningTask extends AsyncTask<Mat, Integer, Boolean> {
        protected Boolean doInBackground(Mat... objects) {
            Log.d("AsyncTask","doInBackground");
            Mat yv12 = new Mat(mReadSize.getHeight()*3/2, mReadSize.getWidth(), CvType.CV_8UC1);
            yv12.put(0, 0, mFrameByte);
            Mat rgb = new Mat(mReadSize.getWidth(), mReadSize.getHeight(),CvType.CV_8UC3);
            Imgproc.cvtColor(yv12, rgb, mConvertType,3);
            float[] rotMat = new float[16];
            SensorManager.getRotationMatrixFromVector(rotMat, mQuaternion);
            ImageStitchingNative.getNativeInstance().aligning(rgb,rotMat);
            return true;
        }

        protected void onProgressUpdate(Integer... progress) {

        }

        protected void onPostExecute(Boolean bool) {

            Log.i("GLSurface Connector","Algin Complete.");

        }
    }


}