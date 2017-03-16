package com.kunato.imagestitching;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.IntentSender;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.util.Log;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.PendingResult;
import com.google.android.gms.common.api.ResultCallback;
import com.google.android.gms.common.api.Status;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationSettingsRequest;
import com.google.android.gms.location.LocationSettingsResult;
import com.google.android.gms.location.LocationSettingsStates;
import com.google.android.gms.location.LocationSettingsStatusCodes;

import java.text.DateFormat;
import java.util.Arrays;
import java.util.Date;

/**
 * Created by kunato on 4/7/16.
 */
public class LocationServices {
    int mCurrentAngle = 0;
    private SensorEventListener mCompassListener = new SensorEventListener() {
        static final float RAD_2_DEGREE = (float) (180.0f / Math.PI);
        float[] mGravity;
        float[] mGeomagnetic;

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                mGravity = Util.lowPass(event.values,mGravity);
            }
            if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                mGeomagnetic = Util.lowPass(event.values,mGeomagnetic);
            }
            if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
//                SensorManager.getRotationMatrixFromVector(mCameraRotation,event.values);
            }
            if(event.sensor.getType() == Sensor.TYPE_ORIENTATION){
                mCurrentAngle = (int) event.values[0];
            }
            if (mGravity == null || mGeomagnetic == null)
                return;
            if(mLastLocation!= null)
                ((MainActivity)mMainController.getActivity()).getTextView().setText(mCurrentAngle+" ["+mLastLocation.getAccuracy()+"]");
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            if(accuracy == SensorManager.SENSOR_STATUS_ACCURACY_LOW){
                Log.i("LocationServices",sensor.getName()+" : Low Accuracy");

            }
            else if(accuracy == SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM){

                Log.i("LocationServices",sensor.getName()+" : Medium Accuracy");
            }
            else{

                Log.i("LocationServices",sensor.getName()+" : High Accuracy");
            }
        }
    };
    private LocationCallback mLocationCallback;
    private SensorManager mSensorManager;
    private GoogleApiClient mGoogleApiClient;
    private MainController mMainController;
    private Location mLastLocation = null;
    PendingResult<LocationSettingsResult> result;
    LocationRequest mLocationRequest;
    boolean mConnected = false;
    private String mLastUpdateTime;

    protected void createLocationRequest() {
        mLocationRequest = new LocationRequest();
        mLocationRequest.setInterval(10000);
        mLocationRequest.setFastestInterval(5000);
        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
        LocationSettingsRequest.Builder builder = new LocationSettingsRequest.Builder()
                .addLocationRequest(mLocationRequest);
        result = com.google.android.gms.location.LocationServices.SettingsApi.checkLocationSettings(mGoogleApiClient,
                builder.build());
        result.setResultCallback(new ResultCallback<LocationSettingsResult>() {
            @Override
            public void onResult(LocationSettingsResult locationSettingsResult) {
                final Status status = locationSettingsResult.getStatus();
                final LocationSettingsStates locationSetting = locationSettingsResult.getLocationSettingsStates();
                switch (status.getStatusCode()) {
                    case LocationSettingsStatusCodes.SUCCESS:
                        Log.d("LocationServices", "SUCCESS");
                        break;
                    case LocationSettingsStatusCodes.RESOLUTION_REQUIRED:
                        Log.d("LocationServices", "RESOLUTION_REQUIRED");
                        break;
                    case LocationSettingsStatusCodes.SETTINGS_CHANGE_UNAVAILABLE:
                        Log.d("LocationServices", "SETTINGS_CHANGE_UNAVAILABLE");
                        break;
                }

            }
        });
    }

    protected void buildGoogleApiClient() {
        mGoogleApiClient = new GoogleApiClient.Builder(mMainController.getActivity())
                .addConnectionCallbacks(mLocationCallback)
                .addOnConnectionFailedListener(mLocationCallback)
                .addApi(com.google.android.gms.location.LocationServices.API)
                .build();
    }

    protected void initSensor() {
        mSensorManager = (SensorManager) mMainController.getActivity().getSystemService(Context.SENSOR_SERVICE);
        mSensorManager.registerListener(mCompassListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mCompassListener, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mCompassListener,mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),SensorManager.SENSOR_DELAY_GAME);
    }

    public LocationServices(MainController controller) {
        mMainController = controller;
        mLocationCallback = new LocationCallback();
        buildGoogleApiClient();
        createLocationRequest();
        initSensor();
        Log.d("LocationServices","LocationServices Started");

    }

    public void start() {
        mGoogleApiClient.connect();
    }

    public void stopLocationUpdate() {
        com.google.android.gms.location.LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, mLocationCallback);
        Log.d("LocationServices","Location Update Stopped");
    }

    public void stop() {
        mSensorManager.unregisterListener(mCompassListener);
        stopLocationUpdate();
        if (mGoogleApiClient.isConnected()) {
            mGoogleApiClient.disconnect();
        }
    }

    public Object[] getLocation() {
//        if (mMainController.getActivity().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && mMainController.getActivity().checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
//            mMainController.permissionRequest();
//        }
        if (mConnected) {
//            mLastLocation = com.google.android.gms.location.LocationServices.FusedLocationApi.getLastLocation(
//                    mGoogleApiClient);
            stopLocationUpdate();
            Object[] returnObject = {mLastLocation, mCurrentAngle};
            mSensorManager.unregisterListener(mCompassListener);
            return returnObject;
        }
        return null;
    }

    class LocationCallback implements GoogleApiClient.ConnectionCallbacks, GoogleApiClient.OnConnectionFailedListener, com.google.android.gms.location.LocationListener {

        @Override
        public void onConnected(Bundle bundle) {
            Log.i("LocationServices", "Connected");
            mConnected = true;
//            if (mMainController.getActivity().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && mMainController.getActivity().checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
//                mMainController.permissionRequest();
//            }
            com.google.android.gms.location.LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, mLocationCallback);

        }

        @Override
        public void onConnectionSuspended(int i) {
            Log.i("LocationServices", "Connection suspended");
            mGoogleApiClient.connect();
        }

        @Override
        public void onConnectionFailed(ConnectionResult connectionResult) {
            Log.i("LocationServices", "Connection failed: ConnectionResult.getErrorCode() = " + connectionResult.getErrorCode());
        }

        @Override
        public void onLocationChanged(Location location) {
            mLastLocation = location;
            mLastUpdateTime = DateFormat.getTimeInstance().format(new Date());
            Log.d("LocationServices","LocationUpdate : ("+ location.getLatitude()+","+location.getLongitude()+") On : "+mLastUpdateTime);

        }
    }

}
