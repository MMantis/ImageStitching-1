package com.kunato.imagestitching;

import android.renderscript.RenderScript;
import android.util.Size;

/**
 * Created by kunato on 2/29/16.
 */
public class Factory {
    GLRenderer mGlRenderer;
    ImageStitchingNative mImageStitchingNative;
    RSProcessor mRSProcessor;
    static Factory mFactory;
    static MainController mainController;
    private Factory(MainController mc){
        mainController = mc;
    }
    public static Factory getFactory(MainController mainController){
        if(mFactory == null){
            mFactory = new Factory(mainController);
        }
        return mFactory;
    }
    public GLRenderer getGlRenderer() {
        if(mGlRenderer == null)
            mGlRenderer = new GLRenderer(mainController);
        return mGlRenderer;
    }
    public ImageStitchingNative getStitchingNative(){
        if(mImageStitchingNative == null){
            mImageStitchingNative = ImageStitchingNative.getNativeInstance();
        }
        return mImageStitchingNative;
    }

}
