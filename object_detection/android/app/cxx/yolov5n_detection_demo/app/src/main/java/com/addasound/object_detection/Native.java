package com.addasound.object_detection;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.PointF;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

public class Native {
    static {
        System.loadLibrary("Native");
    }

    private long ctx = 0;

    public boolean init(String modelDir,
                        String labelPath,
                        int cpuThreadNum,
                        String cpuPowerMode,
                        int inputWidth,
                        int inputHeight,
                        float[] inputMean,
                        float[] inputStd,
                        float scoreThreshold) {
        ctx = nativeInit(
                modelDir,
                labelPath,
                cpuThreadNum,
                cpuPowerMode,
                inputWidth,
                inputHeight,
                inputMean,
                inputStd,
                scoreThreshold);
        nativeSetTrackingClassId(ctx,0);

        return ctx == 0;
    }

    public   boolean setDynamicArea(ArrayList<PointF> area){
        return nativeSetDynamicArea(ctx,area);
    }
    public boolean release() {
        if (ctx == 0) {
            return false;
        }
        return nativeRelease(ctx);
    }

    public boolean process(Bitmap ARGB8888ImageBitmap, String savedImagePath,ArrayList<TrackingObject> outputs) {
        if (ctx == 0) {
            return false;
        }
        // ARGB8888 bitmap is only supported in native, other color formats can be added by yourself.
        return nativeProcess(ctx, ARGB8888ImageBitmap, savedImagePath,outputs);
    }

    public static native long nativeInit(String modelDir,
                                         String labelPath,
                                         int cpuThreadNum,
                                         String cpuPowerMode,
                                         int inputWidth,
                                         int inputHeight,
                                         float[] inputMean,
                                         float[] inputStd,
                                         float scoreThreshold);

    public static native boolean nativeRelease(long ctx);

    public static native boolean nativeProcess(long ctx, Bitmap ARGB888ImageBitmap, String savedImagePath, ArrayList<TrackingObject> outputs);

    public static native boolean nativeSetTrackingClassId(long ctx,int classId);

    public static native boolean nativeSetDynamicArea(long ctx,ArrayList<PointF> area);
}
