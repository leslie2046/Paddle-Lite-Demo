package com.addasound.object_detection;

import android.graphics.Rect;
import android.graphics.RectF;
import android.support.annotation.NonNull;

public class TrackingObject {
    public int trackId;
    public float score;
    public RectF rect = new RectF();
    public int inputW = 0;
    public int inputH = 0;
    int areaAction = -1;//-1:功能未启用 0:not change 1:in to out 2:out to in
    int lineAction = -1; //-1:功能未启用 0:not change 1:in to out 2:out to in
    int welcomeLineAction = -1; //-1:功能未启用 0:not change 1:in to out 2:out to in
    int byeLineAction = -1; //-1:功能未启用 0:not change 1:in to out 2:out to in

    @Override
    public String toString() {
        return "TrackingObject{" +
                "trackId=" + trackId +
                ", score=" + score +
                ", rect=" + rect +
                ", inputW=" + inputW +
                ", inputH=" + inputH +
                ", areaAction=" + areaAction +
                ", lineAction=" + lineAction +
                ", welcomeLineAction=" + welcomeLineAction +
                ", byeLineAction=" + byeLineAction +
                '}';
    }
}
