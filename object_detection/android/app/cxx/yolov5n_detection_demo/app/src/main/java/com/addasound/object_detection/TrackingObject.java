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
    public int areaState = 0;//0:功能未启用  1:in   2:out
    public int areaAction = 0;//0:功能未启用 1:not change 2:in to out 3:out to in

    public int lineState = 0;//0:功能未启用 1:no change 2:跨越了out  3:跨越了in
    public int lineAction = 0; //0:功能未启用 1:not change 2:in to out 3:out to in

    @Override
    public String toString() {
        return "TrackingObject{" +
                "trackId=" + trackId +
                ", score=" + score +
                ", rect=" + rect +
                ", inputW=" + inputW +
                ", inputH=" + inputH +
                ", areaState=" + areaState +
                ", areaAction=" + areaAction +
                ", lineState=" + lineState +
                ", lineAction=" + lineAction +
                '}';
    }
}
