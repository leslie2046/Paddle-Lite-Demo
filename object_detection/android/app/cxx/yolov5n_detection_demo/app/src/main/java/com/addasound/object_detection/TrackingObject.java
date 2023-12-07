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
    int areaState = -1;//-1:功能未启用 0:reset 1:in   2:out
    int areaAction = -1;//-1:功能未启用 0:not change 1:in to out 2:out to in
    int lineState = -1;//-1:功能未启用 0:reset 1:跨越了out  2:跨越了in
    int lineAction = -1; //-1:功能未启用 0:not change 1:in to out 2:out to in

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
