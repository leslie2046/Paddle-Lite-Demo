package com.addasound.object_detection;

import android.graphics.Rect;
import android.graphics.RectF;
import android.support.annotation.NonNull;

public class TrackingObject {
    enum DirectionX { StationaryX, Left, Right };
    enum DirectionZ { StationaryZ, Close, Away };
    public int trackId;
    public float score;
    public RectF rect = new RectF();
    public int directionX = 0;  // 存储x轴运动方向
    public int directionZ = 0;  // 存储z轴运动方向
    public int inputW = 0;
    public int inputH = 0;
    public int areaState = 0;//0:功能未启用  1:in   2:out
    public int areaAction = 0;//0:功能未启用 1:not change 2:in to out 3:out to in

    @Override
    public String toString() {
        return "TrackingObject{" +
                "trackId=" + trackId +
                ", score=" + score +
                ", rect=" + rect +
                ", directionX=" + directionX +
                ", directionZ=" + directionZ +
                ", inputW=" + inputW +
                ", inputH=" + inputH +
                ", areaState=" + areaState +
                ", areaAction=" + areaAction +
                '}';
    }
}
