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
                '}';
    }
}
