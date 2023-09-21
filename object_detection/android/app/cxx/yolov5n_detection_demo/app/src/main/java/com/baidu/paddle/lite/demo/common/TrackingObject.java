package com.baidu.paddle.lite.demo.common;

import android.graphics.Rect;
import android.graphics.RectF;
import android.support.annotation.NonNull;

public class TrackingObject {
    public int trackId;
    public float score;
    public RectF rect = new RectF();

    @Override
    public String toString() {
        return "TrackingObject{" +
                "trackId=" + trackId +
                ", score=" + score +
                ", rect=" + rect +
                '}';
    }
}
