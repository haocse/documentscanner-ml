package com.haotran.documentscanner.activity;

public interface FrameWorker extends Runnable {

    public abstract void allocate(int width, int height);

    public abstract boolean put(byte[] data);

    public abstract void release();

}