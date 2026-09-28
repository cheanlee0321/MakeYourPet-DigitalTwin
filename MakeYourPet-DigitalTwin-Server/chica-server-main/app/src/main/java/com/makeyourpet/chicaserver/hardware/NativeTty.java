package com.makeyourpet.chicaserver.hardware;

final class NativeTty {
    static {
        System.loadLibrary("chica_gait");
    }

    private NativeTty() {
    }

    static native int nativeOpenRaw(String path);
    static native int nativeRead(int fd, byte[] buffer, int offset, int length);
    static native int nativeWrite(int fd, byte[] buffer, int offset, int length);
    static native void nativeClose(int fd);
}
