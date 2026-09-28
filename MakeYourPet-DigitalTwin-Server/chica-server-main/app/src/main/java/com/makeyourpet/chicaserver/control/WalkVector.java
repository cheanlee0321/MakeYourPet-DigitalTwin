package com.makeyourpet.chicaserver.control;

public final class WalkVector {
    public final double forward;
    public final double strafe;
    public final double turn;

    public WalkVector(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public boolean active() {
        return Math.abs(forward) > 0.0001d || Math.abs(strafe) > 0.0001d || Math.abs(turn) > 0.0001d;
    }
}
