package com.example.controlserver.Models;

public class Coordinate {

    private float X;
    private float Y;
    private float theta;

    @Override
    public String toString() {
        return X+";"+Y+";"+";"+theta;
    }

    public float getX() {
        return this.X;
    }

    public float getY() {
        return this.Y;
    }

    public float getTheta() {
        return this.theta;
    }


    public void setX(float X) {
        this.X = X;
    }

    public void setY(float Y) {
        this.Y = Y;
    }

    public void setTheta(float theta) {
        this.theta = theta;
    }
}