package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PidfController {
    private double kP;
    private double tI;
    private double tD;
    private double f;

    private double integral = 0;
    private double sp;
    private int n;
    private long lastTime;
    private double lastPv = 0;
    private double lastError;

    public PidfController(double kP, double tI, double tD, double f) {
        this(kP, tI, tD, f, 0);
    }

    public PidfController(double kP, double tI, double tD, double f, double sp) {
        this.kP = kP;
        this.tI = tI;
        this.tD = tD;
        this.f = f;
        this.sp = sp;
    }
    public double calculate(double pv) {
        long theTime = System.nanoTime();
        if (n == 0) { // first time running
            lastPv = pv;
            lastTime = theTime+1; // avoid div by 0 error
        }
        double error = pv-sp;

        double deltaPv = pv-lastPv;
        double deltaT = theTime-lastTime;
        integral += error*deltaT;
        double output = f + kP*(error + (1/tI)*integral + tD*(deltaPv/deltaT));

        lastTime = theTime;
        lastError = error;
        lastPv = pv;
        n++;
        return output;

    }
    public void setSp(double sp) {
        this.sp = sp;
    }
    public double getError() {
        return lastError;
    }
    public double getSetPoint() {
        return sp;
    }
}
