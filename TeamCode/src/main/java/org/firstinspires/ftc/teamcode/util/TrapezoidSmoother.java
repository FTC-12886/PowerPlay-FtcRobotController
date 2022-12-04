package org.firstinspires.ftc.teamcode.util;

public class TrapezoidSmoother {

    private double rate;
    private double max;
    private double min;
    private double steps;
    private double startTime;

    public TrapezoidSmoother(double min, double max, double rate) {
        this.max = max;
        this.min = min;
        this.rate = rate;
        this.steps = (max-min)/rate;
        System.out.println(steps);
        startTime = System.nanoTime();
    }

    public double calculate() {
        if (System.nanoTime()-startTime < steps) {
            return (System.nanoTime()-startTime)*rate+min;
        } else return max;
    }

    public double getTarget() {
        return max;
    }

    public void setTarget(double min, double max) {
        this.min = min;
        this.max = max;
        if (System.nanoTime()-startTime >= steps) // done with rising
            startTime = System.nanoTime(); // reset time as well
    }

}
