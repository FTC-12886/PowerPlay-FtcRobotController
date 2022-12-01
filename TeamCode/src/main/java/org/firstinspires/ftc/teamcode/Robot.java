package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.CachingImu;
import org.firstinspires.ftc.teamcode.util.PidfController;
import org.firstinspires.ftc.teamcode.util.Point;

import static org.firstinspires.ftc.teamcode.RobotSpecifications.*;

import java.util.List;

public class Robot {
    private final HardwareMap hardwareMap;
    private DcMotorEx rearRight;
    private DcMotorEx rearLeft;
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx armLift;
    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private CachingImu imu;

    private PidfController turnController;
    private AngleUnit turnControllerAngleUnit;
    private double previousArmAngle = 0;
    private double previousArmAngleTime = 0;

    private Point targetPosition;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

//        armLift = hardwareMap.get(DcMotorEx.class, "armLift");
        driveMotors[0] = rearRight = hardwareMap.get(DcMotorEx.class, "backRight");
        driveMotors[1] = rearLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveMotors[2] = frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        driveMotors[3] = frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        imu = new CachingImu(hardwareMap.get(IMU.class, "imu"));

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

//        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu.initialize(new IMU.Parameters(RobotSpecifications.revHubOrientationOnRobot));
    }

    public void drive(double velocityPerSec, DistanceUnit distanceUnit) { // forwards and backwards
        drive(velocityPerSec, distanceUnit, 90, AngleUnit.DEGREES);
    }

    public void drive(double velocityPerSec, DistanceUnit distanceUnit, double angle, AngleUnit angleUnit) { // translational
        drive(velocityPerSec, distanceUnit, angle, angleUnit, 0, AngleUnit.RADIANS);
    }

    public void spin(double angularVelocityPerSec, AngleUnit angleUnit) { // spinning!
        drive(0, DistanceUnit.MM, 0, AngleUnit.RADIANS, angularVelocityPerSec, angleUnit);
    }

    /**
     * Drive with specified velocity at specified angle. Turn at specified angular velocity per sec
     * Math will definitely be wrong when driving/spinning
     * @param velocityPerSec Drive velocity in units/sec
     * @param distanceUnit Distance unit for the drive velocity
     * @param angle Angle to drive at
     * @param driveAngleUnit Angle unit for angle to drive at
     * @param angularVelocityPerSec Angular velocity in units/sec
     * @param turnAngleUnit Angle unit for angular velocity
     */
    public void drive(double velocityPerSec, DistanceUnit distanceUnit, double angle, AngleUnit driveAngleUnit, double angularVelocityPerSec, AngleUnit turnAngleUnit) { // translational and rotational
        double velocity = distanceUnit.toMm(velocityPerSec); // mm/s
        double xVelocity = velocity*Math.cos(driveAngleUnit.toRadians(angle)); // mm/s
        System.out.println("angle "+Math.toDegrees(angle)+"\txvelo "+xVelocity);

        double yVelocity = velocity*Math.sin(driveAngleUnit.toRadians(angle)); // mm/s
        double turnVelocity = turnAngleUnit.toRadians(angularVelocityPerSec); // rad/s
        // move with arm stuff
//        double armAngle = getArmAngle(AngleUnit.RADIANS);
//        long theTime = System.nanoTime();
//        yVelocity -= -Math.sin(armAngle)*(armAngle-previousArmAngle)/(theTime-previousArmAngleTime);

        double turnLinearVelocity = 2*turnVelocity*RobotSpecifications.driveBaseRadius; // mm/s
        double rearRightVelocity = 0;
        double rearLeftVelocity = 0;
        double frontRightVelocity = 0;
        double frontLeftVelocity = 0;

        // no clue if this is right or whether it should be 1/2 or 2 or sqrt(2)/2 or sqrt(2) or something
        double multiplyThing = Math.sqrt(2); //MAYBE???????? TODO Figure out the answer

        yVelocity = Double.isNaN(yVelocity) ? 0: yVelocity;
        rearRightVelocity += rearRightParameters.yPower()*yVelocity;
        rearLeftVelocity += rearLeftParameters.yPower()*yVelocity;
        frontRightVelocity += frontRightParameters.yPower()*yVelocity;
        frontLeftVelocity += frontLeftParameters.yPower()*yVelocity;

        xVelocity = Double.isNaN(xVelocity) ? 0: xVelocity;
        rearRightVelocity += rearRightParameters.xPower()*xVelocity;
        rearLeftVelocity += rearLeftParameters.xPower()*xVelocity;
        frontRightVelocity += frontRightParameters.xPower()*xVelocity;
        frontLeftVelocity += frontLeftParameters.xPower()*xVelocity;

        // TODO how to do?
        rearRightVelocity -= rearRightParameters.yPower()*turnLinearVelocity;
        rearLeftVelocity += rearLeftParameters.yPower()*turnLinearVelocity;
        frontRightVelocity -= frontRightParameters.yPower()*turnLinearVelocity;
        frontLeftVelocity += frontLeftParameters.yPower()*turnLinearVelocity;

        rearRight.setVelocity(rearRightVelocity*RobotSpecifications.driveWheelCountsPerMm);
        rearLeft.setVelocity(rearLeftVelocity*RobotSpecifications.driveWheelCountsPerMm);
        frontRight.setVelocity(frontRightVelocity*RobotSpecifications.driveWheelCountsPerMm);
        frontLeft.setVelocity(frontLeftVelocity*RobotSpecifications.driveWheelCountsPerMm);

//        previousArmAngle = armAngle;
//        previousArmAngleTime = theTime;
    }

    public void stop() {
        drive(0, DistanceUnit.MM, 0, AngleUnit.RADIANS, 0, AngleUnit.RADIANS);
    }



    /** Set turn target for turn with IMU method, using yaw angle. Call the pidTurn() method to turn
     *
     * @param targetAngle Target angle (absolute, not relative)
     * @param angleUnit Angle unit for target angle
     */
    public void setPidTurnTarget(double targetAngle, AngleUnit angleUnit, double kP, double tI, double tD) {
        turnController = new PidfController(kP, tI, tD, targetAngle);
        turnControllerAngleUnit = angleUnit;
    }

    public double pidTurn() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double output = turnController.calculate(angles.getYaw(turnControllerAngleUnit));
        spin(output, turnControllerAngleUnit);
        return turnController.getError();
    }

    public double getArmAngle(AngleUnit angleUnit) {
        return angleUnit.fromRadians(RobotSpecifications.armTicsToRadians.apply(armLift.getCurrentPosition()));
    }

    public Point getPosition(DistanceUnit distanceUnit) {
        double fl = frontLeft.getCurrentPosition()/driveWheelCountsPerMm;
        double fr = frontRight.getCurrentPosition()/driveWheelCountsPerMm;
        double rl = rearLeft.getCurrentPosition()/driveWheelCountsPerMm;
        double rr = rearRight.getCurrentPosition()/driveWheelCountsPerMm;

        double y = fl*frontLeftParameters.yPower()+
                fr*frontRightParameters.yPower()+
                rl*rearLeftParameters.yPower()+
                rr*rearRightParameters.yPower();

        double x = fl*frontLeftParameters.xPower()+
                fr*frontRightParameters.xPower()+
                rl*rearLeftParameters.xPower()+
                rr*rearRightParameters.xPower();

        return new Point(distanceUnit.fromMm(x), distanceUnit.fromMm(y), distanceUnit);
    }

    public void resetPosition() {
        for (DcMotorEx motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void smoothDrive(Point targetPosition) {
//        Point targetPositionMm = targetPosition.toDistanceUnit(DistanceUnit.MM);
//        Point position = getPosition(DistanceUnit.MM);
//        double deltaX = targetPositionMm.x()-position.x();
//        double deltaY = targetPositionMm.y()-position.y();
//        double hypot = Math.hypot(deltaX, deltaY);
//        double theta = Math.atan(deltaY/deltaX);
//        double velocity = 0;
//        if (hypot > 100) { // if still far away
//            velocity =
//        }

    }
    public double getYPosition(DistanceUnit distanceUnit) {
        return getPosition(distanceUnit).y();
    }

    public double getXPosition(DistanceUnit distanceUnit) {
        return getPosition(distanceUnit).x();

    }
    // Getters
    public DcMotorEx getRearRight() {
        return rearRight;
    }

    public DcMotorEx getRearLeft() {
        return rearLeft;
    }

    public DcMotorEx getFrontRight() {
        return frontRight;
    }

    public DcMotorEx getFrontLeft() {
        return frontLeft;
    }

    public DcMotorEx getArmLift() {
        return armLift;
    }

    public IMU getImu() {
        return imu;
    }

}
