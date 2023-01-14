package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// 3 is back left
// 2 is front right, flip
// 1 is front left, flip
// 0  is back right
@TeleOp
public class MotorDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        telemetry.log().add("frontLeft Forwards");
        telemetry.update();
        frontLeft.setPower(1);
        waitForGamepad();

        telemetry.log().add("frontLeft Backwards");
        telemetry.update();
        frontLeft.setPower(-1);
        waitForGamepad();
        frontLeft.setPower(0);

        telemetry.log().add("frontRight Forwards");
        telemetry.update();
        frontRight.setPower(1);
        waitForGamepad();

        telemetry.log().add("frontRight Backwards");
        telemetry.update();
        frontRight.setPower(-1);
        waitForGamepad();
        frontRight.setPower(0);


        telemetry.log().add("rearRight Forwards");
        telemetry.update();
        rearRight.setPower(1);
        waitForGamepad();

        telemetry.log().add("rearRight Backwards");
        telemetry.update();
        rearRight.setPower(-1);
        waitForGamepad();
        rearRight.setPower(0);

        telemetry.log().add("rearLeft Forwards");
        telemetry.update();
        rearLeft.setPower(1);
        waitForGamepad();

        telemetry.log().add("rearLeft Backwards");
        telemetry.update();
        rearLeft.setPower(-1);
        waitForGamepad();
        rearLeft.setPower(0);

    }
    public void waitForGamepad() {
        while (!gamepad1.a) {
            sleep(10);
        }
        while (gamepad1.a) {
            sleep(10);
        }
    }
}
