package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        telemetry.log().add("Front Left Forwards");
        telemetry.update();
        frontLeft.setPower(1);
        waitForGamepad();

        telemetry.log().add("Front Left Backwards");
        telemetry.update();
        frontLeft.setPower(-1);
        waitForGamepad();
        frontLeft.setPower(0);

        telemetry.log().add("Front Right Forwards");
        telemetry.update();
        frontRight.setPower(1);
        waitForGamepad();

        telemetry.log().add("Front Right Backwards");
        telemetry.update();
        frontRight.setPower(-1);
        waitForGamepad();
        frontRight.setPower(0);


        telemetry.log().add("Rear Left Forwards");
        telemetry.update();
        rearLeft.setPower(1);
        waitForGamepad();

        telemetry.log().add("Rear Left Backwards");
        telemetry.update();
        rearLeft.setPower(-1);
        waitForGamepad();
        rearLeft.setPower(0);

        telemetry.log().add("Rear Right Forwards");
        telemetry.update();
        rearRight.setPower(1);
        waitForGamepad();

        telemetry.log().add("Rear Right Backwards");
        telemetry.update();
        rearRight.setPower(-1);
        waitForGamepad();
        rearRight.setPower(0);

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
