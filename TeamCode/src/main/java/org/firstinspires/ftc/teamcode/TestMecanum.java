package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestMecanum extends OpMode {

    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    @Override
    public void init() {
        rearRight = hardwareMap.get(DcMotor.class, "backRight");
        rearLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x); // quadratic curve but flipped
        double leftStickY = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double rightStickX = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double rightStickY = -gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);
        double rearRightPower = 0;
        double rearLeftPower = 0;
        double frontRightPower = 0;
        double frontLeftPower = 0;

        // mecanum code

        // forwards/backwards
        rearRightPower += leftStickY;
        rearLeftPower += leftStickY;
        frontRightPower += leftStickY;
        frontLeftPower += leftStickY;

        // sideways
        rearRightPower -= leftStickX;
        rearLeftPower += leftStickX;
        frontRightPower += leftStickX;
        frontLeftPower -= leftStickX;

        // turning
        frontLeftPower += rightStickX;
        rearLeftPower += rightStickX;

        frontRightPower -= rightStickX;
        rearRightPower -= rightStickX;

        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("RL", rearLeftPower);
        telemetry.addData("RR", rearRightPower);

        rearRight.setPower((gamepad1.left_bumper ? 1 : 0.5)*rearRightPower);
        rearLeft.setPower((gamepad1.left_bumper ? 1 : 0.5)*rearLeftPower);
        frontRight.setPower((gamepad1.left_bumper ? 1 : 0.5)*frontRightPower);
        frontLeft.setPower((gamepad1.left_bumper ? 1 : 0.5)*frontLeftPower);
    }
}
