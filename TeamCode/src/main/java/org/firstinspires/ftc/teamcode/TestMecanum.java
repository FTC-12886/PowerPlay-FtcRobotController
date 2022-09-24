package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestMecanum extends OpMode {

    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    @Override
    public void init() {
        rearRight = hardwareMap.get(DcMotor.class, "nameOfMotor");
        rearLeft = hardwareMap.get(DcMotor.class, "nameOfMotor2");
        frontRight = hardwareMap.get(DcMotor.class, "nameOfMotor3");
        frontLeft = hardwareMap.get(DcMotor.class, "nameOfMotor4");


    }

    @Override
    public void loop() {
        float leftStickX = gamepad1.left_stick_x;
        float leftStickY = gamepad1.left_stick_y;
        float rightStickX = gamepad1.right_stick_x;
        float rightStickY = gamepad1.right_stick_y;
        double rearRightPower = 0;
        double rearLeftPower = 0;
        double frontRightPower = 0;
        double frontLeftPower = 0;

        // mecanum code
        rearRightPower += leftStickY;
        rearLeftPower += leftStickY;
        frontRightPower += leftStickY;
        frontLeftPower += leftStickY;

        rearRightPower += leftStickX;
        rearLeftPower -= leftStickX;
        frontRightPower -= leftStickX;
        frontLeftPower += leftStickX;

        // turning
        frontLeftPower += rightStickX;
        rearLeftPower += rightStickX;

        frontRightPower -= rightStickX;
        rearRightPower -= rightStickX;

        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
    }
}
