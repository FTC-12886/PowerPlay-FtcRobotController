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
        float stickX = gamepad1.left_stick_x;
        float stickY = gamepad1.left_stick_y;
        double rearRightPower = 0;
        double rearLeftPower = 0;
        double frontRightPower = 0;
        double frontLeftPower = 0;

        rearRightPower += stickY;
        rearLeftPower += stickY;
        frontRightPower += stickY;
        frontLeftPower += stickY;

        rearRightPower += stickX;
        rearLeftPower -= stickX;
        frontRightPower -= stickX;
        frontLeftPower += stickX;

        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
    }
}
