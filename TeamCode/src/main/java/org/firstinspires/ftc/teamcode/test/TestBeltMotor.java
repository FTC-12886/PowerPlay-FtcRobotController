package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Belt (John Cena) Motor")
@Disabled
public class TestBeltMotor extends OpMode {

    private DcMotor johnCenaMotor;
    @Override
    public void init() {
        johnCenaMotor = hardwareMap.get(DcMotor.class, "John Cena Motor");
        johnCenaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        johnCenaMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        float left_stick_y = gamepad1.left_stick_y;
        johnCenaMotor.setPower(left_stick_y);
    }
}
