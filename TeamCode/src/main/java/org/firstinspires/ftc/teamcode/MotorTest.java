package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor servo = hardwareMap.get(DcMotor.class, "motor"); // this should be a private class variable if using a normal OpMode
        waitForStart(); // wait for driver to press play

        while (opModeIsActive()) { // repeat while running
            for (double count = -1; count < 1; count += 0.01) { // move forwards
                servo.setPower(count);
                Thread.sleep(20); //wait a little bit
            }
            for (double count = 1; count > -1; count -= 0.01) { // move backwards
                servo.setPower(count);
                Thread.sleep(20); //wait a little bit
            }
        }
    }
}
