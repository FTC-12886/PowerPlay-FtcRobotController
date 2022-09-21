package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo"); // this should be a private class variable if using a normal OpMode
        waitForStart(); // wait for driver to press play

        while (opModeIsActive()) { // repeat while running
            for (double count = 0; count < 1; count += 0.01) { // move forwards
                servo.setPosition(count);
                Thread.sleep(20); //wait a little bit
            }
            for (double count = 1; count > 0; count -= 0.01) { // move backwards
                servo.setPosition(count);
                Thread.sleep(20); //wait a little bit
            }
        }
    }
}
