package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(preselectTeleOp = "AbstractedTeleOp")
public class Auto extends LinearOpMode {
    private final Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    private Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot(hardwareMap);
        } catch (IllegalArgumentException ex) {
            telemetry.log().add(ex.getMessage());
        }
        ObjectDetector objectDetector = new ObjectDetector(hardwareMap);
        objectDetector.activate();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while (opModeInInit()) {
            int detected = objectDetector.loop();
            telemetry.addData("Detected location", detected);
            telemetry.update();
        }
        new Thread() { // dispatch thread to stop tfod
            @Override
            public void run() {
                objectDetector.deactivate();
            }
        }.start();
    }
}
