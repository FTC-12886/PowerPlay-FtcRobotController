package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(preselectTeleOp = "AbstractedTeleOp")
public class Auto extends OpMode {
//    private Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//    private static final String TFOD_MODEL_FILE  = Environment.getExternalStorageDirectory().getPath()+"/FIRST/tflitemodels/signalsleeve2.tflite";
    private static final String[] LABELS = {
            "1 Blue Rectangles",
            "2 Red Triangles",
            "3 Green Ellipses"
    };
    private static final String VUFORIA_KEY = "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int parkingLocation = 1;
    private Robot robot;


    public void init() {
        try {
            robot = new Robot(hardwareMap);
            initVuforia();
            initTfod();

            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1.0, 16.0 / 9.0);
            }

            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.log().add("tfod no working, assume parking location 1");
            telemetry.update();
        }
    }

    public void init_loop() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                parkingLocation = 0; // reset average
                for (Recognition recognition : updatedRecognitions) {
                    String label = recognition.getLabel();
                    if (LABELS[0].equals(label)) {
                        parkingLocation += 1;
                    } else if (LABELS[1].equals(label)) {
                        parkingLocation += 2;
                    } else if (LABELS[2].equals(label)) {
                        parkingLocation += 3;
                    }
                }
                parkingLocation = (int) Math.round((double) parkingLocation / updatedRecognitions.size()); // take an average
                telemetry.addData("location", parkingLocation);
                telemetry.update();
            }
        }
    }

    @Override
    public void start() {
        switch (parkingLocation) {
            case 1:
                robot.drive(1, DistanceUnit.METER, 180, AngleUnit.DEGREES);
                while (robot.getXPosition(DistanceUnit.METER) > -3.75) ;
                robot.stop();
                break;
            case 3:
                robot.drive(1, DistanceUnit.METER, 0, AngleUnit.DEGREES);
                while (robot.getXPosition(DistanceUnit.METER) < 3) ;
                robot.stop();
                break;
            default:
                robot.drive(0.25, DistanceUnit.METER); // 2
                while (robot.getYPosition(DistanceUnit.METER) < 1.75) ;
                robot.stop();
                break;
        }
    }

    @Override
    public void loop() {

    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        try {
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (IllegalArgumentException ex) {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
//        tfodParameters.minResultConfidence = 0.01f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//         tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
