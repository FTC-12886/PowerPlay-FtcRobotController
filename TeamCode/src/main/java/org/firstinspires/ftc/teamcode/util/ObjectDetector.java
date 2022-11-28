package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class ObjectDetector {
    private static final String VUFORIA_KEY = "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final HardwareMap hardwareMap;

    private String tfodModelFile = null;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] MODEL_ASSET_LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private String[] labels = MODEL_ASSET_LABELS;

    private int parkingLocation = 1;

    public ObjectDetector(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public ObjectDetector(HardwareMap hardwareMap, String tfodModelFile, String[] modelFileLabels) {
        this.hardwareMap = hardwareMap;
        this.tfodModelFile = tfodModelFile;
        this.labels = modelFileLabels;
    }
    public void activate() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 9.0/16.0);
        }
    }
    public void deactivate() {
        tfod.deactivate();
        vuforia.close();
    }
    public int loop() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
                    Optional<Recognition> maxConfidence;
                    maxConfidence = updatedRecognitions.stream().max(Comparator.comparing(Recognition::getConfidence));
                    if (maxConfidence.isPresent()) {
                        Recognition theRecognition = maxConfidence.get();

                        String label = theRecognition.getLabel();
                        if (labels[0].equals(label)) {
                            parkingLocation = 1;
                        } else if (labels[1].equals(label)) {
                            parkingLocation = 2;
                        } else if (labels[2].equals(label)) {
                            parkingLocation = 3;
                        }
                    }
                }
            }
        }
        return parkingLocation;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        List<HardwareDevice> cameras = hardwareMap.getAll(WebcamName.class); // check if have camera
        if (cameras.size() > 0)
            parameters.cameraName = (CameraName) cameras.get(0);
        else
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        if (tfodModelFile != null) { // if specify a model file, use that, else use the bundled model
            tfod.loadModelFromFile(tfodModelFile, labels);
        } else
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, labels);

    }
}
