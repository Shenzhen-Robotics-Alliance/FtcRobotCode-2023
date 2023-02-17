package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class ComputerVisionSleevesLocating implements Runnable {
    private HardwareMap hardwareMap;
    // load tensorflow lite model from internal storage
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // label the assets
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    // init the vuforia module
    private static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector TFOD;

    // store the data caught from vuforia
    private double col, row, width, height;
    private String detectedLabelName;

    private boolean terminated = false;
    private boolean paused = false;

    public ComputerVisionSleevesLocating(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // init the modules
        initVuforia();
        initTfod();
        if (TFOD != null) {
            TFOD.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            TFOD.setZoom(1.0, 16.0 / 9.0);
        }
    }

    @Override
    public void run() {
        while (!terminated) {
            // yield when the program is asked to pause
            while (paused) Thread.yield();

            if (TFOD != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = TFOD.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    System.out.print("# Objects Detected"); System.out.println(updatedRecognitions.size());
                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        this.col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        this.row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        this.width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        this.height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        /*
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height); */
                    }
                }
            }
        }
    }

    private void initVuforia() {
        // init the vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        // Initialize the TensorFlow Object Detection engine.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters TFODParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        TFODParameters.minResultConfidence = 0.75f;
        TFODParameters.isModelTensorFlow2 = true;
        TFODParameters.inputSize = 300;
        TFOD = ClassFactory.getInstance().createTFObjectDetector(TFODParameters, vuforia);

        // load the module
        TFOD.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    // thread controlling methods
    public void terminate() { terminated = true; }
    public void pause() { paused = true; }
    public void unpause() {paused = false; }
}
