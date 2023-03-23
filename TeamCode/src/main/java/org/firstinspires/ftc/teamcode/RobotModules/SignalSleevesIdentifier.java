package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: SignalSleevesIdentifier.java
 *
 * identifies the signal sleeves to get the objective parking spot during autonomous period
 * TODO finish this module
 * rotate the image 180 degree around and cast it from pixel (560,360) to (920,720) should get the proper image
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.22
 * @Version 0.0.0
 * */
public class SignalSleevesIdentifier extends RobotModule {
    /** the name of the trained model of object recognition */
    private static final String TFOD_MODEL_ASSET = "SignalSleevesRecognition.tflite";
    /** the name of the labels that the tensorflow model will identify */
    private static final String[] LABELS = {
            "Sector1", // the signal sleeves shows to park in the first sector
            "Sector2", // the signal sleeves shows to park in the second sector
            "Sector3"  // the signal sleeves shows to park in the third sector
    };
    /** the vuforia license that you get from the vuforia website */
    private static final String VUFORIA_KEY =
            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    /** localization engine */
    private VuforiaLocalizer vuforia;
    /** detection engine */
    private TFObjectDetector tfObjectDetector;

    /** construct method of signal sleeves identifier */
    public SignalSleevesIdentifier() {
        super("signalSleevesIdentifier");
    }

    /**
     * initialization of the signal sleeves identifier module
     *
     * @param dependentModules null should be given as this module needs no other modules as dependencies
     * @param dependentInstances this module requires the following instances:
     *                           "hardwareMap" : HardwareMap, the connection to the ports of the robot's hardware
     */
    @Override
    public void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException {
        /** get the instances from the params */
        /* throw out an error if the map of all the instance is given empty or that hardware map is not given */
        if (dependentInstances.isEmpty()) throw new NullPointerException (
                "an empty set of dependent instance given to the module<<" + this.getModuleName() + ">> which requires at least one instance(s)"
        );
        if (!dependentInstances.containsKey("hardwareMap")) throw new NullPointerException(
                "required dependency <<" + "hardwareMap" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the connections to the robot hardware */
        HardwareMap hardwareMap = (HardwareMap) dependentInstances.get("hardwareMap");


        /** configure the vuforia module */
        /* specified the params */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /* create the vuforia engine */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        /** configure the tensorflow engine */
        /* set the params and configurations for the tensorflow engine */
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        /* the minimum accept confidence of the result, lower than that, the model fails */
        tfodParameters.minResultConfidence = 0.5f;
        /* the model is trained using tensorflow 2 */
        tfodParameters.isModelTensorFlow2 = true;
        /* the input size of the picture */
        tfodParameters.inputSize = 360;

        /* create the tensorflow engine */
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        /* load the pre-trained model stored in the file system of the robot's control board */
        tfObjectDetector.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {

    }

    @Override
    public void periodic() {

    }
}
