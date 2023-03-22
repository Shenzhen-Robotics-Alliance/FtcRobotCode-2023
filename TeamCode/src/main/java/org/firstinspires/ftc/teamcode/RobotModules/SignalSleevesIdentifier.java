package org.firstinspires.ftc.teamcode.RobotModules;

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
        // TODO finish here
    }

    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {

    }

    @Override
    public void periodic() {

    }
}
