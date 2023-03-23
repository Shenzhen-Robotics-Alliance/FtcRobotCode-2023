/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: ComputerVisionFieldNavigation_v2.java
 *
 * the program that senses the position of the robot during auto stage
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.2.0
 * */
package org.firstinspires.ftc.teamcode.RobotModules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

public class ComputerVisionFieldNavigation_v2 extends RobotModule {
    /**
     * the position of the x coordinate when storing in an array.
     * reverse x and y axles to fit the robot unless the robot will turn 90 degree when facing navigation sign
     */
    private final short xAxisPositionInArray = 0;
    /**
     * the position of the x coordinate when storing in an array.
     * reverse x and y axles to fit the robot unless the robot will turn 90 degree when facing navigation sign
     */
    private final short yAxisPositionInArray = 1;
    /** the position of the x coordinate when storing in an array */
    private final short zAxisPositionInArray = 2;
    /**
     * the factor that will be multiplied to the x-axis coordinate to correct to format it,
     * leave it to be 1 if it's correct already
     */
    private final double xAxisPositionCorrectionFactor = 1;
    /**
     * the factor that will be multiplied to the y-axis coordinate to correct to format it
     * reverse y axis to match the field
     */
    private final double yAxlesPositionCorrectionFactor = -1;
    /**
     * the factor that will be multiplied to the z-axis coordinate to correct to format it,
     * leave it to be 1 if it's correct already
     */
    private final double zAxlesPositionCorrectionFactor = 1;

    /** some vuforia instances */
    private final VuforiaCurrentGame vuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults vuforiaResults;

    /** the array that stores the robot's position in a 3d space */
    private double[] robotPosition;
    /** the variable that stores */
    private double robotRotation;
    private ElapsedTime positionLastUpdateTime = new ElapsedTime();

    /** construction method of field navigation module */
    public ComputerVisionFieldNavigation_v2() {
        /* call to super and pass in the name of this module */
        super("fieldNavigation");
        /* instantiate the vuforia play instance */
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
    }

    /**
     * initialize the field navigation system
     *
     * @param dependentModules: null should be given as this module does not depend on any other modules
     * @param dependentInstances: the instance needed by the robot's chassis
     *                          "hardwareMap" : HardwareMap the connection to the robot's hardware
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            @NonNull
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out an exception if the instances or given empty or if the hardware map is not specified */
        if (dependentInstances.isEmpty()) throw new NullPointerException (
                "an empty set of dependent instance given to the module<<" + this.getModuleName() + ">> which requires at least one instance(s)"
        );
        if (!dependentInstances.containsKey("hardwareMap")) throw new NullPointerException(
                "required dependency <<" + "hardwareMap" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the connection to the robot's hardware */
        HardwareMap hardwareMap = (HardwareMap) dependentInstances.get("hardwareMap");

        /* set the params for vuforia module */
        final String vuforiaLicenseKey = "";
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        final String webcamCalibrationFilename = "";
        final boolean useExtendedTracking = false;
        final boolean enableCameraMonitoring = true;
        VuforiaLocalizer.Parameters.CameraMonitorFeedback cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        final float dx = 0; final float dy = 0; final float dz = 0;
        AxesOrder axesOrder = AxesOrder.XZY;
        float firstAngle = 90; float secondAngle = 90; float thirdAngle = 0;
        boolean useCompetitionFieldTargetLocations = true; // params for initializing vuforia power play

        /* start the vuforia module */
        vuforiaPOWERPLAY.initialize(vuforiaLicenseKey, cameraName, webcamCalibrationFilename, useExtendedTracking, enableCameraMonitoring, cameraMonitorFeedback, dx, dy, dz, axesOrder, firstAngle, secondAngle, thirdAngle, useCompetitionFieldTargetLocations);
        vuforiaPOWERPLAY.activate();

        /* initialize the robot's position */
        robotPosition = new double[3];
    }

    /**
     * update an instance used in the module
     * @Deprecated this module does not support updating instances once the initialization is completed
     * @TODO this module may or may not support changing between different cameras in the future
     */
    @Override @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    /**
     * update the position of the robot according to the available navigation signs, if any
     * call this method in every run loop of the main program
     */
    @Override
    public void periodic() {
        final String[] targets = {"Red Audience Wall", "Red Rear Wall", "Blue Audience Wall", "Blue Rear Wall"};

        for (String target: targets) if (isTargetVisible(target)) {
            processTarget();
            positionLastUpdateTime.reset();
        }
    }

    /**
     * checks whether a selected target is in the range of the camera
     *
     * @param trackableName the name of the target selected
     *                      "Red Audience Wall", the wall facing the audience, in the left side of the field;
     *                      "Red Rear Wall", TODO add explanation;
     *                      "Blue Audience Wall", TODO add explanation;
     *                      "Blue Rear Wall", TODO add explanation;
     * @return a boolean data representing the visibility of the selected target;
     * */
    private boolean isTargetVisible(String trackableName) {
        // Get vuforia results for target.
        vuforiaResults = vuforiaPOWERPLAY.track(trackableName);
        return vuforiaResults.isVisible;
    }

    /**
     * process and store the position of the robot using the raw vuforia data
     * */
    private void processTarget() {
        robotPosition[xAxisPositionInArray] = vuforiaResults.x * xAxisPositionCorrectionFactor;
        robotPosition[yAxisPositionInArray] = vuforiaResults.y * yAxlesPositionCorrectionFactor;
        robotPosition[zAxisPositionInArray] = vuforiaResults.z * zAxlesPositionCorrectionFactor;
        robotRotation = Math.toRadians(vuforiaResults.zAngle);
    }

    /**
     *  get the position of the robot determined by the camera in the last time the camera sees a navigation sign
     *  @return the position of the robot in an array with length three, the data is in millimeter
     */
    public double[] getRobotPosition() { return robotPosition; }
    /**
     *  get the facing of the robot determined by the camera in the last time the camera sees a navigation sign
     *  @return the rotation of the robot, in radian
     */
    public double getRobotRotation() { return robotRotation; }

    /**
     * to know how fresh are the data
     * @return the time between present and the last the time the data is updated from the camera
     */
    public double PositionLastUpdate() { return positionLastUpdateTime.seconds(); }
    /**
     * to know whether there is(are) at least one navigation sign(s) available to navigate
     * @return the visibility of any single one of all the targets
     */
    public boolean checkNavigationSignsVisibility() {
        // check if any navigation sign is available
        final String[] targets = {"Red Audience Wall", "Red Rear Wall", "Blue Audience Wall", "Blue Rear Wall"};
        for (String target: targets) if (isTargetVisible(target)) return true; // run the check through all navigation signs
        return false;
    }
}
