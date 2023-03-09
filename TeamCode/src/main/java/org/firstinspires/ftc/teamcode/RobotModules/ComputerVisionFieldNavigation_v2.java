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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class ComputerVisionFieldNavigation_v2 implements Runnable {
    // some presets
    // TODO make these following presets a list, containing the preset for each navigation sign
    private final short xAxisPositionInArray = 0;
    private final short yAxisPositionInArray = 1; // do not reverse x and y axles to fit the robot, as the robot will turn 90 degree when facing navigation sign
    private final short zAxisPositionInArray = 2;
    private final double xAxisPositionCorrectionFactor = 1;
    private final double yAxlesPositionCorrectionFactor = -1; // reverse y axis to match the field
    private final double zAxlesPositionCorrectionFactor = 1;
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults vuforiaResults; // some vuforia instances
    
    private double[] robotPosition;
    private double robotRotation;
    private ElapsedTime positionLastUpdateTime = new ElapsedTime();

    private boolean paused = false;
    private boolean terminated = false;
    
    public ComputerVisionFieldNavigation_v2(HardwareMap hardwareMap) {
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

        vuforiaPOWERPLAY = new VuforiaCurrentGame(); // instantiate
        vuforiaPOWERPLAY.initialize(vuforiaLicenseKey, cameraName, webcamCalibrationFilename, useExtendedTracking, enableCameraMonitoring, cameraMonitorFeedback, dx, dy, dz, axesOrder, firstAngle, secondAngle, thirdAngle, useCompetitionFieldTargetLocations);
        vuforiaPOWERPLAY.activate();

        robotPosition = new double[3];
    }

    @Override
    public void run() {
        while (!terminated) {
            updateRobotPosition();
            threadReactions();
        }
    }

    // private void updateRobotPosition() {
    public void updateRobotPosition() {
        final String[] targets = {"Red Audience Wall", "Red Rear Wall", "Blue Audience Wall", "Blue Rear Wall"};

        for (String target: targets) if (isTargetVisible(target)) {
            processTarget();
            positionLastUpdateTime.reset();
        }
    }

    private boolean isTargetVisible(String trackableName) {
        // Get vuforia results for target.
        vuforiaResults = vuforiaPOWERPLAY.track(trackableName);
        return vuforiaResults.isVisible;
    }

    private void processTarget() {
        robotPosition[xAxisPositionInArray] = vuforiaResults.x * xAxisPositionCorrectionFactor;
        robotPosition[yAxisPositionInArray] = vuforiaResults.y * yAxlesPositionCorrectionFactor;
        robotPosition[zAxisPositionInArray] = vuforiaResults.z * zAxlesPositionCorrectionFactor;
        robotRotation = Math.toRadians(vuforiaResults.zAngle);
    }

    public void threadReactions() { // reactions when the process is paused or terminated
        do {
            if (terminated) {
                vuforiaPOWERPLAY.deactivate(); vuforiaPOWERPLAY.close();
                break;
            }
            Thread.yield();
        } while (paused);
    }

    public double[] getRobotPosition() { return robotPosition; } // return the position of the robot, in mm
    public double getRobotRotation() { return robotRotation; } // return the rotation of the robot, in radian
    public double PositionLastUpdate() { return positionLastUpdateTime.seconds(); } // so that you can know how accurate the results are
    public boolean checkNavigationSignsVisibility() {
        if (terminated) return false; // do not execute if the program is already put to stop
        // check if any navigation sign is available
        final String[] targets = {"Red Audience Wall", "Red Rear Wall", "Blue Audience Wall", "Blue Rear Wall"};
        for (String target: targets) if (isTargetVisible(target)) return true; // run the check through all navigation signs
        return false;
    }

    public void pause() { paused = true; }
    public void resume() { paused = false;}
    public void terminate() { terminated = true; }
}
