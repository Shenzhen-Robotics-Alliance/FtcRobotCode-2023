package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class ComputerVisionFieldNavigation_v2 implements Runnable {
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults vuforiaResults; // some vuforia instances
    
    private double[] robotPosition;
    private double robotRotation;
    private boolean targetDetected;

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
        
        vuforiaPOWERPLAY.initialize(vuforiaLicenseKey, cameraName, webcamCalibrationFilename, useExtendedTracking, enableCameraMonitoring, cameraMonitorFeedback, dx, dy, dz, axesOrder, firstAngle, secondAngle, thirdAngle, useCompetitionFieldTargetLocations);
        
        robotPosition = new double[3];
    }

    @Override
    public void run() {
        while (true) {
            updateRobotPosition();
            threadReactions();
        }
    }

    private void updateRobotPosition() {
        final String[] targets = {"Red Audience Wall", "Red Rear Wall", "Blue Audience Wall", "Blue Rear Wall"};

        for (String target: targets) if (isTargetVisible(target)) {
            processTarget();
            this.targetDetected = true;
        }
    }

    private boolean isTargetVisible(String trackableName) {
        boolean isVisible;

        // Get vuforia results for target.
        vuforiaResults = vuforiaPOWERPLAY.track(trackableName);
        // Is this target visible?
        if (vuforiaResults.isVisible) {
            isVisible = true;
        } else {
            isVisible = false;
        }
        return isVisible;
    }

    private void processTarget() {
        robotPosition[0] = vuforiaResults.x;
        robotPosition[1] = vuforiaResults.y;
        robotPosition[2] = vuforiaResults.z;
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

    public void pause() { paused = true; }
    public void resume() { paused = false;}
    public void terminate() { terminated = true; }
}
