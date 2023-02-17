package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class ComputerVisionFieldNavigation_v2 implements Runnable {
    // some presets
    private short xAxlesPositionInArray = 1;
    private short yAxlesPositionInArray = 0; // reverse x and y axles to fit the robot
    private short zAxlesPositionInArray = 2;
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
        robotPosition[xAxlesPositionInArray] = vuforiaResults.x;
        robotPosition[yAxlesPositionInArray] = vuforiaResults.y;
        robotPosition[zAxlesPositionInArray] = vuforiaResults.z;
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

    public void pause() { paused = true; }
    public void resume() { paused = false;}
    public void terminate() { terminated = true; }
}
