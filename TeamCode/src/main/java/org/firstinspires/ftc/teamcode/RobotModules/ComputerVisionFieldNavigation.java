/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: ComputerVisionFieldNavigation.java
 *
 * the program that senses the position of the robot during auto stage
 * this program is deprecated, please use ComputerVisionFieldNavigation_v2.java
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * @Deprecated this program is no longer suggested, please use ComputerVisionFieldNavigation_v2.java
 * */
package org.firstinspires.ftc.teamcode.RobotModules;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

@Deprecated
public class ComputerVisionFieldNavigation {
    private static final String VUFORIA_KEY = "AUrp8E//////AAABmR9EFFA6dECthZSIH4YjVNY8QqiA+WhfTqkO4koNt+QYbXrW9k3nqcvIL16tY13jQdTckP3BXh9+vKLJHQjATt6uffiXKfKUM3k+q3ZcB5g8v8+FT1WKJaOIf+vgOl84gghjBgira8FLQwuOB62wzkJRpoMKSNoyunOqG3PR7ttCl0izt9VRfaJy/2CmqLkXNj//tjcRj1xRqf3Xpo9omrSQcC8qQIRapKCcFUWyc+86WzQD2uVRwBAicmIs6yAHvUqMh/Xj0waomLYgvZ4CTDhgX2srIEgnLmlzxxIy2bggLV7vxnFILfe3wzLh1HYuQ0mHV9YF8ShgxXEbCrQDmUwripCsh3IST2Oxk/ZyVGZD";

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targets;
    private WebcamName webcamName;

    private NavigationRunnable navigationRunnable;

    public static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    public ComputerVisionFieldNavigation (HardwareMap hardwareMap) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // get the instance of webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false; // set the parameters for vuforia
        vuforia = ClassFactory.getInstance().createVuforia(parameters); // create the vuforia instance

        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay"); // load the trained recognition model

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll((Collection<? extends VuforiaTrackable>) targets); // put all the trackable into an array

        // set the location of the navigation pictures in reference to the field
        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        this.navigationRunnable = new NavigationRunnable(allTrackables);
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void startNavigating() {
        Thread navigationThread = new Thread();
        navigationThread.start();
    }

    public double[] getRobotPosition() {
        return navigationRunnable.getCurrentPosition();
    }

    public double[] getRobotRotation() {
        return navigationRunnable.getCurrentRotation();
    }
}

class NavigationRunnable implements Runnable {
    private boolean paused = false;
    private boolean terminated = false; // to pause or terminate the thread

    private boolean targetVisible = false;
    private OpenGLMatrix lastLocation;

    private double[] currentPosition;
    private double[] currentRotation;

    private List<VuforiaTrackable> allTrackables;

    public NavigationRunnable (List<VuforiaTrackable> allTrackables) {
        this.allTrackables = allTrackables;
        currentPosition = new double[3];
        currentRotation = new double[3];
    }


    @Override
    public void run() {
        while (true) {
            if (terminated) break;
            while (paused) Thread.yield();

            updateRobotPosition();
        }
    }

    public void updateRobotPosition() {
        targetVisible = false;
        for (VuforiaTrackable trackable : this.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            System.out.println("target visible");
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            currentPosition[0] = translation.get(0) / ComputerVisionFieldNavigation.mmPerInch;
            currentPosition[1] = translation.get(1) / ComputerVisionFieldNavigation.mmPerInch;
            currentPosition[2] = translation.get(2) / ComputerVisionFieldNavigation.mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            currentRotation[0] = rotation.firstAngle;
            currentRotation[1] = rotation.secondAngle;
            currentRotation[2] = rotation.thirdAngle;
        }
    }

    public double[] getCurrentPosition() {
        return currentPosition;
    }

    public double[] getCurrentRotation() {
        return currentRotation;
    }

    public void pause() {
        this.paused = true;
    }

    public void resume() {
        this.paused = false;
    }

    public void terminate() {
        this.terminated = true;
    }
}