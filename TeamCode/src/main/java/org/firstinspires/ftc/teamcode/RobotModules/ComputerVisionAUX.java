/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: ComputerVisionAUX.java
 *
 * the program that does all the work with computer vision systems in manual stage to support the pilot
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */
package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class ComputerVisionAUX {
    private final VuforiaTrackables relicTrackables;
    private final VuforiaTrackable relicTemplate;

    public ComputerVisionAUX(HardwareMap hardwareMap) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );

        parameters.vuforiaLicenseKey = "AUrp8E//////AAABmR9EFFA6dECthZSIH4YjVNY8QqiA+WhfTqkO4koNt+QYbXrW9k3nqcvIL16tY13jQdTckP3BXh9+vKLJHQjATt6uffiXKfKUM3k+q3ZcB5g8v8+FT1WKJaOIf+vgOl84gghjBgira8FLQwuOB62wzkJRpoMKSNoyunOqG3PR7ttCl0izt9VRfaJy/2CmqLkXNj//tjcRj1xRqf3Xpo9omrSQcC8qQIRapKCcFUWyc+86WzQD2uVRwBAicmIs6yAHvUqMh/Xj0waomLYgvZ4CTDhgX2srIEgnLmlzxxIy2bggLV7vxnFILfe3wzLh1HYuQ0mHV9YF8ShgxXEbCrQDmUwripCsh3IST2Oxk/ZyVGZD";
        parameters.cameraName = webcamName;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        this.relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("cone"); // for debugging
    }

    public void activateAUX() {
        relicTrackables.activate();
    }

    public void aimCone() {
        AimCone aimCone = new AimCone(this.relicTemplate);
        Thread aimConeThread = new Thread(aimCone);
    }

    public void test() {
        ObjectDetectionUnitTest objectDetectionUnitTest = new ObjectDetectionUnitTest(relicTemplate);
        Thread testThread = new Thread(objectDetectionUnitTest);
        testThread.start();
    }
}

class AimCone implements Runnable {
    RelicRecoveryVuMark vuMark;
    VuforiaTrackable relicTemplate;
    public AimCone(VuforiaTrackable relicTemplate) {
        this.relicTemplate = relicTemplate;
    }
    @Override
    public void run() {
        do {
            this.vuMark = RelicRecoveryVuMark.from(relicTemplate);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getFtcCameraFromTarget(); // get the position of the vumark
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); // analyze the result

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;

            // TODO track the cone using the two sets of data each representing the translation and rotation of target, when the aiming precess is settled, exit this function
        } while (vuMark != RelicRecoveryVuMark.UNKNOWN); // when an vumark is visible
    }
}

class ObjectDetectionUnitTest implements Runnable {
    RelicRecoveryVuMark vuMark;
    VuforiaTrackable relicTemplate;
    public ObjectDetectionUnitTest(VuforiaTrackable relicTemplate) {
        this.relicTemplate = relicTemplate;
        System.out.println("<--new test generated-->");
    }
    @Override
    public void run() {
        System.out.println("<--test running...-->");
        while (true) {
            this.vuMark = RelicRecoveryVuMark.from(relicTemplate);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getFtcCameraFromTarget(); // get the position of the vumark
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); // analyze the result

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                System.out.print("<--vumark detected ");
                System.out.print("translation:");
                System.out.print(tX);
                System.out.print(", ");
                System.out.print(tY);
                System.out.print(", ");
                System.out.print(tZ);
                System.out.println("-->");
            } else {
                System.out.println("<--no object detected-->");
            }
        }
    }
}