/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: pos2.java
 *
 * auto stage program
 * the robot moves to position 2 by the end
 * the pilot selects the position manually to temporarily replace signal sleeves
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */
package org.firstinspires.ftc.teamcode.AutoStagePrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;

import java.util.HashMap;

@Autonomous(name = "pos2")
public class pos2 extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private boolean terminationFlag;

    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis chassisModule;
    private Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        configureRobot();

        /** pass the hardware ports to the field navigation module */
        HashMap<String, RobotModule> fieldNavigationDependentModules = null;
        HashMap<String, Object> fieldNavigationDependentInstances = new HashMap<>(1);
        fieldNavigationDependentInstances.put("hardwareMap", hardwareMap);
        fieldNavigation = new ComputerVisionFieldNavigation_v2();
        fieldNavigation.init(fieldNavigationDependentModules, fieldNavigationDependentInstances);

        chassisModule = new AutoStageRobotChassis(hardwareDriver, hardwareMap, fieldNavigation);
        chassisModule.initRobotChassis();
        elapsedTime.reset();

        /* pass the hardware ports to the arm module, TODO write this method for all the modules */
        HashMap armModuleDependentModules = null;
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
            while (!isStopRequested() && opModeIsActive()) Thread.yield();
            chassisModule.terminate();
            terminationFlag = true;
        }
        }); terminationListenerThread.start();

        Thread robotStatusMonitoringThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
                String cameraPositionString = robotCurrentPosition[0] + " " + robotCurrentPosition[1] + " " + robotCurrentPosition[2];
                telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

                double[] encoderPosition = chassisModule.getEncoderPosition();
                String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
                telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

                telemetry.update();
            }
        });

        terminationListenerThread.start();
        robotStatusMonitoringThread.start();
        waitForStart();

        // start of the auto stage scripts
        proceedAutoStageInstructions();

        proceedGoToSector2();


        // end of the program
        chassisModule.terminate();
        arm.deactivateArm();

    }

    private void configureRobot() {
        hardwareDriver.leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        hardwareDriver.leftRear = hardwareMap.get(DcMotorEx.class, "leftrear");
        hardwareDriver.rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
        hardwareDriver.rightRear = hardwareMap.get(DcMotorEx.class, "rightrear");

        hardwareDriver.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareDriver.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        hardwareDriver.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareDriver.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareDriver.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareDriver.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwareDriver.claw = hardwareMap.get(Servo.class, "tipperhopper");

        hardwareDriver.lift_left = hardwareMap.get(DcMotorEx.class, "lifter");
        hardwareDriver.lift_right = hardwareMap.get(DcMotorEx.class, "lifter_right");

        hardwareDriver.lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void proceedAutoStageInstructions() throws InterruptedException {
        // grab the preloaded sleeve
        arm.closeClaw();
        Thread.sleep(1000);

        // go to the center of the grid (200, 130), in reference to the red side team
        chassisModule.setRobotPosition(0, 100);

        // line up vertically with the place where the sleeves are stored
        chassisModule.setRobotPosition(-800, 100);
        chassisModule.setRobotPosition(-800, 780);
        chassisModule.setRobotPosition(-1340, 780);

        // turn the robot to the goal
        chassisModule.setRobotRotation(0);

        // raise the arm
        arm.toHighArmPosition();

        // go forward a step
        chassisModule.setRobotPosition(-1340, 860);

        // place the preloaded goal
        arm.toMidArmPosition();
        arm.openClaw();

        // go to the sleeve stack
        chassisModule.setRobotPosition(-1340, 780); // step back from the goal
        chassisModule.setRobotRotation(0);
        chassisModule.setRobotPosition(-800, 780);

        // drop tHE arm
        sleep(200);
        arm.toMidArmPosition();
        arm.deactivateArm();

        /* sleep(2000);

        // turn the robot to the stick
        chassisModule.setRobotRotation(Math.toRadians(90));

        // precise navigation to the sleeves using visual guidance
        if (fieldNavigation.checkNavigationSignsVisibility()) // if the navigation signs are available
            chassisModule.setRobotPositionWithVisualNavigation(-1000, 1500); // visual guidance to the sleeves
        else chassisModule.setRobotPosition(0, 600); // other wise, dive to the sleeves using encoders TODO set this position to make robot very close to the sleeves

        /*
        // raise the arms
        armControllingMethods.openClaw();
        armControllingMethods.toLoadingArmPosition();

        // go to the sleeves
        chassisModule.moveRobotWithEncoder(0, 200);
        // grab a sleeve and raise it up
        armControllingMethods.closeClaw();
        armControllingMethods.toMidArmPosition();
        // step back and drag it on the ground
        chassisModule.moveRobotWithEncoder(0, -200);
        armControllingMethods.toGroundArmPosition();

        // turn the robot to the goal
        chassisModule.setRobotRotation(270);

        // go to the goal
        if (fieldNavigation.checkNavigationSignsVisibility())
            chassisModule.setRobotPositionWithVisualNavigation(0, 0);
        else chassisModule.moveRobotWithEncoder(0, 0); // TODO set the positions to make the position line up and stick close with the goal one step away

        // raise the arm
        armControllingMethods.toHighArmPosition();

        // go forward a step
        chassisModule.moveRobotWithEncoder(0, 100); // TODO set the position so that the sleeve goes right to the goal

        // place the sleeve
        armControllingMethods.deactivateArm();
        armControllingMethods.openClaw();
        chassisModule.moveRobotWithEncoder(0, -100); // step back from the goal
        */

        // TODO move to parking position according to the driver input to pretend having visual recognizing
    }

    private void proceedGoToSector1() {
        chassisModule.setRobotRotation(0);
        chassisModule.setRobotPosition(-800, 780);
    }

    private void proceedGoToSector2() {
        chassisModule.setRobotRotation(0);
        chassisModule.setRobotPosition(-50, 780);
    }

    private void proceedGoToSector3() {
        chassisModule.setRobotRotation(0);
        chassisModule.setRobotPosition(700, 780);
    }
}
