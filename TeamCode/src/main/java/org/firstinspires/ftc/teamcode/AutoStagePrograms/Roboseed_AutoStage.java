package org.firstinspires.ftc.teamcode.AutoStagePrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Roboseed_AutoStage.java
 *
 * program for auto stage, not completed.
 * the robot starts in the corner of the field.
 * first, the robot moves out of the parking spot and rotates 90 degree to face the navigation marks,
 * the robot moves to position(according to camera) -1022, -782.
 * the auto stage programs should extend this program as they have common procedure in the first 25 seconds,
 * but the auto stage programs should have different objectives in the last 5 seconds, they maybe go to a fixed sector according to pilot's selection, or may use cameras to determine
 *
 * TODO complete this program by making it a periodic program
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
abstract class Roboseed_AutoStage extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();

    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis robotChassis;
    private Arm arm;
    /** the number of the sector the robot parks into by the end of auto stage */
    private short parkingSectorNum;

    /**
     * the main entry of the robot's program during auto stage
     *
     * @throws InterruptedException: when the operation mode is interrupted by the system
     */
    @Override
    public void runOpMode() throws InterruptedException {
        configureRobot();

        /** pass the hardware ports to the field navigation module */
        HashMap<String, RobotModule> fieldNavigationDependentModules = null;
        HashMap<String, Object> fieldNavigationDependentInstances = new HashMap<>(1);
        fieldNavigationDependentInstances.put("hardwareMap", hardwareMap);
        fieldNavigation = new ComputerVisionFieldNavigation_v2();
        fieldNavigation.init(fieldNavigationDependentModules, fieldNavigationDependentInstances);

        /** pass the hardware ports, drivers and dependent modules to the auto stage chassis module, which is for testing */
        HashMap<String, RobotModule> autoStageRobotChassisDependentModules = new HashMap<>(1);
        HashMap<String, Object> autoStageRobotChassisDependentInstances = new HashMap<>(1);
        autoStageRobotChassisDependentModules.put("fieldNavigation", fieldNavigation);
        autoStageRobotChassisDependentInstances.put("hardwareDriver", hardwareDriver);
        autoStageRobotChassisDependentInstances.put("hardwareMap", hardwareMap);
        this.robotChassis = new AutoStageRobotChassis();
        this.robotChassis.init(autoStageRobotChassisDependentModules, autoStageRobotChassisDependentInstances);
        elapsedTime.reset();

        /** pass the hardware ports to the arm module */
        HashMap armModuleDependentModules = null;
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);

        /* determines where to park */
        this.parkingSectorNum = determineParkingSector();

        waitForStart();

        /** start of the auto stage scripts */

        proceedAutoStageInstructions();

        /** determine where to park */
        switch (parkingSectorNum) {
            case 1: {
                proceedGoToSector1();
                break;
            }
            case 2: {
                proceedGoToSector2();
                break;
            }
            case 3: {
                proceedGoToSector3();
                break;
            }
        }
    }

    /**
     * the function that to set up the robot's hardware
     */
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

    /**
     * determine which sector to park in by the end of the program
     * overwrite this method, as the auto stage programs should have different objectives in the last 5 seconds,
     * they maybe go to a fixed sector according to pilot's selection, or may use cameras to determine which sector to go
     */
    abstract short determineParkingSector();

    /*
     * the instruction given to the robot to make it score
     *  1. the robot grabs the signal sleeves, go to the center of the field
     *  2. the robot moves to the targeted towers, lifts its arm , move a step forward and score goal
     *  3. the robot release its arm, move to the center of the current grid
     *  4. the robot moves, according to the instructions that pilots selected manually in the beginning, to the parking sector that the signal sleeves pointed
     *
     * @param Nah
     * @return Nah
     * @throws InterruptedException
     * */
    private void proceedAutoStageInstructions() throws InterruptedException {
        // grab the preloaded sleeve
        arm.closeClaw();
        Thread.sleep(1000);

        // go to the center of the grid (200, 130), in reference to the red side team
        robotChassis.setRobotPosition(0, 100);

        // line up vertically with the place where the targets
        robotChassis.setRobotPosition(-800, 100);
        robotChassis.setRobotPosition(-800, 780);
        robotChassis.setRobotPosition(-1340, 780);

        // turn the robot to the goal
        robotChassis.setRobotRotation(0);

        // raise the arm
        arm.toHighArmPosition();

        // go forward a step
        robotChassis.setRobotPosition(-1340, 860);

        // place the preloaded goal
        arm.toMidArmPosition();
        arm.openClaw();

        // go to the sleeve stack
        robotChassis.setRobotPosition(-1340, 780); // step back from the goal
        robotChassis.setRobotRotation(0);
        robotChassis.setRobotPosition(-800, 780);

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
    }

    /**
     * go to sector 1 if the pilot asks to
     */
    private void proceedGoToSector1() {
        robotChassis.setRobotRotation(0);
        robotChassis.setRobotPosition(-800, 780);
    }
    /**
     * go to sector 2 if the pilot asks to
     */
    private void proceedGoToSector2() {
        robotChassis.setRobotRotation(0);
        robotChassis.setRobotPosition(-50, 780);
    }
    /**
     * go to sector 3 if the pilot asks to
     */
    private void proceedGoToSector3() {
        robotChassis.setRobotRotation(0);
        robotChassis.setRobotPosition(700, 780);
    }
}
