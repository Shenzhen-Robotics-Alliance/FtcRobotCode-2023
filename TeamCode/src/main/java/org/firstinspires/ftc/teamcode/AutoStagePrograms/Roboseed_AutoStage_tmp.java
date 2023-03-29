package org.firstinspires.ftc.teamcode.AutoStagePrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageArm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis_tmp;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Mini1024EncoderReader;
import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator_tmp;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Roboseed_AutoStage_tmp.java
 *
 * a temporary program for auto stage
 * the robot starts in the right corner of the field, with it's left side lining up with the righter border between the second and third region, counting from the right side
 * step1, the robot moves out of the parking spot to the center of the third(counting from the right) region
 * step2, the robot places the pre-loaded
 * the robot moves to position(according to camera) -1022, -782.
 * the auto stage programs should extend this program as they have common procedure in the first 25 seconds,
 * but the auto stage programs should have different objectives in the last 5 seconds, they maybe go to a fixed sector according to pilot's selection, or may use cameras to determine
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
public abstract class Roboseed_AutoStage_tmp extends LinearOpMode {
    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis_tmp robotChassis;
    private Arm arm;
    private AutoStageArm autoStageArm;
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
        this.parkingSectorNum = determineParkingSector();

        /** pass the hardware ports to the arm module */
        HashMap armModuleDependentModules = null;
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);

        /** the temporary arm module to operate the arm during auto stage */
        autoStageArm = new AutoStageArm(arm);

        /** pass the hardware ports to the encoder reader module */
        HashMap<String, RobotModule> encoderReaderDependentModules = null;
        HashMap<String, Object> encoderReaderDependentInstances = new HashMap<>(1);
        /* no enough ports, use the encoder ports of the driving motors instead */
        encoderReaderDependentInstances.put("encoder-1-instance", hardwareDriver.leftFront);
        encoderReaderDependentInstances.put("encoder-2-instance", hardwareDriver.rightFront);
        encoderReaderDependentInstances.put("encoder-3-instance", hardwareDriver.leftRear);
        Mini1024EncoderReader encoderReader = new Mini1024EncoderReader();
        encoderReader.init(encoderReaderDependentModules, encoderReaderDependentInstances);

        /** pass the encoder reader to the temporary position calculator */
        HashMap<String, RobotModule> positionCalculatorDependentModules = new HashMap<>(1);
        HashMap<String, Object> positionCalculatorDependentInstances = null;
        positionCalculatorDependentModules.put("encoderReader", encoderReader);
        RobotPositionCalculator_tmp positionCalculator = new RobotPositionCalculator_tmp();
        positionCalculator.init(positionCalculatorDependentModules, positionCalculatorDependentInstances);

        /** the temporary chassis module */
        this.robotChassis = new AutoStageRobotChassis_tmp(hardwareMap, hardwareDriver, positionCalculator);


        /* sey the parking sector */
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

    /**
     * the instruction given to the robot to make it score
     *  the robot starts in the right corner of the field, with it's left side lining up with the righter border between the second and third region, counting from the right side
     *  step1, the robot moves out of the parking spot and go to the center of the starting region
     *  step2, the robot moves to next region in the front, then to right behind the tower, and place the pre-loaded sleeve onto it
     *  step3, the robot goes back to the
     *  the robot moves to position(according to camera) -1022, -782.
     *
     *
     * @throws InterruptedException if the process is interrupted by sytem
     * */
    private void proceedAutoStageInstructions() throws InterruptedException {
        /* grab the pre-loaded sleeve */
        autoStageArm.holdPreLoadedSleeve();
        /* step1, the robot moves out of the parking spot and go to the center of the starting region */
        robotChassis.setRobotPosition(-11250, 2000);

        /* step2, the robot moves to next region in the front */
        robotChassis.setRobotPosition(-11250, 14500);
        /* then to right behind the tower */
        robotChassis.setRobotPosition(-19000, 14500);
        /* place the pre-loaded sleeve onto it */
        autoStageArm.goToHighestTower();
        robotChassis.setRobotPosition(-19000, 17000);
        autoStageArm.dropSleeve();

        // TODO finish the rest
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