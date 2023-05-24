package org.firstinspires.ftc.teamcode.AutoStagePrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageArm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Mini1024EncoderReader;
import org.firstinspires.ftc.teamcode.RobotModules.RobotAuxiliarySystem;
import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;

import java.util.HashMap;

import dalvik.system.DelegateLastClassLoader;

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
 * @Date 2023.5.21
 * @Version v0.2.0
 */
abstract class AutoStage extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();

    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ChassisDriver chassis;
    private AutoStageArm arm;
    private RobotPositionCalculator positionCalculator;
    private RobotAuxiliarySystem robotAuxiliarySystem;

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

        /** pass the hardware ports to the arm module */
        HashMap<String, RobotModule> armModuleDependentModules = new HashMap<>(1);
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        armModuleDependentInstances.put("initialControllerPad", new Gamepad());
        Arm armModule = new Arm();
        armModule.init(armModuleDependentModules, armModuleDependentInstances, false);

        /** the temporary arm module to operate the arm during auto stage */
        this.arm = new AutoStageArm(armModule);

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
        this.positionCalculator = new RobotPositionCalculator();
        positionCalculator.init(positionCalculatorDependentModules, positionCalculatorDependentInstances);

        /** the driver of the chassis */
        this.chassis = new ChassisDriver(hardwareDriver, positionCalculator);

        /** the RAS */
        ColorDistanceSensor color = new ColorDistanceSensor(hardwareMap, 1);
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");
        HashMap<String, RobotModule> robotAuxiliarySystemDependentModules = new HashMap<>(1);
        HashMap<String, Object> robotAuxiliarySystemDependentInstances = new HashMap<>(1);
        robotAuxiliarySystemDependentModules.put("positionCalculator", positionCalculator);
        robotAuxiliarySystemDependentInstances.put("colorDistanceSensor", color);
        robotAuxiliarySystemDependentInstances.put("tofDistanceSensor", distance);
        robotAuxiliarySystemDependentInstances.put("chassisDriver", chassis);
        robotAuxiliarySystemDependentModules.put("arm", armModule);
        this.robotAuxiliarySystem = new RobotAuxiliarySystem();
        robotAuxiliarySystem.init(robotAuxiliarySystemDependentModules, robotAuxiliarySystemDependentInstances, this, false);

        chassis.setAutoMode(false);


        /** the thread for telemetry monitoring */
        Thread telemetryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive() && !isStopRequested()) {
                    telemetry.addData("position", positionCalculator.getRobotPosition()[0] + "," + positionCalculator.getRobotPosition()[1]);
                    telemetry.addData("rotation", Math.toDegrees(positionCalculator.getRobotRotation()));
                    telemetry.update();
                }
            }
        }); // TODO replace with robot debug bridge

        /* determines where to park */
        this.parkingSectorNum = determineParkingSector();

        waitForStart();

        telemetryThread.start();

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

        chassis.setRobotTranslationalMotion(0, 0); chassis.setRotationalMotion(0);
        while (opModeIsActive() && !isStopRequested()) {
            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();
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
        /* grab the preloaded sleeve */
        arm.holdPreLoadedSleeve();
        Thread.sleep(1000);

        /* go to the center of the grid */
        chassis.goToPosition(0, 1000);

        /* move to center the grid on the left */
        chassis.goToPosition(-11500, 1000);

        /* go to the center of the grid ahead */
        chassis.goToPosition(-11500, 14500); // TODO measure the y-axis

        /* raise the arm */
        arm.goToHighestTower();

        /* scores goal */
        aimAndScore(1);
    }

    /**
     * aim tower and score sleeve
     * @param direction: the direction of the targeted tower 1 for left and 2 for right
     * @return whether the process succeeded
     * */
    private boolean aimAndScore(int direction) {
        chassis.setAutoMode(false);
        robotAuxiliarySystem.startAim(direction);

        ElapsedTime timeUsed = new ElapsedTime(); timeUsed.reset();
        do {
            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();
            robotAuxiliarySystem.periodic();

            if (timeUsed.seconds() > 5) {
                chassis.setAutoMode(true);
                return false;
            }

        } while (robotAuxiliarySystem.statusCode != 0);

        chassis.setAutoMode(true);
        return robotAuxiliarySystem.isLastAimSucceeded();
    }

    /**
     * go to sector 1 if the pilot asks to
     */
    private void proceedGoToSector1() {
    }
    /**
     * go to sector 2 if the pilot asks to
     */
    private void proceedGoToSector2() {
    }
    /**
     * go to sector 3 if the pilot asks to
     */
    private void proceedGoToSector3() {
    }
}
