/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Roboseed_SinglePilot.java
 *
 * tele-operation program with one pilot
 * TODO add dual pilot control mode
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Robot.AutoStageChassisModule;
import org.firstinspires.ftc.teamcode.Robot.ChassisModule;
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;
import org.firstinspires.ftc.teamcode.Robot.ArmControllingMethods;
import org.firstinspires.ftc.teamcode.Robot.IMUReader;

@TeleOp(name = "ManualControlMode_v1.0_SinglePilot")
public class Roboseed_SinglePilot extends LinearOpMode {
    /* the interface that connects the robot's hardware */
    private final HardwareDriver hardwareDriver = new HardwareDriver();

    /* variables that record the time after pressing a button, so that the button is not activated over and over */
    private final ElapsedTime PreviousElevatorActivation = new ElapsedTime();
    private final ElapsedTime PreviousClawActivation = new ElapsedTime();
    private final ElapsedTime PreviousGrepActivation = new ElapsedTime();
    private boolean PreviousSlowMotionModeAutoActivation = false;

    /* connect to the modules */
    private ArmControllingMethods armControllingMethods;
    private ChassisModule chassisModule;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageChassisModule autoStageChassisModule;
    private IMUReader imuReader;

    /*
    * the main entry of the robot's program during manual stage
    *
    * @param Nah
    * @return Nah
    * @throws InterruptedException: when the operation mode is interrupted by the system
    * */
    @Override
    public void runOpMode() throws InterruptedException {
        this.configureRobot();

        armControllingMethods = new ArmControllingMethods(hardwareDriver, telemetry);
        chassisModule = new ChassisModule(gamepad1, hardwareDriver, hardwareMap.get(IMU.class, "imu2")); // back up imu module from extension hub
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);

        imuReader = new IMUReader(hardwareMap);
        imuReader.calibrateIMU();
        autoStageChassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap, fieldNavigation, imuReader);
        // autoStageChassisModule.initRobotChassis(); // to gather encoder data for auto stage

        telemetry.addLine("robotCurrentPosition(Camera)");
        telemetry.addLine("robotCurrentPosition(Encoder)");
        telemetry.addLine("robotCurrentRotation(Encoder)");
        telemetry.addLine("robotCurrentPosition(IMU)");

        Thread chassisThread = new Thread(chassisModule);
        chassisThread.start(); // start an independent thread to run chassis module

        Thread navigationThread = new Thread(fieldNavigation);
        navigationThread.start();

        Thread imuReaderThread = new Thread(imuReader);
        imuReaderThread.start();

        // computerVisionAUX.test(); // run the test

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
//        currentState = State.TRAJECTORY_1;
//        drive.followTrajectoryAsync(trajectory1);

        Thread robotStatusMonitoringThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive() && !isStopRequested()) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) { throw new RuntimeException(e); }
                    System.out.println("monitoring thread running");
                    double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
                    String cameraPositionString = String.valueOf(robotCurrentPosition[0]) + " " + String.valueOf(robotCurrentPosition[1]) + " " + String.valueOf(robotCurrentPosition[2]);
                    telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

                    double[] encoderPosition = autoStageChassisModule.getEncoderPosition();
                    String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
                    telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

                    double encoderRotation = autoStageChassisModule.getEncoderRotation();
                    telemetry.addData("robotCurrentRotation(Encoder)", encoderRotation);

                    telemetry.update();
                }
            }
        }); // robotStatusMonitoringThread.start();

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
            while (!isStopRequested() && opModeIsActive()) Thread.yield();
            fieldNavigation.terminate();
            chassisModule.terminate();
            autoStageChassisModule.terminate();
            imuReader.terminate();
            System.exit(0);
        }
        }); terminationListenerThread.start();

        waitForStart();
        sleep(1000);

        autoStageChassisModule.calibrateEncoder();
        imuReader.calibrateIMU();

        while (opModeIsActive() && !isStopRequested()) { // main loop
            telemetry.addData("This is the loop", "------------------------------");
            runLoop(armControllingMethods, chassisModule);
        } chassisModule.terminate(); fieldNavigation.terminate(); autoStageChassisModule.terminate(); // stop the chassis and navigation modules after the op mode is put to stop
    }

    /*
     * the periodic function that is called in every each loop of the program
     *
     * @param Nah
     * @return Nah
     * @throws InterruptedException: when the operation mode is interrupted by the system
     * */
    private void runLoop(ArmControllingMethods armControllingMethods, ChassisModule chassisModule) throws InterruptedException {
        double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
        String cameraPositionString = String.valueOf(robotCurrentPosition[0]) + " " + String.valueOf(robotCurrentPosition[1]) + " " + String.valueOf(robotCurrentPosition[2]);
        telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

        double[] encoderPosition = autoStageChassisModule.getEncoderPosition();
        String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
        telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

        double[] IMUPosition = imuReader.getIMUPosition();
        String IMUPositionString = String.valueOf(IMUPosition[0]) + "," + String.valueOf(IMUPosition[1]);
        telemetry.addData("robotCurrentPosition(IMU)", IMUPositionString);

        telemetry.update();
        
        if (gamepad1.right_bumper) armControllingMethods.closeClaw();
        else if (gamepad1.left_bumper) armControllingMethods.openClaw();

        if (gamepad1.y) {
            armControllingMethods.toHighArmPosition();
        }
        if (gamepad1.x) {
            armControllingMethods.toMidArmPosition();
        }
        if (gamepad1.b) {
            armControllingMethods.toLowArmPosition();
        }
        if (gamepad1.a) {
            armControllingMethods.toGroundArmPosition();
        }
        telemetry.addData("going to pos", 0);
        if (gamepad1.right_trigger>0.2 & PreviousGrepActivation.seconds() > .3) {
            PreviousGrepActivation.reset();
            armControllingMethods.openClaw();
            armControllingMethods.deactivateArm();
            chassisModule.pause();
            // TODO aim the target automatically using computer vision
            chassisModule.resume();
            armControllingMethods.closeClaw();
            Thread.sleep(300);
            armControllingMethods.toMidArmPosition();
        }

        if (gamepad1.left_stick_y < -0.8 & PreviousElevatorActivation.seconds() > .3) { // the elevator cannot be immediately activated until 0.2 seconds after the last activation
            System.out.println("RA");
            armControllingMethods.raiseArm();
            PreviousElevatorActivation.reset();
        } else if (gamepad1.left_stick_y > 0.8 & PreviousElevatorActivation.seconds() > .3) {
            System.out.println("LA");
            armControllingMethods.lowerArm();
            PreviousElevatorActivation.reset();
        }

        if (PreviousElevatorActivation.seconds() > 30 & chassisModule.getLastMovementTime() > 30 & PreviousClawActivation.seconds() > 30) { // no operation after 30s
            hardwareDriver.lift_left.setPower(0);
            hardwareDriver.lift_left.setPower(0);
            System.exit(0);
        } if (PreviousElevatorActivation.seconds() > 5 & armControllingMethods.getClaw()) {
            System.out.println("saving battery...");
            armControllingMethods.deactivateArm(); // deactivate when no use for 5 seconds so that the motors don't overheat
            PreviousElevatorActivation.reset(); // so that it does not proceed deactivate all the time
        }

        // control slow motion automatically
        if (armControllingMethods.getArmStatus()) chassisModule.setSlowMotionModeActivationSwitch(true);
        else chassisModule.setSlowMotionModeActivationSwitch(false);
        telemetry.update();
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
}

