package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.RobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.IMUReader;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Roboseed_SinglePilot.java
 *
 * tele-operation program with one pilot
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
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

    private Arm arm;
    private RobotChassis robotChassis;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis autoStageRobotChassis;
    private IMUReader imuReader;

    /**
    * the main entry of the robot's program during manual stage
    *
    * @throws InterruptedException: when the operation mode is interrupted by the system
    */
    @Override
    public void runOpMode() throws InterruptedException {
        this.configureRobot();


        /** pass the hardware ports to the arm module */
        HashMap armModuleDependentModules = null;
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);


        /** pass the hardware ports to the robot chassis */
        HashMap<String, RobotModule> robotChassisDependentModules = null;
        HashMap<String, Object> robotChassisDependentInstances = new HashMap<>();
        /* give the first pilot's controller pad as the initial controller pad for robot's movement to the chassis module */
        robotChassisDependentInstances.put("initialControllerPad", gamepad1);
        /* give the connection to the hardware to the module */
        robotChassisDependentInstances.put("hardwareDriver", hardwareDriver);
        /* give the back up imu module of the extension hub to the chassis module*/
        robotChassisDependentInstances.put("imu", hardwareMap.get(IMU.class, "imu2"));
        robotChassis = new RobotChassis();
        robotChassis.init(robotChassisDependentModules, robotChassisDependentInstances);

        /** pass the hardware ports to the field navigation module */
        HashMap<String, RobotModule> fieldNavigationDependentModules = null;
        HashMap<String, Object> fieldNavigationDependentInstances = new HashMap<>(1);
        fieldNavigationDependentInstances.put("hardwareMap", hardwareMap);
        fieldNavigation = new ComputerVisionFieldNavigation_v2();
        fieldNavigation.init(fieldNavigationDependentModules, fieldNavigationDependentInstances);

        /** TODO write the above to pass the dependencies and ports all the modules */


        imuReader = new IMUReader(hardwareMap);
        imuReader.calibrateIMU();
        autoStageRobotChassis = new AutoStageRobotChassis(hardwareDriver, hardwareMap, fieldNavigation, imuReader);
        // autoStageChassisModule.initRobotChassis(); // to gather encoder data for auto stage

        telemetry.addLine("robotCurrentPosition(Camera)");
        telemetry.addLine("robotCurrentPosition(Encoder)");
        telemetry.addLine("robotCurrentRotation(Encoder)");
        telemetry.addLine("robotCurrentPosition(IMU)");

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

                    double[] encoderPosition = autoStageRobotChassis.getEncoderPosition();
                    String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
                    telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

                    double encoderRotation = autoStageRobotChassis.getEncoderRotation();
                    telemetry.addData("robotCurrentRotation(Encoder)", encoderRotation);

                    telemetry.update();
                }
            }
        }); // robotStatusMonitoringThread.start();

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
            while (!isStopRequested() && opModeIsActive()) Thread.yield();
            autoStageRobotChassis.terminate();
            imuReader.terminate();
        }
        }); terminationListenerThread.start();

        waitForStart();
        sleep(1000);

        autoStageRobotChassis.calibrateEncoder();
        imuReader.calibrateIMU();

        while (opModeIsActive() && !isStopRequested()) { // main loop
            telemetry.addData("This is the loop", "------------------------------");
            runLoop(arm, robotChassis);
        } autoStageRobotChassis.terminate(); // stop the chassis and navigation modules after the op mode is put to stop
    }

    /**
     * the periodic function that is called in every each loop of the program
     *
     * @throws InterruptedException: when the operation mode is interrupted by the system
     */
    private void runLoop(Arm arm, RobotChassis robotChassis) throws InterruptedException {
        double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
        String cameraPositionString = robotCurrentPosition[0] + " " + robotCurrentPosition[1] + " " + String.valueOf(robotCurrentPosition[2]);
        telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

        double[] encoderPosition = autoStageRobotChassis.getEncoderPosition();
        String encoderPositionString = encoderPosition[0] + "," + encoderPosition[1];
        telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

        double[] IMUPosition = imuReader.getIMUPosition();
        String IMUPositionString = IMUPosition[0] + "," + IMUPosition[1];
        telemetry.addData("robotCurrentPosition(IMU)", IMUPositionString);

        telemetry.update();
        
        if (gamepad1.right_bumper) arm.closeClaw();
        else if (gamepad1.left_bumper) arm.openClaw();

        if (gamepad1.y) {
            arm.toHighArmPosition();
        }
        if (gamepad1.x) {
            arm.toMidArmPosition();
        }
        if (gamepad1.b) {
            arm.toLowArmPosition();
        }
        if (gamepad1.a) {
            arm.toGroundArmPosition();
        }
        telemetry.addData("going to pos", 0);
        if (gamepad1.right_trigger>0.2 & PreviousGrepActivation.seconds() > .3) {
            PreviousGrepActivation.reset();
            arm.openClaw();
            arm.deactivateArm();
            // TODO aim the target automatically using computer vision
            arm.closeClaw();
            Thread.sleep(300);
            arm.toMidArmPosition();
        }

        if (gamepad1.left_stick_y < -0.8 & PreviousElevatorActivation.seconds() > .3) { // the elevator cannot be immediately activated until 0.2 seconds after the last activation
            System.out.println("RA");
            arm.raiseArm();
            PreviousElevatorActivation.reset();
        } else if (gamepad1.left_stick_y > 0.8 & PreviousElevatorActivation.seconds() > .3) {
            System.out.println("LA");
            arm.lowerArm();
            PreviousElevatorActivation.reset();
        }

        if (PreviousElevatorActivation.seconds() > 30 & robotChassis.getLastMovementTime() > 30 & PreviousClawActivation.seconds() > 30) { // no operation after 30s
            hardwareDriver.lift_left.setPower(0);
            hardwareDriver.lift_left.setPower(0);
            System.exit(0);
        } if (PreviousElevatorActivation.seconds() > 5 & arm.getClaw()) {
            System.out.println("saving battery...");
            arm.deactivateArm(); // deactivate when no use for 5 seconds so that the motors don't overheat
            PreviousElevatorActivation.reset(); // so that it does not proceed deactivate all the time
        }

        // control slow motion automatically
        if (arm.getArmIsBusy()) robotChassis.setSlowMotionModeActivationSwitch(true);
        else robotChassis.setSlowMotionModeActivationSwitch(false);
        telemetry.update();
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
}

