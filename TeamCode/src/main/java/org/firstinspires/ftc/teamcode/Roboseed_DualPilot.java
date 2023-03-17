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
 * tele-operation program for two pilots to control
 * support single pilot still, but enable dual pilot mode if the second gamepad asks to plug in
 * TODO fit the program with the robot modules plugin (first priority)
 * TODO write "RobotPositionCalculator.java" and navigate with the encoders(to sense the chang in position) and imu (to sense the current direction and know where the robot is moving towards)
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
@TeleOp(name = "ManualControlMode_v2.0_DualPilot")
public class Roboseed_DualPilot extends LinearOpMode {
    /** the interface that connects the robot's hardware */
    private final HardwareDriver hardwareDriver = new HardwareDriver();

    /** whether the program will switch to slow motion mode automatically when using the arm */
    private final boolean PreviousSlowMotionModeAutoActivation = false;

    /** whether dual-piloting mode is activated or not */
    private boolean dualPilotActivated;

    /** connect to the robot modules */
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
        /* configure the ports for all the hardware's */
        this.configureRobot();

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

        /** pass the dependent modules to the arm module */
        HashMap<String, RobotModule> armModuleDependentModules = new HashMap<>(1);
        armModuleDependentModules.put("robotChassis", robotChassis);
        /** pass the hardware ports to the arm module */
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        armModuleDependentInstances.put("initialControllerPad", gamepad1);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);

        /* TODO write the above to pass the dependencies and ports all the modules */
        imuReader = new IMUReader(hardwareMap);
        imuReader.calibrateIMU();
        autoStageRobotChassis = new AutoStageRobotChassis(hardwareDriver, hardwareMap, fieldNavigation, imuReader);
        // autoStageChassisModule.initRobotChassis(); // to gather encoder data for auto stage

        /* telemetry.addLine("robotCurrentPosition(Camera)");
        telemetry.addLine("robotCurrentPosition(Encoder)");
        telemetry.addLine("robotCurrentRotation(Encoder)");
        telemetry.addLine("robotCurrentPosition(IMU)"); */

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
            fieldNavigation.terminate();
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
            runLoop();
        } fieldNavigation.terminate(); autoStageRobotChassis.terminate(); // stop the chassis and navigation modules after the op mode is put to stop
    }

    /**
     * the periodic function that is called in every each loop of the program
     *
     * @throws InterruptedException: when the operation mode is interrupted by the system
     */
    private void runLoop() throws InterruptedException {
        /** calls the periodic function of the modules TODO put the modules in a map and go through tem to run per */
        robotChassis.periodic();
        arm.periodic();


        /** switch between the two control modes if asked to */
        /* switch to dual pilot mode if the second pilot asks to take over */
        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            /* update the controller pad of arm module */
            arm.updateDependentInstances("controllerPad", gamepad2);
            /* shake the game pad to remind the pilots */
            gamepad2.rumble(500);
        }

        /* switch back to single pilot mode if the first pilot asks to take over the arms */
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            /* update the controller pad of arm module */
            arm.updateDependentInstances("controllerPad", gamepad1);
            /* shake the game pad to remind the pilots */
            gamepad1.rumble(500);
        }


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

