package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.Mini1024EncoderReader;
import org.firstinspires.ftc.teamcode.RobotModules.PilotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.IMUReader;
import org.firstinspires.ftc.teamcode.RobotModules.RobotAuxiliarySystem;
import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Roboseed_SinglePilot.java
 *
 * tele-operation program for two pilots to control
 * support single pilot still, but enable dual pilot mode if the second gamepad asks to plug in
 * TODO write "RobotPositionCalculator_v2_todo.java" and navigate with the encoders(to sense the chang in position) and imu (to sense the current direction and know where the robot is moving towards)
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
@TeleOp(name = "ManualControlMode_v2.0_DualPilot")
public class Roboseed_DualPilot extends LinearOpMode {
    /** the interface that connects the robot's hardware */
    private final HardwareDriver hardwareDriver = new HardwareDriver();
    private ChassisDriver chassisDriver;

    /** whether the program will switch to slow motion mode automatically when using the arm */
    private final boolean PreviousSlowMotionModeAutoActivation = false;

    /** whether dual-piloting mode is activated or not */
    private boolean dualPilotActivated;

    /** to measure the delay of the program */
    private final ElapsedTime programDelay = new ElapsedTime();

    /** connect to the robot modules */
    private Arm arm;
    private PilotChassis pilotChassis;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis autoStageRobotChassis;
    private IMUReader imuReader;
    private Mini1024EncoderReader encoderReader;
    private RobotPositionCalculator positionCalculator;
    private RobotAuxiliarySystem robotAuxiliarySystem;

    /**
     * the main entry of the robot's program during manual stage
     *
     * @throws InterruptedException: when the operation mode is interrupted by the system
     */
    @Override
    public void runOpMode() throws InterruptedException {
        /* configure the ports for all the hardware's */
        this.configureRobot();

        /** pass the hardware ports to the encoder reader module */
        HashMap<String, RobotModule> encoderReaderDependentModules = null;
        HashMap<String, Object> encoderReaderDependentInstances = new HashMap<>(1);
        /* get the instances of the encoders from hardware map */
//        encoderReaderDependentInstances.put("encoder-1-instance", hardwareMap.get(DcMotorEx.class, "vertical-encoder-left"));
//        encoderReaderDependentInstances.put("encoder-2-instance", hardwareMap.get(DcMotorEx.class, "vertical-encoder-right"));
//        encoderReaderDependentInstances.put("encoder-3-instance", hardwareMap.get(DcMotorEx.class, "horizontal-encoder"));
        /* no enough ports, use the encoder ports of the driving motors instead */
        encoderReaderDependentInstances.put("encoder-1-instance", hardwareDriver.leftFront);
        encoderReaderDependentInstances.put("encoder-2-instance", hardwareDriver.rightFront);
        encoderReaderDependentInstances.put("encoder-3-instance", hardwareDriver.leftRear);
        encoderReader = new Mini1024EncoderReader();
        encoderReader.init(encoderReaderDependentModules, encoderReaderDependentInstances);

        /** pass the encoder reader to the temporary position calculator */
        HashMap<String, RobotModule> positionCalculatorDependentModules = new HashMap<>(1);
        HashMap<String, Object> positionCalculatorDependentInstances = null;
        positionCalculatorDependentModules.put("encoderReader", encoderReader);
        this.positionCalculator = new RobotPositionCalculator();
        this.positionCalculator.init(positionCalculatorDependentModules, positionCalculatorDependentInstances);

        /** pass the hardware ports to the robot chassis */
        this.chassisDriver = new ChassisDriver(hardwareDriver, this.positionCalculator);
        HashMap<String, RobotModule> robotChassisDependentModules = new HashMap<>(1);
        /* the module that computes the robot's current position and motion */
        robotChassisDependentModules.put("positionCalculator", positionCalculator);
        HashMap<String, Object> robotChassisDependentInstances = new HashMap<>(1);
        /* give the first pilot's controller pad as the initial controller pad for robot's movement to the chassis module */
        robotChassisDependentInstances.put("initialControllerPad", gamepad1);
        /* give the connection to the hardware to the module */
        robotChassisDependentInstances.put("chassisDriver", this.chassisDriver);
        /* give the back up imu module of the extension hub to the chassis module*/
        robotChassisDependentInstances.put("imu", hardwareMap.get(IMU.class, "imu2"));
        pilotChassis = new PilotChassis();
        pilotChassis.init(robotChassisDependentModules, robotChassisDependentInstances);

        /** pass the dependent modules to the arm module */
        HashMap<String, RobotModule> armModuleDependentModules = new HashMap<>(1);
        armModuleDependentModules.put("robotChassis", pilotChassis);
        /** pass the hardware ports to the arm module */
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        armModuleDependentInstances.put("initialControllerPad", gamepad1);
        arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances);

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
        autoStageRobotChassis = new AutoStageRobotChassis();
        autoStageRobotChassis.init(autoStageRobotChassisDependentModules, autoStageRobotChassisDependentInstances);

        /** the RAS */
        ColorDistanceSensor color = new ColorDistanceSensor(hardwareMap, 1);
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");
        HashMap<String, RobotModule> robotAuxiliarySystemDependentModules = new HashMap<>(1);
        HashMap<String, Object> robotAuxiliarySystemDependentInstances = new HashMap<>(1);
        robotAuxiliarySystemDependentModules.put("positionCalculator", this.positionCalculator);
        robotAuxiliarySystemDependentInstances.put("colorDistanceSensor", color);
        robotAuxiliarySystemDependentInstances.put("tofDistanceSensor", distance);
        robotAuxiliarySystemDependentInstances.put("chassisDriver", chassisDriver);
        robotAuxiliarySystemDependentModules.put("arm", arm);
        this.robotAuxiliarySystem = new RobotAuxiliarySystem();
        robotAuxiliarySystem.init(robotAuxiliarySystemDependentModules, robotAuxiliarySystemDependentInstances, this, true);


        /* telemetry.addLine("robotCurrentPosition(Camera)");
        telemetry.addLine("robotCurrentPosition(Encoder)");
        telemetry.addLine("robotCurrentRotation(Encoder)");
        telemetry.addLine("robotCurrentPosition(IMU)"); */

        if (isStopRequested()) return;

        autoStageRobotChassis.calibrateEncoder();
        // imuReader.calibrateIMU();

        /* add telemetry line, TODO:use robot-debug-bridge to do it */
        telemetry.addLine("<--start-up complete-->");
        telemetry.update();

        waitForStart();
        telemetry.addLine("position");
        telemetry.addLine("rotation");
        telemetry.addLine("program-delay");

        while (opModeIsActive() && !isStopRequested()) { // main loop
            telemetry.addData("This is the loop", "------------------------------");
            runLoop();
            telemetry.update();
        }
    }

    /**
     * the periodic function that is called in every each loop of the program
     *
     * @throws InterruptedException: when the operation mode is interrupted by the system
     */
    private void runLoop() throws InterruptedException {
        telemetry.addData("program-delay", programDelay.seconds()*1000);
        programDelay.reset();

        /** calls the periodic function of the modules TODO put the modules in a map and go through in every run loop */
        // ElapsedTime elapsedTime = new ElapsedTime();
        // elapsedTime.reset();
        pilotChassis.periodic();
        // System.out.println("<--chassis module delay: " + elapsedTime.seconds()*1000 + "-->");
        // elapsedTime.reset();
        arm.periodic();
        // System.out.println("<--arm module delay: " + elapsedTime.seconds()*1000 + "-->");
        // elapsedTime.reset();
        // fieldNavigation.periodic();
        // System.out.println("<--visual navigation module delay: " + elapsedTime.seconds()*1000 + "-->");
        // elapsedTime.reset();
        // imuReader.periodic();
        // System.out.println("<--imu reader module delay: " + elapsedTime.seconds()*1000 + "-->");
        // elapsedTime.reset();
        encoderReader.periodic();
        // System.out.println("<--encoder reader module delay: " + elapsedTime.seconds()*1000 + "-->");
        // elapsedTime.reset();
        positionCalculator.periodic();
        robotAuxiliarySystem.periodic();
        // System.out.println("<--position calculator module delay: " + elapsedTime.seconds()*1000 + "-->");



        /** switch between the two control modes if asked to */
        /* switch to dual pilot mode if the second pilot asks to take over */
        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            /* update the controller pad of arm module */
            arm.updateDependentInstances("controllerPad", gamepad2);
            /* shake the game pad to remind the pilots */
            gamepad2.rumble(500);
        }

        if (gamepad1.left_trigger > 0.5) robotAuxiliarySystem.startAim(1);
        else if (gamepad1.right_trigger > 0.5) robotAuxiliarySystem.startAim(2);

        /* switch back to single pilot mode if the first pilot asks to take over the arms */
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            /* update the controller pad of arm module */
            arm.updateDependentInstances("controllerPad", gamepad1);
            /* shake the game pad to remind the pilots */
            gamepad1.rumble(500);
        }

        telemetry.addData("position", positionCalculator.getRobotPosition()[0] + ", " + positionCalculator.getRobotPosition()[1]);
        // System.out.println("arm position code: " + arm.getArmStatusCode());
        telemetry.addData("rotation", Math.toDegrees(positionCalculator.getRobotRotation()));

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

