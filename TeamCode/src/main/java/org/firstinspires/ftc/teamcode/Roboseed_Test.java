/*
* Copyright © 2023 SCCSC-Robotics-Club
* FileName: Roboseed_Test.java
*
* an autonomous program to run some tests of the robot
*
* @Author 四只爱写代码の猫
* @Date 2023.2.27
* @Version v0.1.0
* @Deprecated this program is for test only
* */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModules.Arm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageArm;
import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis_tmp;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Mini1024EncoderReader;
import org.firstinspires.ftc.teamcode.RobotModules.RobotAuxiliarySystem;
import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;

import java.util.HashMap;

/*
 * the robot starts in the corner of the field.
 * first, the robot moves out of the parking spot and rotates 90 degree to face the navigation marks,
 * the robot moves to position(according to camera) -1022, -782
 * */

@Autonomous(name = "robot_test_runner")
public class Roboseed_Test extends LinearOpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    HardwareDriver hardwareDriver = new HardwareDriver();

    ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageRobotChassis_tmp robotChassis;
    private RobotAuxiliarySystem robotAuxiliarySystem;

    private ColorSensor color;
    private DistanceSensor distanceSensor;

    public boolean programAlive;

    public void runOpMode_disabled() throws InterruptedException {
        configureRobot();

         // Wait for the Play button to be pressed
         waitForStart();

        hardwareDriver.lift_left.setPower(0.5);

         // While the Op Mode is running, update the telemetry values.
         while (opModeIsActive()) {
             // telemetry.addData("color sensor result", color.alpha());
             telemetry.addData("left_lifter:", hardwareDriver.lift_left.getCurrentPosition());
             telemetry.update();
         }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        configureRobot();

        /** pass the hardware ports to the arm module */
        HashMap<String, RobotModule> armModuleDependentModules = new HashMap<>(1);
        HashMap<String, Object> armModuleDependentInstances = new HashMap<>(1);
        armModuleDependentInstances.put("hardwareDriver", hardwareDriver);
        armModuleDependentInstances.put("initialControllerPad", new Gamepad());
        Arm arm = new Arm();
        arm.init(armModuleDependentModules, armModuleDependentInstances, false);

        /** the temporary arm module to operate the arm during auto stage */
        AutoStageArm autoStageArm = new AutoStageArm(arm);

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
        RobotPositionCalculator positionCalculator = new RobotPositionCalculator();
        positionCalculator.init(positionCalculatorDependentModules, positionCalculatorDependentInstances);

        /** the temporary chassis module */
        this.robotChassis = new AutoStageRobotChassis_tmp(hardwareMap, hardwareDriver, positionCalculator);


        /** the RAS */
        ColorDistanceSensor color = new ColorDistanceSensor(hardwareMap, 1);
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");
        ChassisDriver chassisDriver = new ChassisDriver(hardwareDriver, positionCalculator);
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
        HashMap<String, RobotModule> robotAuxiliarySystemDependentModules = new HashMap<>(1);
        HashMap<String, Object> robotAuxiliarySystemDependentInstances = new HashMap<>(1);
        robotAuxiliarySystemDependentModules.put("positionCalculator", positionCalculator);
        robotAuxiliarySystemDependentInstances.put("colorDistanceSensor", color);
        robotAuxiliarySystemDependentInstances.put("tofDistanceSensor", distance);
        robotAuxiliarySystemDependentInstances.put("chassisDriver", chassisDriver);
        robotAuxiliarySystemDependentModules.put("arm", arm);
        this.robotAuxiliarySystem = new RobotAuxiliarySystem();
        robotAuxiliarySystem.init(robotAuxiliarySystemDependentModules, robotAuxiliarySystemDependentInstances, this);

        waitForStart();
        ElapsedTime dt = new ElapsedTime();

        arm.closeClaw();
        Thread.sleep(500);
        arm.toHighArmPosition();
        while (arm.getArmStatusCode() != 0) {
            System.out.println(hardwareDriver.lift_left.getCurrentPosition());
            arm.periodic();
        }
        robotAuxiliarySystem.startAim(2);
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("sensor reading:", robotAuxiliarySystem.tofDistanceSensorReading);
            // System.out.println("delay:" + dt.seconds()*1000);

            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();
            robotAuxiliarySystem.periodic();
            arm.periodic();


//            positionCalculator.forceUpdateEncoderValue();
//            positionCalculator.periodic();
//            robotAuxiliarySystem.periodic();
//            arm.periodic();
//            arm.closeClaw();

//            positionCalculator.forceUpdateEncoderValue();
//            positionCalculator.periodic();
//            chassisDriver.sendCommandsToMotors();

            // telemetry.addData("distance sensor result: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("position:", String.format("%.2f",positionCalculator.getRobotPosition()[0]) + "," + String.format("%.2f",positionCalculator.getRobotPosition()[1]));
            telemetry.addData("rotation:", Math.toDegrees(positionCalculator.getRobotRotation()));
            telemetry.update();
        }
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

        try {
            hardwareDriver.claw = hardwareMap.get(Servo.class, "tipperhopper");

            hardwareDriver.lift_left = hardwareMap.get(DcMotorEx.class, "lifter");
            hardwareDriver.lift_right = hardwareMap.get(DcMotorEx.class, "lifter_right");

            hardwareDriver.lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (NullPointerException e) {
            e.printStackTrace();
            System.exit(0);
        }
    }
}
