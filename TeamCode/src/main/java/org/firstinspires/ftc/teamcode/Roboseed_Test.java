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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModules.AutoStageRobotChassis;
import org.firstinspires.ftc.teamcode.RobotModules.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.RobotModules.Mini1024EncoderReader;

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

    AutoStageRobotChassis autoStageRobotChassis;
    ComputerVisionFieldNavigation_v2 fieldNavigation;

    @Override
    public void runOpMode() throws InterruptedException {
        Mini1024EncoderReader encoderReader;
        configureRobot();
        HashMap<String, RobotModule> encoderReaderDependentModules = null;
        HashMap<String, Object> encoderReaderDependentInstances = new HashMap<>(1);
        /* no enough ports, use the encoder ports of the driving motors instead */
        encoderReaderDependentInstances.put("encoder-1-instance", hardwareDriver.leftFront);
        encoderReaderDependentInstances.put("encoder-2-instance", hardwareDriver.rightFront);
        encoderReaderDependentInstances.put("encoder-3-instance", hardwareDriver.leftRear);
        encoderReader = new Mini1024EncoderReader();
        encoderReader.init(encoderReaderDependentModules, encoderReaderDependentInstances);

        telemetry.addLine("encoder1Value");
        telemetry.addLine("encoder2Value");
        telemetry.addLine("encoder3Value");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            encoderReader.periodic();
            /* test the encoder */
            telemetry.addData("encoder1Value", encoderReader.getEncoderPosition(1));
            telemetry.addData("encoder2Value", encoderReader.getEncoderPosition(2));
            telemetry.addData("encoder3Value", encoderReader.getEncoderPosition(3));
            telemetry.update();
        }
    }

    private void configureRobot() {
        try {
            hardwareDriver.leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
            hardwareDriver.leftRear = hardwareMap.get(DcMotorEx.class, "leftrear");
            hardwareDriver.rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
            hardwareDriver.rightRear = hardwareMap.get(DcMotorEx.class, "rightrear");
        } catch (NullPointerException e) {
            e.printStackTrace();
            System.exit(-1);
        }

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
