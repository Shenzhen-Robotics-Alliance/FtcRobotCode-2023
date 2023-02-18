package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.AutoStageChassisModule;
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;

/*
 * the robot starts in the corner of the field.
 * first, the robot moves out of the parking spot and rotates 90 degree to face the navigation marks,
 * the robot moves to position(according to camera) -1022, -782
 *
 * */

@Autonomous(name = "robot test runner")
public class Roboseed_Test extends LinearOpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    HardwareDriver hardwareDriver = new HardwareDriver();

    AutoStageChassisModule autoStageChassisModule;
    ComputerVisionFieldNavigation_v2 fieldNavigation;

    @Override
    public void runOpMode() throws InterruptedException {
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        autoStageChassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap, fieldNavigation);
        this.configureRobot();
        autoStageChassisModule.initRobotChassis();
        waitForStart();

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
            while (!isStopRequested() && opModeIsActive()) Thread.yield();
            autoStageChassisModule.terminate();
        }
        }); terminationListenerThread.start();


        // autoStageChassisModule.setRobotPositionWithVisualNavigation(-1200, -1000);

        autoStageChassisModule.setRobotRotation(Math.toRadians(90));
        while (opModeIsActive() && !isStopRequested()) {
            final double[] robotPosition = autoStageChassisModule.getEncoderPosition();
            final double robotRotation = autoStageChassisModule.getEncoderRotation();
            telemetry.addData("robotXPosition", robotPosition[0]);
            telemetry.addData("robotYPosition", robotPosition[1]);
            telemetry.addData("robotRotation", robotRotation);
            telemetry.addData("robotRotationIMU", autoStageChassisModule.getImuYaw());
            telemetry.addData("RobotPosition(vision):", fieldNavigation.getRobotPosition()[0] + ","  + fieldNavigation.getRobotPosition()[0]);
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
