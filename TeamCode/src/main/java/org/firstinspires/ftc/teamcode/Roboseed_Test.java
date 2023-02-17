package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Robot.AutoStageChassisModule;
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

    @Override
    public void runOpMode() throws InterruptedException {
        autoStageChassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap);
        this.configureRobot();
        autoStageChassisModule.initRobotChassis();
        waitForStart();

        autoStageChassisModule.testRobtMotion(0, 0, 0.2);

        while (opModeIsActive() && !isStopRequested()) {
            final double[] robotPosition = autoStageChassisModule.getEncoderPosition();
            final double robotRotation = autoStageChassisModule.getEncoderRotation();
            telemetry.addData("robotXPosition", robotPosition[0]);
            telemetry.addData("robotYPosition", robotPosition[1]);
            telemetry.addData("robotRotation", robotRotation);
            telemetry.addData("robotRotationIMU", autoStageChassisModule.getImuYaw());
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
