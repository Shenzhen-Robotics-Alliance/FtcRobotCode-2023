package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;
import org.firstinspires.ftc.teamcode.Robot.IMUReader;

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

    @Override
    public void runOpMode() throws InterruptedException {
        this.configureRobot();
        waitForStart();

        hardwareDriver.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareDriver.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareDriver.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareDriver.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardwareDriver.leftFront.setPower(0.2);
        hardwareDriver.leftRear.setPower(0.2);
        hardwareDriver.rightFront.setPower(0.2);
        hardwareDriver.rightRear.setPower(0.2);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("leftFront", hardwareDriver.leftFront.getCurrentPosition());
            telemetry.addData("leftRear", hardwareDriver.leftRear.getCurrentPosition());
            telemetry.addData("rightFront", hardwareDriver.rightFront.getCurrentPosition());
            telemetry.addData("rightRear", hardwareDriver.rightRear.getCurrentPosition());
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

        hardwareDriver.claw = hardwareMap.get(Servo.class, "tipperhopper");

        hardwareDriver.lift_left = hardwareMap.get(DcMotorEx.class, "lifter");
        hardwareDriver.lift_right = hardwareMap.get(DcMotorEx.class, "lifter_right");

        hardwareDriver.lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
