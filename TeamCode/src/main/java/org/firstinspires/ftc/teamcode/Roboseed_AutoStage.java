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

@Autonomous(name = "AutoStateProgram_v1.0")
public class Roboseed_AutoStage extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private boolean terminationFlag;

    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private AutoStageChassisModule chassisModule;

    @Override
    public void runOpMode() throws InterruptedException {
        configureRobot();
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        Thread fieldNavigationThread = new Thread(fieldNavigation);
        chassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap, fieldNavigation);

        waitForStart();

        fieldNavigationThread.start();
        chassisModule.initRobotChassis();
        elapsedTime.reset();

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
                while (!isStopRequested() && opModeIsActive()) Thread.yield();
                fieldNavigation.terminate();
                chassisModule.terminate();
                terminationFlag = true;
            }
        }); terminationListenerThread.start();

        Thread robotStatusMonitoringThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive() && !isStopRequested()) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) { throw new RuntimeException(e); }
                    // System.out.println("monitoring thread running");
                    double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
                    String cameraPositionString = String.valueOf(robotCurrentPosition[0]) + " " + String.valueOf(robotCurrentPosition[1]) + " " + String.valueOf(robotCurrentPosition[2]);
                    telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

                    double[] encoderPosition = chassisModule.getEncoderPosition();
                    String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
                    telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

                    telemetry.update();
                }
            }
        }); robotStatusMonitoringThread.start();



        // start of the auto stage scripts
        // go to the center of the grid (280, 3220), in reference to the red side team
        chassisModule.setRobotPosition(0, 500);
        chassisModule.setRobotPosition(280, 500);
        chassisModule.setRobotPosition(280, 3220);

        if (terminationFlag) return; // check for termination in each step

        // end of the program
        fieldNavigation.terminate();
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
