package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ArmControllingMethods;
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
    private ArmControllingMethods armControllingMethods;

    @Override
    public void runOpMode() throws InterruptedException {
        configureRobot();
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        Thread fieldNavigationThread = new Thread(fieldNavigation);

        chassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap, fieldNavigation);
        chassisModule.initRobotChassis();
        elapsedTime.reset();

        armControllingMethods = new ArmControllingMethods(hardwareDriver, telemetry);

        Thread terminationListenerThread = new Thread(new Runnable() { @Override public void run() {
                while (!isStopRequested() && opModeIsActive()) Thread.yield();
                fieldNavigation.terminate();
                chassisModule.terminate();
                terminationFlag = true;
            }
        }); terminationListenerThread.start();

        Thread robotStatusMonitoringThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
                String cameraPositionString = robotCurrentPosition[0] + " " + robotCurrentPosition[1] + " " + robotCurrentPosition[2];
                telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

                double[] encoderPosition = chassisModule.getEncoderPosition();
                String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
                telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

                telemetry.update();
            }
        });

        waitForStart();

        fieldNavigationThread.start();
        terminationListenerThread.start();
        robotStatusMonitoringThread.start();

        
        // start of the auto stage scripts
        proceedAutoStageInstructions();


        // end of the program
        fieldNavigation.terminate(); chassisModule.terminate();
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

    private void proceedAutoStageInstructions() {
        // grab the preloaded sleeve
        armControllingMethods.deactivateArm();
        armControllingMethods.closeClaw();

        // go to the center of the grid (75, 130), in reference to the red side team
        chassisModule.setRobotPosition(95, 130);
        // check for termination in each step

        // line up horizontally with the place where the sleeves are stored
        chassisModule.setRobotPosition(95, 1500);
        // check for termination in each step

        // turn the robot to the goal
        chassisModule.setRobotRotation(270);

        // go to the goal
        if (fieldNavigation.checkNavigationSignsVisability())
            chassisModule.setRobotPositionWithVisualNavigation(0, 0);
        else chassisModule.moveRobotWithEncoder(0, 0); // TODO set the positions to make the position line up and stick close with the goal one step away

        // raise the arm
        armControllingMethods.toHighArmPosition();

        // go forward a step
        chassisModule.moveRobotWithEncoder(0, 100); // TODO set the position so that the sleeve goes right to the goal

        // place the preloaded goal
        armControllingMethods.deactivateArm();
        armControllingMethods.openClaw();
        chassisModule.moveRobotWithEncoder(0, -100); // step back from the goal

        // turn the robot to the stick
        chassisModule.setRobotRotation(Math.toRadians(90));

        // precise navigation to the sleeves using visual guidance
        if (fieldNavigation.checkNavigationSignsVisability()) // if the navigation signs are available
            chassisModule.setRobotPositionWithVisualNavigation(-1050, 1260); // visual guidance to the sleeves
        else chassisModule.setRobotPosition(0, 0); // other wise, dive to the sleeves using encoders TODO set this position to make robot very close to the sleeves

        // raise the arms
        armControllingMethods.openClaw();
        armControllingMethods.toLoadingArmPosition();

        // go to the sleeves
        chassisModule.moveRobotWithEncoder(0, 200);
        // grab a sleeve and raise it up
        armControllingMethods.closeClaw();
        armControllingMethods.toMidArmPosition();
        // step back and drag it on the ground
        chassisModule.moveRobotWithEncoder(0, -200);
        armControllingMethods.toGroundArmPosition();

        // turn the robot to the goal
        chassisModule.setRobotRotation(270);

        // go to the goal
        if (fieldNavigation.checkNavigationSignsVisability())
            chassisModule.setRobotPositionWithVisualNavigation(0, 0);
        else chassisModule.moveRobotWithEncoder(0, 0); // TODO set the positions to make the position line up and stick close with the goal one step away

        // raise the arm
        armControllingMethods.toHighArmPosition();

        // go forward a step
        chassisModule.moveRobotWithEncoder(0, 100); // TODO set the position so that the sleeve goes right to the goal

        // place the sleeve
        armControllingMethods.deactivateArm();
        armControllingMethods.openClaw();
        chassisModule.moveRobotWithEncoder(0, -100); // step back from the goal

        // TODO move to parking position according to the driver input to pretend having visual recognizing
    }
}
