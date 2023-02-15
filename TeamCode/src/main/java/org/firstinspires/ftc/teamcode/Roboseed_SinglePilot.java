package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.Robot.ChassisModule;
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionFieldNavigation_v2;
import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;
import org.firstinspires.ftc.teamcode.Robot.ArmControllingMethods;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@TeleOp(name = "ManualControlMode_v1.0_SinglePilot")
//@Disabled //updated with some functions to all mode, intake
public class Roboseed_SinglePilot extends LinearOpMode {
    private final HardwareDriver hardwareDriver = new HardwareDriver();

    //Key Delay settings
    private final ElapsedTime PreviousElevatorActivation = new ElapsedTime(); // the time elapsed after the last time the arm is elevated
    private final ElapsedTime PreviousClawActivation = new ElapsedTime(); // the time elapsed after the last time the claw is moved
    private final ElapsedTime PreviousGrepActivation = new ElapsedTime();
    boolean slowMotionActivated = false; // if the slow-motion mode is activated

    private ArmControllingMethods armControllingMethods;
    private ChassisModule chassisModule;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;

    private AutoStageChassisModule autoStageChassisModule;

    @Override
    public void runOpMode() throws InterruptedException {
        this.configureRobot();

        armControllingMethods = new ArmControllingMethods(hardwareDriver, telemetry);
        chassisModule = new ChassisModule(gamepad1, hardwareDriver, hardwareMap.get(IMU.class, "imu2")); // back up imu module from extension hub
        fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);

        autoStageChassisModule = new AutoStageChassisModule(hardwareDriver, hardwareMap);
        // autoStageChassisModule.initRobotChassis(); // to gather encoder data for auto stage

        telemetry.addLine("robotCurrentPosition(Camera)");
        telemetry.addLine("robotCurrentPosition(Encoder)");


        waitForStart();


        Thread chassisThread = new Thread(chassisModule);
        chassisThread.start(); // start an independent thread to run chassis module

        Thread navigationThread = new Thread(fieldNavigation);
        navigationThread.start();

        // computerVisionAUX.test(); // run the test

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
//        currentState = State.TRAJECTORY_1;
//        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) { // main loop
            telemetry.addData("This is the loop", "------------------------------");
            runLoop(armControllingMethods, chassisModule);
        } chassisModule.terminate(); fieldNavigation.terminate(); // stop the chassis and navigation modules after the op mode is put to stop
    }

    private void runLoop(ArmControllingMethods armControllingMethods, ChassisModule chassisModule) throws InterruptedException {
        double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
        String cameraPositionString = String.valueOf(robotCurrentPosition[0]) + " " + String.valueOf(robotCurrentPosition[1]) + " " + String.valueOf(robotCurrentPosition[2]);
        telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);

        double[] encoderPosition = autoStageChassisModule.getEncoderPosition();
        String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
        telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);

        if (gamepad1.right_bumper) armControllingMethods.closeClaw();
        else if (gamepad1.left_bumper) armControllingMethods.openClaw();

        if (gamepad1.y) {
            armControllingMethods.toHighArmPosition();
        }
        if (gamepad1.x) {
            armControllingMethods.toMidArmPosition();
        }
        if (gamepad1.b) {
            armControllingMethods.toLowArmPosition();
        }
        if (gamepad1.a) {
            armControllingMethods.toGroundArmPosition();
        }
        telemetry.addData("going to pos", 0);
        if (gamepad1.right_trigger>0.2 & PreviousGrepActivation.seconds() > .3) {
            PreviousGrepActivation.reset();
            armControllingMethods.openClaw();
            armControllingMethods.deactivateArm();
            chassisModule.pause();
            // TODO aim the target automatically using computer vision
            chassisModule.resume();
            armControllingMethods.closeClaw();
            Thread.sleep(300);
            armControllingMethods.toMidArmPosition();
        }

        if (gamepad1.left_stick_y < -0.5 & PreviousElevatorActivation.seconds() > .2) { // the elevator cannot be immediately activated until 0.2 seconds after the last activation
            System.out.println("RA");
            armControllingMethods.raiseArm();
            PreviousElevatorActivation.reset();
        } else if (gamepad1.left_stick_y > 0.5 & PreviousElevatorActivation.seconds() > .2) {
            System.out.println("LA");
            armControllingMethods.lowerArm();
            PreviousElevatorActivation.reset();
        }

        if (PreviousElevatorActivation.seconds() > 30 & chassisModule.getLastMovementTime() > 30 & PreviousClawActivation.seconds() > 30) { // no operation after 30s
            hardwareDriver.lift_left.setPower(0);
            hardwareDriver.lift_left.setPower(0);
            System.exit(0);
        } if (PreviousElevatorActivation.seconds() > 5 & armControllingMethods.getClaw()) {
            System.out.println("saving battery...");
            armControllingMethods.deactivateArm(); // deactivate when no use for 5 seconds so that the motors don't overheat
            PreviousElevatorActivation.reset(); // so that it does not proceed deactivate all the time
        }

        telemetry.update();
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

