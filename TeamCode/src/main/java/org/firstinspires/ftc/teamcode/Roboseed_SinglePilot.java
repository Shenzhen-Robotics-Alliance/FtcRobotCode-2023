package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.ChassisModule;
import org.firstinspires.ftc.teamcode.Robot.ComputerVisionAUX;
import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;
import org.firstinspires.ftc.teamcode.Robot.ControllingMethods;

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
    private HardwareDriver hardwareDriver = new HardwareDriver();
    private ControllingMethods controllingMethods;
    private ChassisModule chassisModule;

    //Key Delay settings
    private ElapsedTime PreviousElevatorActivation = new ElapsedTime(); // the time elapsed after the last time the arm is elevated
    private ElapsedTime PreviousClawActivation = new ElapsedTime(); // the time elapsed after the last time the claw is moved
    boolean slowMotionActivated = false; // if the slow-motion mode is activated

    @Override
    public void runOpMode() throws InterruptedException {
        // config motors

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


        controllingMethods = new ControllingMethods(hardwareDriver, telemetry);
        chassisModule = new ChassisModule(gamepad1, hardwareDriver);
        ComputerVisionAUX computerVisionAUX = new ComputerVisionAUX(hardwareMap);

        Thread chassisThread = new Thread(chassisModule);
        chassisThread.start(); // start an independent thread to run chassis module

        computerVisionAUX.test(); // run the test

        telemetry.update(); // update the debug console

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
//        currentState = State.TRAJECTORY_1;
//        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) { // main loop
            telemetry.addData("This is the loop", "------------------------------");


            //auto constraint for machine protecting
            /*
            if (hardwareRobot.armLift.item.getCurrentPosition() > 900 && autoHoldFlag) {
                autoHoldFlag = false;
                if(!gamepad2.x)
                    hardwareRobot.claw.holdTheBowl();
            }

            if (hardwareRobot.armLift.item.getCurrentPosition() < 900 && !autoHoldFlag) {
                autoHoldFlag = true;
                if(!gamepad2.x)
                    hardwareRobot.claw.setBowlReady();
            }
             */


            //global key action in all mode

            //global claw
            if (gamepad1.right_bumper & PreviousClawActivation.seconds() > 0.2) {
                controllingMethods.open_closeClaw();
                PreviousClawActivation.reset();
            }

            if (gamepad1.y) {
                controllingMethods.toHighArmPosition();
            }
            if (gamepad1.x) {
                controllingMethods.toMidArmPosition();
            }
            if (gamepad1.b) {
                controllingMethods.toLowArmPosition();
            }
            if (gamepad1.a) {
                controllingMethods.toGroundArmPosition();
            }
            telemetry.addData("going to pos", 0);
            if (gamepad1.right_trigger>0.2) {
                controllingMethods.toLowArmPosition();
                chassisModule.pause();
                // TODO aim the target automatically using computer vision
                chassisModule.resume();
                controllingMethods.closeClaw();
                controllingMethods.toMidArmPosition();
            }

            if (gamepad1.right_stick_y < -0.5 & PreviousElevatorActivation.seconds() > .2) { // the elevator cannot be immediately activated until 0.3 seconds after the last activation
                System.out.println("RA");
                controllingMethods.raiseArm();
                PreviousElevatorActivation.reset();
            } else if (gamepad1.right_stick_y > 0.5 & PreviousElevatorActivation.seconds() > .2) {
                System.out.println("LA");
                controllingMethods.lowerArm();
                PreviousElevatorActivation.reset();
            }

            if (PreviousElevatorActivation.seconds() > 5) {
                controllingMethods.deactivateArm(); // deactivate when no use for 5 seconds so that the motors don't overheat
            }

            telemetry.update();
        }
    }
}



