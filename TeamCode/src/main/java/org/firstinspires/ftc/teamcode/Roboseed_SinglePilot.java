package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.HardwareDriver;
import org.firstinspires.ftc.teamcode.Robot.RobotController;

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
    HardwareDriver hr = new HardwareDriver();
    RobotController robotController = new RobotController(hr, telemetry);

    //Key Delay settings
    private ElapsedTime PreviousModeButtonActivation = new ElapsedTime(); // the time elapsed after the last time the "mode" button is pressed
    private ElapsedTime PreviousElevatorActivation = new ElapsedTime(); // the elasped after the last time the arm is elevated
    double upspeed = 0.6; // speed when raising the arm
    double downspeed = 0.4; // speed when lowering the arm

    boolean slowMotionActivated = false; // if the slow-motion mode is activated

    @Override
    public void runOpMode() throws InterruptedException {
        // config motors

        hr.leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        hr.leftRear = hardwareMap.get(DcMotorEx.class, "leftrear");
        hr.rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
        hr.rightRear = hardwareMap.get(DcMotorEx.class, "rightrear");

        hr.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hr.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        hr.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hr.claw = hardwareMap.get(Servo.class, "tipperhopper");

        hr.lift_left = hardwareMap.get(DcMotorEx.class, "lifter");
        hr.lift_right = hardwareMap.get(DcMotorEx.class, "lifter_right");

        hr.lift_left.setDirection(DcMotorSimple.Direction.REVERSE);

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
            if (gamepad1.right_bumper) {
                robotController.open_closeClaw();
            }

            if (gamepad1.y) {
                robotController.toHighArmPosition();
            }
            if (gamepad1.x) {
                robotController.toMidArmPosition();
            }
            if (gamepad1.b) {
                robotController.toLowArmPosition();
            }
            if (gamepad1.a) {
                robotController.toGroundArmPosition();
            }
            telemetry.addData("going to pos", 0);
            if (gamepad1.right_trigger>0.2) {
                robotController.toLowArmPosition();
                // TODO aim the target automatically using computer vision
                robotController.closeClaw();
                robotController.toMidArmPosition();
            }

            if (gamepad1.right_stick_y > 0.5) robotController.raiseArm();
            else if (gamepad1.right_stick_y < -0.5) robotController.lowerArm();

            double forward = -gamepad1.left_stick_y;
            // note here is the FTC field axis, and moreover, take the opposite number to charge the car
            double rotation = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(forward) < 0.05) {
                forward = 0;
            }
            if (Math.abs(turn) < 0.05) {
                turn = 0;
            }
            if (Math.abs(rotation) < 0.05) {
                rotation = 0;
            }
            forward = Math.signum(forward) * forward * forward;
            rotation = Math.signum(rotation) * rotation * rotation;
            turn = Math.signum(turn) * turn * turn;

            forward = Range.clip(forward, -1, 1);
            rotation = Range.clip(rotation, -1, 1);
            turn = Range.clip(turn, -1, 1);


            if (slowMotionActivated) {
                forward *= 0.4;
                rotation *= 0.4;
                turn *= 0.25;
            }
            double[] speed = {
                    forward + turn + rotation,
                    forward + turn - rotation,
                    forward - turn - rotation,
                    forward - turn + rotation
            };

            hr.leftFront.setPower(speed[0]);
            hr.leftRear.setPower(speed[1]);
            hr.rightFront.setPower(speed[2]);
            hr.rightRear.setPower(speed[3]);

            if (gamepad1.dpad_down && PreviousModeButtonActivation.seconds() > 0.3) { // when control mode button is pressed, and hasn't been pressed in the last 0.3 seconds
                robotController.switchMode(); // switch control mode
                PreviousModeButtonActivation.reset();
            }
            telemetry.update();
        }
    }
}



