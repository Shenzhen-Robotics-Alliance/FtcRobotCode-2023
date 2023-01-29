package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roboseed_testcar.HardwareRobot;

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
@TeleOp(name = "种萝卜_正式_手动_20230113_V1")
//@Disabled //updated with some functions to all mode, intake
public class TeleOpLinear_Roboseed_V2 extends LinearOpMode {
    HardwareRobot hr = new HardwareRobot();
    //Key Delay settings
    private ElapsedTime keyDelay = new ElapsedTime();
    double upspeed = 0.6;
    double downspeed = 0.4;


    int highpos = 700;
    int midpos = 450;
    int lowpos = 280;
    int groundpos = 60;

    boolean isSlowMove = false;

    @Override
    public void runOpMode() throws InterruptedException {

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

        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
//        currentState = State.TRAJECTORY_1;
//        drive.followTrajectoryAsync(trajectory1);
        while (opModeIsActive() && !isStopRequested()) {
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
                hr.claw.setPosition(0.35);//open
            }
            if (gamepad1.left_bumper) {
                hr.claw.setPosition(0.61);//close
            }

            if (gamepad1.y) {
                hr.lift_left.setTargetPosition(highpos);
                hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_left.setPower(upspeed);
                hr.lift_right.setTargetPosition(highpos);
                hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_right.setPower(upspeed);
                telemetry.addData("going to toppos", highpos);
            }
            if (gamepad1.x) {
                hr.lift_left.setTargetPosition(midpos);
                hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_left.setPower(upspeed);
                hr.lift_right.setTargetPosition(midpos);
                hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_right.setPower(upspeed);
                telemetry.addData("going to midpos", midpos);
            }
            if (gamepad1.b) {
                hr.lift_left.setTargetPosition(lowpos);
                hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_left.setPower(upspeed);
                hr.lift_right.setTargetPosition(lowpos);
                hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_right.setPower(upspeed);
                telemetry.addData("going to lowpos", lowpos);
            }
            if (gamepad1.a) {
                hr.lift_left.setTargetPosition(groundpos);
                hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_left.setPower(downspeed);
                hr.lift_right.setTargetPosition(groundpos);
                hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_right.setPower(downspeed);
                telemetry.addData("going to lowpos", lowpos);
            }
            telemetry.addData("going to pos", 0);
            if (gamepad1.right_trigger>0) {
                hr.lift_left.setTargetPosition(0);
                hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_left.setPower(downspeed);
                hr.lift_right.setTargetPosition(0);
                hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hr.lift_right.setPower(downspeed);
                telemetry.addData("going to lowpos", lowpos);
            }

            double forward = -gamepad1.left_stick_y;
            // note here is the FTC field axis, and moreover, take the opposite number to charge the car
            double translation = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(forward) < 0.05) {
                forward = 0;
            }
            if (Math.abs(turn) < 0.05) {
                turn = 0;
            }
            if (Math.abs(translation) < 0.05) {
                translation = 0;
            }
            forward = Math.signum(forward) * forward * forward;
            translation = Math.signum(translation) * translation * translation;
            turn = Math.signum(turn) * turn * turn;

            forward = Range.clip(forward, -1, 1);
            translation = Range.clip(translation, -1, 1);
            turn = Range.clip(turn, -1, 1);


            if (isSlowMove) {
                forward *= 0.4;
                translation *= 0.4;
                turn *= 0.25;
            }
            double[] speed = {
                    forward + turn + translation,
                    forward + turn - translation,
                    forward - turn - translation,
                    forward - turn + translation
            };

            hr.leftFront.setPower(speed[0]);
            hr.leftRear.setPower(speed[1]);
            hr.rightFront.setPower(speed[2]);
            hr.rightRear.setPower(speed[3]);

            if (gamepad1.dpad_down && keyDelay.seconds() > 0.3) {//debug switch
                isSlowMove = !isSlowMove;
                keyDelay.reset();
            }
            telemetry.update();
        }
    }
}



