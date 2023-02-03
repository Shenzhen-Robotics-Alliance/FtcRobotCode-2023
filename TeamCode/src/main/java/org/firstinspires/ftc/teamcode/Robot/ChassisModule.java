package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ChassisModule implements Runnable { // controls the moving of the robot
    private final Gamepad gamepad;
    private final HardwareDriver driver;
    private final IMU imu;

    private boolean slowMotionModeActivationSwitch;
    private boolean groundNavigatingModeActivationSwitch;

    private final ElapsedTime PreviousMotionModeButtonActivation;
    private final ElapsedTime PreviousNavigationModeButtonActivation;
    private final ElapsedTime lastMovement;

    private boolean paused;

    public ChassisModule(Gamepad gamepad, HardwareDriver driver, IMU imu) {
        this.gamepad = gamepad;
        this.driver = driver;
        this.imu = imu;

        // the rotation of the control hub, in reference to the chassis, see https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html
        final double xRotation = 0;
        final double yRotation = 145.64;
        final double zRotation = 0;
        // init the imu
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        this.slowMotionModeActivationSwitch = false;
        this.PreviousMotionModeButtonActivation = new ElapsedTime();
        this.PreviousNavigationModeButtonActivation = new ElapsedTime();
        this.lastMovement = new ElapsedTime();
        this.paused = false;
    }

    @Override
    public void run() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        double facing;
        double velocityYAW;

        while (true) {
            while (paused) Thread.yield();
            double yAxleMotion = linearMap(-gamepad.right_stick_y); // the left stick is reversed to match the vehicle
            double xAxleMotion = linearMap(gamepad.right_stick_x);
            double rotationalMotion = linearMap(gamepad.left_stick_x);

            if (groundNavigatingModeActivationSwitch) { // when the pilot chooses to navigate according to the ground
                // get the rotation and angular velocity of the robot from imu
                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

                // get the facing, and the angular velocity in YAW axle, of the robot
                facing = orientation.getYaw(AngleUnit.RADIANS);
                velocityYAW = angularVelocity.zRotationRate;

                // TODO correct xAxelMotion and yAxelMotion using the IMU
            }

            if (yAxleMotion != 0 | xAxleMotion != 0 | rotationalMotion != 0) lastMovement.reset();

            yAxleMotion = Math.copySign(yAxleMotion * yAxleMotion, yAxleMotion);
            xAxleMotion = Math.copySign(xAxleMotion * xAxleMotion, xAxleMotion);
            rotationalMotion = Math.copySign(rotationalMotion * rotationalMotion, rotationalMotion); // square the axis, keep the sign

            yAxleMotion = Range.clip(yAxleMotion, -1, 1);
            xAxleMotion = Range.clip(xAxleMotion, -1, 1);
            rotationalMotion = Range.clip(rotationalMotion, -1, 1);

            // control the Mecanum wheel
            driver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
            driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
            driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
            driver.rightRear.setPower(yAxleMotion + rotationalMotion + xAxleMotion);

            if (gamepad.dpad_down & PreviousMotionModeButtonActivation.seconds() > 0.5) { // when control mode button is pressed, and hasn't been pressed in the last 0.3 seconds
                slowMotionModeActivationSwitch = !slowMotionModeActivationSwitch; // activate or deactivate slow motion
                PreviousMotionModeButtonActivation.reset();
            } if(gamepad.dpad_up & PreviousNavigationModeButtonActivation.seconds() > 0.5) {
                groundNavigatingModeActivationSwitch = !groundNavigatingModeActivationSwitch;
                PreviousNavigationModeButtonActivation.reset();
            } if (gamepad.dpad_right) { // debug the imu by resetting the heading
                imu.resetYaw();
            }
        }
    }

    private double linearMap(double value) {
        if (slowMotionModeActivationSwitch) { // when slow motion activated
            if (value > 0) return linearMap(0.05, 1, 0, 0.4, value);
            return linearMap(-0.05, -1, 0, -0.4, value); // change the speed range to -0.4~0.4
        }
        if (value > 0) return linearMap(0.1, 1, 0, 1, value);
        return linearMap(-0.1, -1, 0, -1, value); // map the axle of the stick to make sure inputs below 10% are ignored
    }
    private double linearMap(double fromFloor, double fromCeiling, double toFloor, double toCeiling, double value){
        value -= fromFloor;
        value *= (toCeiling-toFloor) / (fromCeiling - fromFloor);
        value += toFloor;
        return value;
    }

    public void pause() {
        this.paused = true;
    }
    public void resume() {
        this.paused = false;
    }

    public double getLastMovementTime() {
        return lastMovement.seconds();
    }
}