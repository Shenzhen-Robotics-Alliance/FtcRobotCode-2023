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
    private boolean terminated;

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
        imu.resetYaw();

        this.slowMotionModeActivationSwitch = false;
        this.PreviousMotionModeButtonActivation = new ElapsedTime();
        this.PreviousNavigationModeButtonActivation = new ElapsedTime();
        this.lastMovement = new ElapsedTime();
        this.paused = false;
        this.terminated = false;
    }

    @Override
    public void run() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        double facing;
        double velocityYAW;
        double[] correctedMotion;

        while (true) {
            while (paused) Thread.yield();
            if (terminated) break;
            double yAxleMotion = linearMap(-gamepad.right_stick_y); // the left stick is reversed to match the vehicle
            double xAxleMotion = linearMap(gamepad.right_stick_x);
            double rotationalMotion = linearMap(gamepad.left_stick_x);

            boolean movement = xAxleMotion != 0 | yAxleMotion != 0;
            if (groundNavigatingModeActivationSwitch & movement) { // when the pilot chooses to navigate according to the ground, don't apply when the robot is still
                // get the rotation and angular velocity of the robot from imu
                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

                // get the facing, and the angular velocity in YAW axle, of the robot
                facing = orientation.getYaw(AngleUnit.RADIANS);
                velocityYAW = angularVelocity.zRotationRate;


                // correct xAxelMotion and yAxelMotion using the IMU
                correctedMotion = navigateGround(xAxleMotion, yAxleMotion, -facing);
                xAxleMotion = correctedMotion[0];
                yAxleMotion = correctedMotion[1];
            }

            if (yAxleMotion != 0 | xAxleMotion != 0 | rotationalMotion != 0) lastMovement.reset();

            yAxleMotion = Range.clip(yAxleMotion, -1, 1);
            xAxleMotion = Range.clip(xAxleMotion, -1, 1);
            rotationalMotion = Range.clip(rotationalMotion, -1, 1);

            /* yAxleMotion *= -1;
            xAxleMotion *= -1;
            rotationalMotion *= -1; // flip the axles, to replace ground navigation mode temporarily */

            /* System.out.print(xAxleMotion); System.out.print("  ");
            System.out.print(yAxleMotion); System.out.print("  ");
            System.out.print(rotationalMotion); System.out.print("  ");
            System.out.println(); */


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

    private double[] navigateGround(double objectiveXMotion, double objectiveYMotion, double facing) {
        double[] correctedMotion = new double[2];

        // correct the motion
        double speed = Math.sqrt(objectiveXMotion * objectiveXMotion + objectiveYMotion * objectiveYMotion); // the magnitude of resultant velocity

        int sector; // calculate the sector of the direction we want the robot to move(in reference to the ground)
        if (objectiveXMotion > 0) {
            if (objectiveYMotion > 0) sector = 1;
            else sector = 4;
        } else if (objectiveYMotion > 0) sector = 2;
        else sector = 3;

        // System.out.print(sector); System.out.print("    "); // until here, no problem

        objectiveXMotion = Math.abs(objectiveXMotion); objectiveYMotion = Math.abs(objectiveYMotion); // take abs value, avoid errors in calculation
        double targetedVelocityArc = Math.atan(objectiveYMotion / objectiveXMotion); // find the angle of the direction we want the robot to move, but taking it down to sector 1
        switch (sector) { // transfer targetedVeloctiyArc back to the sector its taken down from
            // at sector 1, the velocity arc is itself
            case 2 :{
                targetedVelocityArc = Math.PI - targetedVelocityArc; // at sector 2, the arc needs to be flipped according to y-axis, so the actual arc will be 180deg - arc
                break;
            }
            case 3 :{
                targetedVelocityArc += Math.PI; // at sector 3, the arc needs to be pointing at the other way around, makes the actual arc be 180deg + arc
                break;
            }
            case 4 :{
                targetedVelocityArc = Math.PI * 2 - targetedVelocityArc; // at sector 4, the arc needs to be flipped according to x-axis
                break;
            }
        }

        // System.out.print(targetedVelocityArc); System.out.print("    "); // until here, no problem found

        double correctedVelocityArc = targetedVelocityArc + facing; // correct the direction of velocity according the direction of the car, the corrected velocity is now in reference to the car itself.

        // System.out.print(correctedVelocityArc); System.out.print("    "); // no problem until here

        if (correctedVelocityArc > Math.PI) {
            // if (correctedVelocityArc > 3/2*Math.PI) {sector = 4; correctedVelocityArc = Math.PI*2 - correctedVelocityArc; } // error, 3/2 will be automatically taken down to integer
            if (correctedVelocityArc > Math.PI*1.5) {sector = 4; correctedVelocityArc = Math.PI*2 - correctedVelocityArc; } // taking it down to sector 1 is the reverse of the above process
            else {sector = 3; correctedVelocityArc -= Math.PI; }
            // } else if (correctedVelocityArc > 1/2*Math.PI) {sector = 2; correctedVelocityArc = Math.PI - correctedVelocityArc; } // error, 1/2 will be automatically taken down to integer
        } else if (correctedVelocityArc > Math.PI*0.5) {sector = 2; correctedVelocityArc = Math.PI - correctedVelocityArc; }
        else {sector = 1; } // judge the sector of correctVelocityArc, store the secotr, and take correctedVelocityArc down to sector1

        // System.out.print(correctedVelocityArc); System.out.print("    "); System.out.print(sector); System.out.print("    "); // problem found, sector incorrect

        double xVelocity = 0, yVelocity = 0; // the velocity the robot needs to achieve the resultant velocity, now in refernce to itself
        xVelocity = Math.cos(correctedVelocityArc) * speed;
        yVelocity = Math.sin(correctedVelocityArc) * speed;
        switch (sector) { // transform the velocity back to the sector
            // in the first sector, velocity remains the same
            case 2 :{
                xVelocity *= -1; // in the second sector, velocity is mirrored according to y-axis
                break;
            }
            case 3 :{
                xVelocity *= -1;
                yVelocity *= -1; // in the third sector, velocity is pointing to the opposite direction
                break;
            }
            case 4:{
                yVelocity *= -1; // in the fourth sector, velocity is mirrored according to x-axis
                break;
            }
        }

        correctedMotion[0] = xVelocity;
        correctedMotion[1] = yVelocity;

        return correctedMotion;
    }
    private double linearMap(double value) {
        if (slowMotionModeActivationSwitch) { // when slow motion activated
            if (value > 0) return linearMap(0.05, 1, 0, 0.4, value);
            return linearMap(-1, -0.05, -0.4, 0, value); // change the speed range to -0.4~0.4
        } if (value > 0) return linearMap(0.1, 1, 0, 1, value);
        return linearMap(-1, -0.1, -1, 0, value); // map the axle of the stick to make sure inputs below 10% are ignored
    }
    private double linearMap(double fromFloor, double fromCeiling, double toFloor, double toCeiling, double value){
        if (value < fromFloor | value > fromCeiling) return 0;
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

    public void terminate() {
        terminated = true;
    }

    public double getLastMovementTime() {
        return lastMovement.seconds();
    }
}