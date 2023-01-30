package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ChassisModule implements Runnable { // controls the moving of the robot
    private Gamepad gamepad;
    private HardwareDriver driver;

    private boolean slowMotionActivationSwitch;
    private ElapsedTime PreviousModeButtonActivation;

    public ChassisModule(Gamepad gamepad, HardwareDriver driver) {
        this.gamepad = gamepad;
        this.driver = driver;
        this.slowMotionActivationSwitch = false;
        this.PreviousModeButtonActivation = new ElapsedTime();
    }

    @Override
    public void run() {
        double yAxleMotion = linearMap(-gamepad.left_stick_y); // the left stick is reversed to match the vehicle
        double xAxleMotion = linearMap(gamepad.left_stick_x);
        double rotationalMotion = linearMap(gamepad.right_stick_x);

        yAxleMotion = Math.copySign(yAxleMotion * yAxleMotion, yAxleMotion);
        xAxleMotion = Math.copySign(xAxleMotion * xAxleMotion, xAxleMotion);
        rotationalMotion = Math.copySign(rotationalMotion * rotationalMotion, rotationalMotion); // square the axis, keep the sign

        yAxleMotion = Range.clip(yAxleMotion, -1, 1);
        xAxleMotion = Range.clip(xAxleMotion, -1 ,1);
        rotationalMotion = Range.clip(rotationalMotion, -1, 1);

        // control the Mecanum wheel
        driver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion + rotationalMotion + xAxleMotion);

        if (gamepad.dpad_down & PreviousModeButtonActivation.seconds() > 0.5) { // when control mode button is pressed, and hasn't been pressed in the last 0.3 seconds
            slowMotionActivationSwitch = ! slowMotionActivationSwitch; // activate or deactivate slow motion
            PreviousModeButtonActivation.reset();
        }
    }

    private double linearMap(double value) {
        if (slowMotionActivationSwitch) { // when slow motion activated
            if (value > 0) return linearMap(0.1, 1, 0, 0.4, value);
            return linearMap(-0.1, -1, 0, -0.4, value); // change the speed range to -0.4~0.4
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
}