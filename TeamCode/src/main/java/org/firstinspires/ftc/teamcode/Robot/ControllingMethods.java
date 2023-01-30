package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ControllingMethods {

    private HardwareDriver hr;

    private Telemetry telemetry;

    private boolean claw;

    private short arm;

    private final int highPos = 700; // highest position of the arm
    private final int midPos = 450; // midpoint position of the arm
    private final int lowPos = 280; // position of the arm when grabbing stuff
    private final int gndPos = 60; // lowest position of the arm
    private final double armInclineSpeed = 0.6;
    private final double armDeclineSpeed = 0.4;

    public ControllingMethods(HardwareDriver hr, Telemetry telemetry) {
        this.hr = hr;
        this.telemetry = telemetry;
        this.claw = false;
        hr.claw.setPosition(0.6);
        toGroundArmPosition();
    }

    public void lowerArm() {
        switch (arm) {
            case 2: toLowArmPosition(); break;
            case 3: toMidArmPosition(); break;
            // cannot reverse the two, otherwise, case 2 will be triggered immediately after case 3 is executed
        }
    }

    public void raiseArm() {
        switch (arm) {
            case 2: toHighArmPosition(); break;
            case 0: case 1: toMidArmPosition(); break;
            // cannot reverse the two cases, otherwise, case 2 will be triggered after case 0 or 1
        }
    }

    public void toHighArmPosition() {
        elevateArm(highPos);
        arm = 3;
        telemetry.addData("going to top_pos", highPos);
    }

    public void toMidArmPosition() {
        elevateArm(midPos);
        arm = 2;
        telemetry.addData("going to mid_pos", midPos);
    }

    public void toLowArmPosition() {
        elevateArm(lowPos);
        arm = 1;
        telemetry.addData("going to low_pos", lowPos);
    }

    public void toGroundArmPosition() {
        elevateArm(gndPos);
        arm = 0;
        telemetry.addData("going to gnd_pos", gndPos);
    }

    public void open_closeClaw() {
        if(claw) closeClaw();
        else openClaw();
    }

    public void openClaw() {
        claw = true;
        hr.claw.setPosition(.35); // open grabber
        while (Math.abs(hr.claw.getPosition() - .35) > .05) Thread.yield(); // wait until the movement is finished, accept any inaccuracy below 5%
    }

    public void closeClaw() {
        claw = false;
        hr.claw.setPosition(.61); // close grabber
        while (Math.abs(hr.claw.getPosition() - .61) > .05) Thread.yield();
    }


    private void elevateArm(int position) {
        hr.lift_left.setTargetPosition(position);
        hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (position > hr.lift_left.getCurrentPosition()) {
            hr.lift_left.setPower(armInclineSpeed);
            hr.lift_right.setPower(armInclineSpeed);
        } else {
            hr.lift_left.setPower(armDeclineSpeed);
            hr.lift_right.setPower(armDeclineSpeed);
        }
        hr.lift_right.setTargetPosition(position);
        hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hr.lift_right.setPower(position);

        while (Math.abs(hr.lift_left.getCurrentPosition()-position) > 5 | Math.abs(hr.lift_right.getCurrentPosition()-position) > 5) Thread.yield(); // wait until the movement is finished, accept any deviation below ±5
    }

    public void deactivateArm() {
        hr.lift_left.setPower(0);
    }
}
