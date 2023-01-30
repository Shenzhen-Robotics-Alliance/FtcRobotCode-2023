package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotController {

    private HardwareDriver hr;

    private Telemetry telemetry;

    private boolean slowMotionModeActivated = false;

    private boolean claw;

    private short arm;

    private final int highPos = 700; // highest position of the arm
    private final int midPos = 450; // midpoint position of the arm
    private final int lowPos = 280; // position of the arm when grabbing stuff
    private final int gndPos = 60; // lowest position of the arm

    public RobotController (HardwareDriver hr, Telemetry telemetry) {
        this.hr = hr;
        this.telemetry = telemetry;
        this.claw = false;
        hr.claw.setPosition(0.6);
        toGroundArmPosition();
    }

    public void switchMode() {
        slowMotionModeActivated = ! slowMotionModeActivated; // activate or deactivate slow motion
    }

    public void lowerArm() {
        switch (arm) {
            case 3: toMidArmPosition(); break;
            case 2: toLowArmPosition(); break;
        }
    }

    public void raiseArm() {
        switch (arm) {
            case 0: case 1:toMidArmPosition(); break;
            case 2: toHighArmPosition(); break;
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
    }

    public void closeClaw() {
        claw = false;
        hr.claw.setPosition(.61); // close grabber
    }


    private void elevateArm(int position) {
        hr.lift_left.setTargetPosition(position);
        hr.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hr.lift_left.setPower(position);
        hr.lift_right.setTargetPosition(position);
        hr.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hr.lift_right.setPower(position);
    }
}
