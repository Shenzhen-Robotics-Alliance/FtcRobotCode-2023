package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotController {

    private HardwareDriver hr;

    private Telemetry telemetry;

    private boolean slowMotionModeActivated = false;

    private boolean claw;

    private final int highpos = 700; // highest position of the arm
    private final int midpos = 450; // midpoint position of the arm
    private final int lowpos = 280; // position of the arm when grabbing stuff
    private final int gndpos = 60; // lowest position of the arm

    public RobotController (HardwareDriver hr, Telemetry telemetry) {
        this.hr = hr;
        this.telemetry = telemetry;
        this.claw = false;
        hr.claw.setPosition(0.6);
    }

    public void switchMode() {
        slowMotionModeActivated = ! slowMotionModeActivated; // activate or deactivate slow motion
    }

    public void toHighArmPosition() {
        elevateArm(highpos);
        telemetry.addData("going to top_pos", highpos);
    }

    public void toMidArmPosition() {
        elevateArm(highpos);
        telemetry.addData("going to mid_pos", midpos);
    }

    public void toLowArmPosition() {
        elevateArm(highpos);
        telemetry.addData("going to low_pos", lowpos);
    }

    public void toGroundArmPosition() {
        elevateArm(highpos);
        telemetry.addData("going to gnd_pos", gndpos);
    }

    public void open_closeClaw() {
        if(claw) closeClaw();
        else openClaw();
    }

    public void openClaw() {
        claw = true;
        hr.claw.setPosition(.35);
    }

    public void closeClaw() {
        claw = false;
        hr.claw.setPosition(.61);
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
