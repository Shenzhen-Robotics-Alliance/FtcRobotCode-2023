package org.firstinspires.ftc.teamcode.Drivers;

public class ChassisDriver {
    private HardwareDriver hardwareDriver;
    private double xAxleMotion = 0;
    private double yAxleMotion = 0;
    private double rotationalMotion = 0;
    private boolean RASActivation = false;

    public ChassisDriver(HardwareDriver hardwareDriver) {
        this.hardwareDriver = hardwareDriver;
    }

    public void setRobotTranslationalMotion(double xAxleMotion, double yAxleMotion) {
        this.xAxleMotion = xAxleMotion;
        this.yAxleMotion = yAxleMotion;
    }

    public void setRotationalMotion(double rotationalMotion) {
        this.rotationalMotion = rotationalMotion;
    }

    public void pilotInterruption() {
        RASActivation = false;
    }

    public void newAimStarted() {
        RASActivation = false;
    }

    public boolean isRASActivated() {
        return RASActivation;
    }

    private void sendCommandsToMotors() {
        hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
    }
}
