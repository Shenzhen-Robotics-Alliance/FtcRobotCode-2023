/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: IMUReader.java
 *
 * connects to the imu module and analyze the received data
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */

package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

public class IMUReader extends RobotModule {
    // The IMU sensor object
    BNO055IMU imu;
    private boolean terminated;
    private boolean paused;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    double headingCorrectionBias;
    private double[] accelerationBias = new double[2]; //  to correct the effect of gravity
    private double[] velocity = new double[2];
    private double[] position = new double[2];
    private ElapsedTime dt = new ElapsedTime();
    final double imuHeadingCorrectionFactor = 1;


    public IMUReader(){
        /* call to super method */
        super("imuReader");
    }

    /**
     * initialize the imu reader module
     * @param dependentModules null should be given, as this module does not any other modules as dependency
     * @param dependentInstances this module needs the following instances(pass them in the form of hashmap):
     *                          "hardwareMap" : HardwareMap, the connection to the ports of the robot;
     */
    @Override
    public void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException {
        /* throw out an error if the map of the instances needed is given as an empty map,
        * or if the hardware map isn't given */
        if (dependentInstances.isEmpty()) throw new NullPointerException(
                "an empty set of dependent instance given to the module<<" + this.getModuleName() + ">> which requires at least one instance(s)"
        );
        if (!dependentInstances.containsKey("hardwareMap")) throw new NullPointerException(
                "required dependency <<" + "hardwareMap" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the connection to the hardware from the params */
        HardwareMap hardwareMap = (HardwareMap) dependentInstances.get("hardwareMap");

        /* set the params for the imu */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /* instantiate the imu */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        dt.reset();
        // imu.stopAccelerationIntegration();
    }

    /**
     * update an instance used in the module
     *
     * @Deprecated the imu reader does not need to change any instance after initialization
     */
    @Override @Deprecated public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    @Override
    public void periodic() {
        updateIMUStatus();
        double dX, dY;
        dX = dY = 0;
        // update the imu position
        updateIMUStatus();
        if (Math.abs(getRobotXAcceleration()) > 0.06) {
            // calculate the current position, using trapezoid secondary integral of acceleration
            dX = dt.seconds() * velocity[0];
            velocity[0] += dt.seconds() * getRobotXAcceleration();
            dX = dt.seconds() * velocity[0];
            dX /= 2;
        } else {
            // assume it's motioning with constant velocity
            // dX = dt.seconds() * velocity[0];
        } if(Math.abs(getRobotYAcceleration()) > 0.06) {
            dY = dt.seconds() * velocity[1];
            velocity[1] += dt.seconds() * getRobotYAcceleration();
            dY = dt.seconds() * velocity[1];
            dY /= 2;
        } else {
            // dY = dt.seconds() * velocity[1];
        }
        position[0] += dX;
        position[1] += dY;
        dt.reset();
    }

    public void calibrateIMU() {
        updateIMUStatus();
        headingCorrectionBias = -getRobotHeading();
        Acceleration gravity = imu.getGravity();
        accelerationBias[0] = gravity.xAccel;
        accelerationBias[1] = gravity.yAccel;
        position[0] = position[1] = velocity[0] = velocity[1] = 0; // init the variables
        System.out.print("acceleration bias:"); System.out.print(accelerationBias[0]); System.out.print(" "); System.out.println(accelerationBias[1]);
    }
    public void updateIMUStatus() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    public double getRobotHeading() {
        // TODO fix bugs here
        return angles.firstAngle * imuHeadingCorrectionFactor + headingCorrectionBias;
    }
    public double getRobotRoll() {
        return angles.secondAngle;
    }
    public double getRobotPitch() {
        return angles.thirdAngle;
    }

    public double getRobotXAcceleration() {
        return gravity.xAccel - accelerationBias[0];
    }
    public double getRobotYAcceleration() {
        return gravity.yAccel - accelerationBias[1];
    }
    public String getGravitation() {
        return gravity.toString();
    }

    public double[] getIMUPosition() {
        return position;
    }

    public void pause() {
        this.paused = true;
    }

    public void resume() {
        this.paused = false;
    }
}
