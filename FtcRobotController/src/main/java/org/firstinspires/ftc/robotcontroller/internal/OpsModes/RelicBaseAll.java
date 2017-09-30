package org.firstinspires.ftc.robotcontroller.internal.OpsModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Henri on 9/24/2017.
 */
public abstract class RelicBaseAll extends LinearOpMode{

    RelicRecoveryHardware robot = new RelicRecoveryHardware();


    // Telemetry
    public void say(String text1, String text2) {  //Enter two strings and it will be posted as telemetry
        telemetry.addData(text1, text2);
        telemetry.update();
    }

    public void dataLabel(String text1, double data) {  //Use a string and double to post inputs/outputs as telemetry
        telemetry.addData(text1, "%" + data);
    }

    //Setup

    public void setupDone() {   // Signifies setup is done and has the game wait for START button
        say("Say", "It is working and you loaded the package.");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }


    //Timing

    public void pause() {
        sleep(750);
        say("Pausing", "");
    }


    //Sensors

    public void gyroCalibrate() {       // Calibrates gyro DO NOT TOUCH WHILE CALIBRATING!!!!!
        robot.gyroSensor.calibrate();
        while(robot.gyroSensor.isCalibrating()) {
            say("GYRO CALIBRATING", "DO NOT TOUCH!!!!");
            sleep(10);
        }
        say("Gyro Calibrated", "Good Luck");
    }


    //Power Drive
    public void fDrive(double power) {        // Drives forward and prints power
        robot.leftBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(power);
        dataLabel("Power: ", power);
    }

    public void bDrive(double power) {          // Drives backwards and prints power
        robot.leftBackMotor.setPower(-power);
        robot.leftFrontMotor.setPower(-power);
        robot.rightBackMotor.setPower(-power);
        robot.rightFrontMotor.setPower(-power);
        dataLabel("Power: ", power);
    }

    public void lDrive(double power) {         // Drives left and prints power
        robot.leftBackMotor.setPower(-power);
        robot.leftFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(-power);
        dataLabel("Power: ", power);
    }

    public void rDrive(double power) {            // Drives right and prints power
        robot.leftBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(-power);
        robot.rightBackMotor.setPower(-power);
        robot.rightFrontMotor.setPower(power);
        dataLabel("Power: ", power);
    }

    public void wheelStop() {  // Stops drive train and prints power(0)
        fDrive(0);
    }
}
