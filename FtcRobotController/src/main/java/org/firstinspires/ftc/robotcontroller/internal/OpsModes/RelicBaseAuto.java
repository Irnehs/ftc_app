package org.firstinspires.ftc.robotcontroller.internal.OpsModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.internal.OpsModes.RelicRecoveryHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.OpsModes.RelicRecoveryHardware;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbotMatrix;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by Henri on 8/13/2017.
 */
public abstract class RelicBaseAuto extends RelicBaseAll{
    RelicRecoveryHardware robot           = new RelicRecoveryHardware();

//long speed;
    double brake = 30;
    double turnAccuracy = 0.5; //Accuracy in degrees for turning
    double turnCorrect = 3; //Straying accuracy in degrees for driving
    int tpr = 1120; //Ticks per revolution
    double straightCorrect; //Error of placement in inches for driving




//THE FOLLOWING THREE ARE MUTUALLY EXCLUSIVE
    
    public void targetMode() { //Sets motors to rotate for ticks(revolutions)
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        say("Motors: ", "Target");
        sleep(25);
    }

    public void encoderReset() { //Resets encoders
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        say("Encoders: ", "Reset");
        sleep(25);
    }

    public void powerDrive(){ //Sets drive train to run off power instead of ticks
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        say("Motors: ", "Power");
        sleep(25);
    }


/*targetDrive
    
    public void leftTarget(int distance, double power) {
        targetDrive();
        int ticks = tpr*(distance/circumference);
        robot.leftFrontMotor.setTargetPosition(ticks);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setTargetPosition(ticks);
        robot.leftBackMotor.setPower(power);
        dataLabel("Left Target: ", distance);
        dataLabel("Left Power: ", power);

    }
    public void rightTarget(int distance, double power) {
        targetDrive();
        int ticks = tpr*(distance/circumference);
        robot.rightFrontMotor.setTargetPosition(ticks);
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setTargetPosition(ticks);
        robot.rightBackMotor.setPower(power);
        dataLabel("Right Target: ", distance);
        dataLabel("Right Power: ", power);
    }
*/


//Driving Functions
    
    public void turn(double degrees, double speed) { //Turning with speed and degrees - +=clockwise, -=counterclockwise
        int loop = 0; // Stage counting variable
        double start = robot.gyroSensor.getHeading(); //Starting gyro direction
        double end = start+degrees; // Gyro direction where turn should end
        double endcalc = end; //End used for further calculations
        double lastPos;  //Last recorded gyro heading
        double power; //Multiplier based off angle that still needs to be turned
        double pos = start; // Current position starting with "start" for logistics
        if(end<0){end+=360;}  //If it turns past the 0 point (counterclockwise), carry over to 0 - 359 scale
        if(end>=360){end-=360;} //If it turns past/to the 0 point (clockwise), carry over to 0 - 359 scale
        say("Turn Status: ", "Calculating"); //Displays telemetry that says it is busy rotating
        sleep(100); //Take a break
        while(loop<1) { //As long
            lastPos = pos;
            pos = robot.gyroSensor.getHeading();
            double dif = pos-lastPos;
            if(Math.abs(dif) > 180) {
                endcalc = end;
            }
            if(pos<end+turnAccuracy&&pos>end-turnAccuracy){ //If it is within TurnAccuracy
                loop+=1;
            }

            power = (endcalc-pos)/brake;

            if(power>1) {
                power=1;
            }

            if(power<-1){
                power=-1;
            }

            robot.leftFrontMotor.setPower(power*speed);
            robot.leftBackMotor.setPower(power*speed);
            robot.rightFrontMotor.setPower(-power*speed);
            robot.rightBackMotor.setPower(-power*speed);

            say("Turn Status: ", "Turning");
            dataLabel("Power: ", power*speed);
            sleep(10);
        }

        encoderReset();
    }


    public void target(int distance, double power) {
        double baseDirect = robot.gyroSensor.getHeading(); //Used to set starting direction.
        while(Math.abs(distance - robot.gyroSensor.rawY() ) > straightCorrect) {//As long as the robot is less than "straightCorrect" distance away...
            fDrive(power / (distance-robot.gyroSensor.rawY()) );
            double nowD = robot.gyroSensor.getHeading(); // ...Measure the current direction of the robot.
            double dif = nowD-baseDirect; //Calculates the difference between current direction and starting direction.
            if(Math.abs(dif) > turnCorrect) { //If the robot's direction is off by more than "turnCorrect" degrees...
                turn(dif, power); //...Turn the robot to make it straight again
            }
            sleep(10); //Checks every 10 milliseconds
        }
        wheelStop(); //When it is close enough, stop
    }
}
