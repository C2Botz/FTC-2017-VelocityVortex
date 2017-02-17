package org.firstinspires.ftc.teamcode;

/**
 * Created by Fibonacci on 12/12/16.
 */

import android.media.CamcorderProfile;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRobot
{
    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;

    public ColorSensor colorSensor = null;

    public Servo leftArmServo   = null;//Servo Port 1
    public Servo rightArmServo   = null;//Servo Port 2

    public AllianceColor alliance = AllianceColor.NULL;
    public AutoMode autoMode = AutoMode.NULL;
    public boolean isLedOn = false;
    public boolean isSensing = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    public double frontSpeedMultipler = 1.25;
    public double rightStrafeMultipler = 1.1;

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor   = hwMap.dcMotor.get("frontLeftMotor");
        frontRightMotor  = hwMap.dcMotor.get("frontRightMotor");
        rearLeftMotor  = hwMap.dcMotor.get("rearLeftMotor");
        rearRightMotor  = hwMap.dcMotor.get("rearRightMotor");

        colorSensor = hwMap.colorSensor.get("beaconColorSensor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        resetEncoders();

        leftArmServo = hwMap.servo.get("leftArmServo");
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        leftArmServo.setPosition(.5);


        rightArmServo = hwMap.servo.get("rightArmServo");
        rightArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setPosition(.5);

        switch(FtcRobotControllerActivity.allianceColorString){

            case "RED":
                alliance = AllianceColor.RED;
                break;

            case "BLUE":
                alliance = AllianceColor.BLUE;
                break;

            case "NULL":
                alliance = AllianceColor.NULL;
                break;

        }

        switch(FtcRobotControllerActivity.autoModeString){

            case "BEACON":
                autoMode = AutoMode.BEACON;
                break;

            case "CORNER":
                autoMode = AutoMode.CORNER;
                break;

            case "CAPBALL":
                autoMode = AutoMode.CAPBALL;
                break;

            case "NULL":
                autoMode = AutoMode.NULL;
                break;

        }
        
    }

    public void resetEncoders(){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveAtSpeed(float leftSpeed, float rightSpeed){
        frontRightMotor.setPower(rightSpeed);
        rearRightMotor.setPower(rightSpeed);
        frontLeftMotor.setPower(leftSpeed);
        rearLeftMotor.setPower(leftSpeed);
    }

    public void strafeLeft(float power){
        frontRightMotor.setPower(power*frontSpeedMultipler/**rightStrafeMultipler*/);
        rearRightMotor.setPower(-power/**rightStrafeMultipler*/);
        frontLeftMotor.setPower(-power*frontSpeedMultipler);
        rearLeftMotor.setPower(power);
    }

    public void strafeRight(float power){
        frontRightMotor.setPower(-power*frontSpeedMultipler/**rightStrafeMultipler*/);
        rearRightMotor.setPower(power/**rightStrafeMultipler*/);
        frontLeftMotor.setPower(power*frontSpeedMultipler);
        rearLeftMotor.setPower(-power);
    }

    public void rotateRightSideForTime(float speed, float timeSeconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds() < timeSeconds) {
            frontRightMotor.setPower(speed);
            rearRightMotor.setPower(speed);
        }
        stop();
    }
    public void rotateLeftSideForTime(float speed, float timeSeconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds() < timeSeconds) {
            frontLeftMotor.setPower(speed);
            rearLeftMotor.setPower(speed);
        }
        stop();
    }

    public void strafeForTime(Direction direction, float power, float timeSeconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < timeSeconds){
            if(direction == Direction.LEFT){
                strafeLeft(power);
            }else if(direction == Direction.RIGHT){
                strafeRight(power);
            }
        }
        stop();

    }

    public void stop(){
        frontRightMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }


    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void enableColorSensorLED(boolean enable){
        isLedOn = enable;
        colorSensor.enableLed(enable);
    }




}

