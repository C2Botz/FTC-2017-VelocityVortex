package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.provider.Settings;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Fibonacci on 12/12/16.
 */


@TeleOp(name="Release Teleop", group="Release Opmode")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRobot robot = new HardwareRobot();   // Use a Pushbot's hardware

    float frontSpeedMultiplier = .75f;


    @Override
    public void runOpMode() {
        float left;
        float right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Auto Alliance", robot.alliance);    //
        telemetry.update();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.

        // Set the LED in the beginning
        robot.enableColorSensorLED(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            float throttle = -gamepad1.left_stick_y;
            float direction = gamepad1.left_stick_x;
            float leftStrafe = gamepad1.left_trigger;
            float rightStrafe = gamepad1.right_trigger;

            if(gamepad1.dpad_up){
                robot.frontSpeedMultipler += .05;
            }else if(gamepad1.dpad_down){
                robot.frontSpeedMultipler -= .05;
            }

            boolean elevatorShouldMoveUp = gamepad2.dpad_up;
            boolean elevatorShouldMoveDown = gamepad2.dpad_down;

            boolean shouldSenseLeft = gamepad1.left_bumper;
            boolean shouldSenseRight = gamepad1.right_bumper;

            float swingValue = gamepad2.right_stick_y;

            right = throttle - direction;
            left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);
            if(swingValue == 0){
                swingValue = .5f;
            }

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            right = (float)scaleInput(right);
            left =  (float)scaleInput(left);

            float speedMultiplier = .5f;
            float strafeSpeed = .5f;

            if(gamepad2.x){
                swingValue = 1;
            }
            if(gamepad2.b){
                swingValue = -.7f;
            }
            if(gamepad2.a){
                swingValue = .75f;
            }

            // write the values to the motors
            if(leftStrafe == 0 && rightStrafe == 0) {
                robot.driveAtSpeed(left * speedMultiplier, right * speedMultiplier);
            }
            else{
                if(leftStrafe > 0 && rightStrafe == 0) {
                    robot.strafeLeft(leftStrafe * strafeSpeed);
                }else if (rightStrafe > 0 && leftStrafe == 0){
                    robot.strafeRight(rightStrafe * strafeSpeed);
                }
            }


            telemetry.addData("Swing Value: ", swingValue);
            robot.leftArmServo.setPosition(swingValue);
            robot.rightArmServo.setPosition(swingValue);

            if(!robot.isSensing){
                if(shouldSenseLeft && !shouldSenseRight){
                    //robot.senseInDirection("left");
                }else if(!shouldSenseLeft && shouldSenseRight){
                    //robot.senseInDirection("right");
                }
            }

            telemetry.addData("Front Multiplier: ", robot.frontSpeedMultipler);
            telemetry.addData("Encoder Values",  "Starting at %7d :%7d",
                    robot.frontLeftMotor.getCurrentPosition(),
                    robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Left Servo Position: ", robot.leftArmServo.getPosition());
            telemetry.addData("Right Servo Position: ", robot.rightArmServo.getPosition());
            telemetry.addData("Seconds: ", timer.seconds());
            telemetry.update();


            // update previous state variable.

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");

            float red = robot.colorSensor.red();
            float green = robot.colorSensor.green();
            float blue = robot.colorSensor.blue();

            telemetry.addData("Red: ", red);
            telemetry.addData("Green: ", green);
            telemetry.addData("Blue: ", blue);

            if(red > green && red > blue){
                telemetry.addData("Color: ", "Red");
            }
            if(blue > green && blue > red){
                telemetry.addData("Color: ", "Blue");
            }
//          //Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }







}

