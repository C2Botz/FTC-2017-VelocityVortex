package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Fibonacci on 2/3/17.
 */
@Autonomous(name = "Release Autonomous", group = "Release Opmode")
public class Auto extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.resetEncoders();
        telemetry.addData("Auto Alliance", robot.alliance);    //
        telemetry.update();

        waitForStart();
        if(robot.alliance != AllianceColor.NULL) {
            switch(robot.autoMode){

                case BEACON:
                    beaconAutoMode();
                    break;

                case CORNER:
                    cornerAutoMode();
                    break;

                case CAPBALL:
                    capballAutoMode();
                    break;

                default:
                    System.out.println("Automode does not exist");
            }
        }


    }

    private void beaconAutoMode(){
        encoderDrive(DRIVE_SPEED, 39, 39, 10);
        encoderDrive(DRIVE_SPEED, -5, -5, 10);
        if (robot.alliance == AllianceColor.RED) {
            encoderDrive(DRIVE_SPEED, -19, 19, 10);
        } else {
            encoderDrive(DRIVE_SPEED, 19, -19, 10);
        }
        encoderDrive(DRIVE_SPEED, 39.5, 39.5, 10);
        if (robot.alliance == AllianceColor.RED) {
            senseInDirection(Direction.RIGHT, 5);
        } else {
            senseInDirection(Direction.LEFT, 5);
        }
        encoderDrive(DRIVE_SPEED, -7, -7, 10);
        if (robot.alliance == AllianceColor.RED) {
            encoderDrive(DRIVE_SPEED, -19, 19, 10);
        } else {
            encoderDrive(DRIVE_SPEED, 19, -19, 10);
        }
        encoderDrive(DRIVE_SPEED, 45, 45, 10);
    }

    private void cornerAutoMode(){
        encoderDrive(DRIVE_SPEED, 39, 39, 10);
        encoderDrive(DRIVE_SPEED, -5, -5, 10);
        robot.leftArmServo.setPosition(1);
        robot.rightArmServo.setPosition(1);
        if (robot.alliance == AllianceColor.RED) {
            encoderDrive(DRIVE_SPEED, -27, 27, 10);
        } else {
            encoderDrive(DRIVE_SPEED, 28, -28, 10);
        }
        robot.leftArmServo.setPosition(1);
        robot.rightArmServo.setPosition(1);
        encoderDrive(DRIVE_SPEED, 35, 35, 10);
        robot.leftArmServo.setPosition(1);
        robot.rightArmServo.setPosition(1);
    }

    private void capballAutoMode(){
        encoderDrive(DRIVE_SPEED, 39, 39, 10);
        encoderDrive(DRIVE_SPEED, -38, -38, 10);

    }



    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if(rightInches < 0){
                rightInches += .35d;
            }else if(rightInches > 0){
                rightInches -= .35d;
            }
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftMotor.setPower(Math.abs(speed*robot.frontSpeedMultipler));
            robot.frontRightMotor.setPower(Math.abs(speed*robot.frontSpeedMultipler));
            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.rearLeftMotor.setPower(0);
            robot.rearRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }





    void senseInDirection(Direction direction, int timeout){
        int beaconsPressed = 0;

        ElapsedTime timer = new ElapsedTime();
        double seconds = -1;
        boolean setSeconds = false;
        robot.enableColorSensorLED(false);

        while(beaconsPressed < 1){
            telemetry.addData("LED is Enabled: ", robot.isLedOn);
            AllianceColor seenColor = AllianceColor.NULL;
            telemetry.update();
            if(!robot.isLedOn){
                if(direction == Direction.LEFT){
                    robot.strafeLeft(0.35f);
                }else if(direction == Direction.RIGHT){
                    robot.strafeRight(0.35f);
                }else{
                    robot.stop();
                    break;
                }

            }

            float red = robot.colorSensor.red();
            float blue = robot.colorSensor.blue();
            float green = robot.colorSensor.green();

            if(setSeconds){
                seconds = timer.seconds();
                telemetry.addData("Seconds Since Activation: ", seconds);
                telemetry.update();
            }

            if(seconds > 2 || seconds == -1) {
                if ((red > blue) && !robot.isLedOn && robot.alliance != AllianceColor.RED) {
                    seenColor = AllianceColor.RED;
                }
                if(red > blue && robot.alliance == AllianceColor.RED){
                    seenColor = AllianceColor.NULL;
                }

                if ((blue > red) && !robot.isLedOn && robot.alliance != AllianceColor.BLUE) {
                    seenColor = AllianceColor.BLUE;
                }
                if(blue > red && robot.alliance == AllianceColor.BLUE){
                    seenColor = AllianceColor.NULL;
                }
                switch (robot.alliance) {

                    case RED:

                        if ((red > blue) && !robot.isLedOn) {
                            robot.enableColorSensorLED(true);
                        }
                        break;

                    case BLUE:
                        if ((blue > red) && !robot.isLedOn) {
                            robot.enableColorSensorLED(true);
                        }
                        break;

                    case NULL:
                        break;
                }
                setSeconds = false;
                seconds = -1;
            }
            if(robot.isLedOn){
                if(red == 0 && blue == 0 && green == 0){
                    //Move a little and go forward to press button
                    if(seenColor == AllianceColor.NULL) {
                        robot.strafeForTime(direction, (float) DRIVE_SPEED, .15f);
                    }else{
                        robot.strafeForTime(direction, (float) DRIVE_SPEED, .10f);
                    }
                    encoderDrive(.1, 7, 7, 10);
                    encoderDrive(.1, -7, -7, 10);
                    beaconsPressed += 1;
                    timer.reset();
                    setSeconds = true;
                    //After, turn led off
                    if(beaconsPressed == 1) {
                        if(robot.alliance == AllianceColor.RED) {
                            robot.rotateRightSideForTime(-0.1f, .65f);
                        }else if(robot.alliance == AllianceColor.BLUE){
                            robot.rotateLeftSideForTime(-0.1f, .65f);
                        }
                    }
                    robot.enableColorSensorLED(false);
                }
            }



        }
        robot.isSensing = false;
    }
}
