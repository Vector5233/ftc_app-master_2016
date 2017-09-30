import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="MecanumDriveTest", group = "myGroup")

/**
 * Created Nov. 29, 2016 by Wil Orlando
 *
 * This is my attempt at a teleop without being able to test. Most of it should work.
 *
 * Control scheme should be identical to the one laid out by Stephen and Olu on the controller printouts.
 *
 * Possible problems:
 *  General- Some button maps (i.e. strafing, collecting) may be awkward for driver to push
 */

public class MecanumDriveTest extends OpMode {

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 10, FUDGE_FACTOR = 1.10;


    boolean targetSet = false;
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181
    boolean flickLock = false; // Locks when the flicker is running
    int FLICK_MANUAL_INCREMENT = 1; // Increment at which manual flicker resetting changes flicker position

    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0, obsCheckTime = 0;
    int encCheckCounter = 0, obstacleCounter = 0;
    double frontSpeed = 0, backSpeed = 0;
    double encCheckTime[] = new double [2];
    double frontEncAvg[] = new double [2];
    double backEncAvg[] = new double [2];
    boolean firstEncLoop = true;
    int MARGIN_OF_ERROR = 400;
    int WALL_TOLERANCE_SECONDS = 3;

    // Initialize robot before match starts
    public void init() {

        // Map this program's names for motors to the RC Phone's config file
        frontRight = hardwareMap.dcMotor.get ("frontRight");
        frontLeft = hardwareMap.dcMotor.get ("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor. get("backLeft");
        flicker = hardwareMap.dcMotor.get("flicker");
        collector = hardwareMap.dcMotor.get("collector");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Make sure that drive motors are using encoders
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setDirection(DcMotor.Direction.REVERSE);
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        // POTENTIAL TROUBLE!  EMPTY LOOP MAY NOT SYNCHRONIZE
        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {

        }

        telemetry.addLine("Fully Initialized!");
    }

    public void loop() {



        telemetry.addData(">", "Robot Heading Z = %d", gyro.getIntegratedZValue());
        // Drive Controls //

        // Strafe to the right if the joysticks are pushed to the right
        if (gamepad1.right_stick_x > .5 && gamepad1.left_stick_x > .5) {
            //Record initial heading of robot
            if (!headingSet) {
                setInitialHeading();
            }
            // Strafe right, keeping the initial heading, while the driver holds joysticks
            else {
                strafeStraightRight();
            }
        }
        // Strafe to the left if the joysticks are pushed to the left
        else if (gamepad1.right_stick_x < -.5 && gamepad1.left_stick_x < -.5) {
            //Record initial heading of robot
            if (!headingSet) {
                setInitialHeading();
            }
            // Strafe left, keeping the initial heading, while the driver holds joysticks
            else {
                strafeStraightLeft();
            }
        }
        // Otherwise, bind motors to joysticks' y axes (normal tank drive)
        else {
            if (headingSet) {
                headingSet = false;
            }

            // Compare front/back rotation speeds every second
            if ((runTime.time() - obsCheckTime) >= 1) {
                obsCheckTime = runTime.time();

                //record encoder ticks and current time of front axle and back axle
                encCheckTime[encCheckCounter % 2] = obsCheckTime;
                frontEncAvg[encCheckCounter % 2] = getFrontDriveEncAvg();
                backEncAvg[encCheckCounter % 2]= getBackDriveEncAvg();

                if (!firstEncLoop) {
                    frontSpeed = (frontEncAvg[encCheckCounter%2] - frontEncAvg[(encCheckCounter - 1)%2]) / (encCheckTime[encCheckCounter%2] - encCheckTime[(encCheckCounter - 1)%2]);
                    backSpeed = (backEncAvg[encCheckCounter%2] - backEncAvg[(encCheckCounter - 1)%2]) / (encCheckTime[encCheckCounter%2] - encCheckTime[(encCheckCounter - 1)%2]);

                    // If the magnitude of the difference between the front and back encoder averages is greater than a margin of error, add 1 to the obstacle counter
                    if (Math.abs(frontSpeed - backSpeed) > MARGIN_OF_ERROR) {
                        if (obstacleCounter < (WALL_TOLERANCE_SECONDS + 1)) obstacleCounter++;
                    }
                    // If the axles are moving at the same rate, reset the obstacle counter
                    else {
                        if (obstacleCounter > 0) obstacleCounter--;
                    }

                }
                else firstEncLoop = false;

                encCheckCounter++;
            }

            // If the obstacle counter reaches 3 (meaning you've been running up against a wall for 3 seconds), reduce power on drive motors
            if (obstacleCounter >= WALL_TOLERANCE_SECONDS) {
                frontRight.setPower(gamepad1.right_stick_y * .1);
                backRight.setPower(gamepad1.right_stick_y * .1);
                frontLeft.setPower(gamepad1.left_stick_y * .1);
                backLeft.setPower(gamepad1.left_stick_y * .1);
            }
            else {
                frontRight.setPower(gamepad1.right_stick_y);
                backRight.setPower(gamepad1.right_stick_y);
                frontLeft.setPower(gamepad1.left_stick_y);
                backLeft.setPower(gamepad1.left_stick_y);
            }

        }

        telemetry.addData("frontSpeed ", frontSpeed);
        telemetry.addData("backSpeed ", backSpeed);
        telemetry.addData("encCheckCounter ", encCheckCounter);
        telemetry.addData("obstacleCounter", obstacleCounter);

        // Gate Controls //

        // Opens gate when x is pressed
        if (gamepad2.x) {
            gate.setPosition(.3); // Last set at .3
            gateState = "Open";
        }
        // Closes gate by default
        else {
            gate.setPosition(0); // Last set at 0
            gateState = "Closed";
        }

        telemetry.addData("Gate: ", gateState);


        // Collector Controls //

        // Collector out when dpad_up is pressed
        if (gamepad2.dpad_up) {
            collector.setPower(1);
        }
        // Collector in when dpad_down is pressed
        else if (gamepad2.dpad_down) {
            collector.setPower(-1);
        }
        // Collector off by default
        else {
            collector.setPower(0);
        }


        // Flicker Controls //

        //Working note: flicker.setPower(1) is the right direction

        if (gamepad2.y && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }
        // Manual reset, in case the flicker gets stuck, to rotate it a third of a rotation
        else if (gamepad2.right_bumper && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360/3;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }

        if ((runTime.time() - timeMark) >= 2) {
            flicker.setPower(0);
        }

        telemetry.addData("Flicker Enc: ", flicker.getCurrentPosition());
        telemetry.addData("Flicker Power: ", flicker.getPower());
        /*
        if(gamepad2.y && (flicker.getPower() == 0)) {
            //flickState = "Targeting";
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;
            flicker.setPower(1);
        }

        if (Math.abs(flicker.getCurrentPosition()) < (targetVal - FLICKER_ERROR)) {
            //flickState = "Running";
            flicker.setPower(1);
        }
        else {
            //flickState = "Idle";
            flicker.setPower(0);
        }*/

        /*
        telemetry.addData("Flick State: ", flickState);
        telemetry.addData("Flicker Enc: ", flicker.getCurrentPosition());
        telemetry.addData("Flicker Power(): ", flicker.getPower());
        telemetry.addData("EncVal: ", encVal);
        telemetry.addData("TargetVal: ", targetVal);
        */
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
        headingSet = true;
    }


    public void strafeStraightRight() {
        double powerFL = -.5, powerFR = .5, powerBL = .5, powerBR = -.5;
        double turnCorrection = (double)(gyro.getIntegratedZValue() - initialHeading)/100.0; // Positive when turning left, negative when turning right
        //double driftCorrection = (double)((frontLeft.getCurrentPosition() + frontRight.getCurrentPosition())
        //                            - (backLeft.getCurrentPosition() + backRight.getCurrentPosition()))/100.0; // Positive when drifting forward, negative when drifting backward
        double driftCorrection = 0;

        //headingDiffPercent = ((double)(gyro.getIntegratedZValue() - initialHeading)/100.0); //.02

        powerFL += -turnCorrection - driftCorrection;
        powerFR += turnCorrection - driftCorrection;
        powerBL += -turnCorrection + driftCorrection;
        powerBR += turnCorrection + driftCorrection;

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

        /*
        powerFL *=  (1/FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBR *= (1/FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        */


        Range.clip(powerFL, -1, 1);
        Range.clip(powerFR, -1, 1);
        Range.clip(powerBL, -1, 1);
        Range.clip(powerBR, -1, 1);


        //if drifting forward, headingDiffPercent will be negative (and positive if drifting back)
        frontRight.setPower(powerFR);
        backRight.setPower(powerBR);
        frontLeft.setPower(powerFL);
        backLeft.setPower(powerBL);

        telemetry.addData("initialHeading: ", initialHeading);
        telemetry.addData("turnCorrection: ", turnCorrection);
        telemetry.addData("powerFR: ", powerFR);
        telemetry.addData("powerFL: ", powerFL);
        telemetry.addData("powerBR: ", powerBR);
        telemetry.addData("powerBL: ", powerBL);

        //headingCycles += 1;
        //telemetry.addData("headingCycles: ", headingCycles);
    }


    //Strafe left doesn't strafe perfectly left; signs are wrong on either fudgefactor (multiplied instead of divided) or the 1-() and 1+() in powers
    public void strafeStraightLeft() {
        double powerFL = .5, powerFR = -.5, powerBL = -.5, powerBR = .5;
        double turnCorrection = (double)(gyro.getIntegratedZValue() - initialHeading)/100.0; // Positive when turning left, negative when turning right
        //double driftCorrection = (double)((frontLeft.getCurrentPosition() + frontRight.getCurrentPosition())
        //        - (backLeft.getCurrentPosition() + backRight.getCurrentPosition()))/100.0; // Positive when drifting forward, negative when drifting backward
        double driftCorrection = 0;
        //headingDiffPercent = ((double)(gyro.getIntegratedZValue() - initialHeading)/100.0); //.02

        powerFL += -turnCorrection - driftCorrection;
        powerFR += turnCorrection - driftCorrection;
        powerBL += -turnCorrection + driftCorrection;
        powerBR += turnCorrection + driftCorrection;

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

        /*
        powerFL *=  (1/FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBR *= (1/FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        */


        Range.clip(powerFL, -1, 1);
        Range.clip(powerFR, -1, 1);
        Range.clip(powerBL, -1, 1);
        Range.clip(powerBR, -1, 1);


        //if drifting forward, headingDiffPercent will be negative (and positive if drifting back)
        frontRight.setPower(powerFR);
        backRight.setPower(powerBR);
        frontLeft.setPower(powerFL);
        backLeft.setPower(powerBL);

        telemetry.addData("initialHeading: ", initialHeading);
        telemetry.addData("turnCorrection: ", turnCorrection);
        telemetry.addData("powerFR: ", powerFR);
        telemetry.addData("powerFL: ", powerFL);
        telemetry.addData("powerBR: ", powerBR);
        telemetry.addData("powerBL: ", powerBL);

        //headingCycles += 1;
        //telemetry.addData("headingCycles: ", headingCycles);
        /*
        double powerFL = .5, powerFR = -.5, powerBL = -.5, powerBR = .5;
        headingDiffPercent = ((double)(gyro.getIntegratedZValue() - initialHeading)/100.0); //.02

        powerFL *=  (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (1/FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (1/FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBR *= (FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));

        Range.clip(powerFL, -1, 1);
        Range.clip(powerFR, -1, 1);
        Range.clip(powerBL, -1, 1);
        Range.clip(powerBR, -1, 1);


        //if drifting forward, headingDiffPercent will be negative (and positive if drifting back)
        frontRight.setPower(powerFR);
        backRight.setPower(powerBR);
        frontLeft.setPower(powerFL);
        backLeft.setPower(powerBL);

        telemetry.addData("initialHeading: ", initialHeading);
        telemetry.addData("headingDiffPercent: ", headingDiffPercent);
        telemetry.addData("powerFR: ", powerFR);
        telemetry.addData("powerFL: ", powerFL);
        telemetry.addData("powerBR: ", powerBR);
        telemetry.addData("powerBL: ", powerBL);*/

        //headingCycles += 1;
        //telemetry.addData("headingCycles: ", headingCycles);
    }

    // Sets all drive motors to a specified RunMode
    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public double getBackDriveEncAvg() {
        return (backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 2;
    }

    // Get average of right drive encoders
    public double getFrontDriveEncAvg() {
        return (frontRight.getCurrentPosition()
                + frontLeft.getCurrentPosition()) / 2;
    }

    // Checks to see if bot has hit obstacle by checking difference in front/back encoders
    public boolean hitObstacle(){
        if (Math.abs(getBackDriveEncAvg() - getFrontDriveEncAvg()) > 3000) return true;
        else return false;
    }


}

