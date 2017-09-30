import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="WilStrafer", group = "myGroup")

/**
 * Wil's top secret strafe arcade-mode control style. Allows for diagonal strafing with right joystick, turns bot with RB and LB.
 * Otherwise identical to WilTeleTest (as of 12/5)
 */
public class WilStrafer extends OpMode{
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 10, FUDGE_FACTOR = 1.10;
    int headingCycles = 0;


    boolean targetSet = false;
    int encVal, targetVal;
    int flickError = 181; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181



    //Wil's diagonal strafe thing vars
    double powerFLBR = 0, powerFRBL = 0;
    double joyX = 0, joyY = 0;

    public void init() {

        // Map this program's names for motors to the RC Phone's config file
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        flicker = hardwareMap.dcMotor.get("flicker");
        collector = hardwareMap.dcMotor.get("collector");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();
    }

    public void loop() {

        telemetry.addData(">", "Robot Heading Z = %d", gyro.getIntegratedZValue());
        // Drive Controls //

        //Wil's super secret experimental arcade-mode strafetastic control system


        //left joystick x turns
        if (gamepad1.left_bumper) {
            frontRight.setPower(-1);
            backRight.setPower(-1);
            frontLeft.setPower(1);
            backLeft.setPower(1);
        }
        else if (gamepad1.right_bumper) {
            frontRight.setPower(1);
            backRight.setPower(1);
            frontLeft.setPower(-1);
            backLeft.setPower(-1);
        }
        //right joystick (both axes) strafes
        else {
            joyX = gamepad1.right_stick_x; //+ dpheta ? (would it be dpheta/pi or dpheta/2pi or something, to scale within the +-1 range?
            joyY = gamepad1.right_stick_y; //+ dpheta (difference from angle at start of teleop (actually, start of autonomous), so controls always function same from driver's perspective?

            powerFLBR = joyY - joyX;  //Can divide both power values by root(2); will make strafing slower, but more accurate, by scaling values from a range of +-(root(2) / 2) to +-1
            powerFRBL = joyY + joyX;

            frontRight.setPower(powerFRBL);
            backRight.setPower(powerFLBR);
            frontLeft.setPower(powerFLBR);
            backLeft.setPower(powerFRBL);

            telemetry.addData("joyX", joyX);
            telemetry.addData("joyY", joyY);
            telemetry.addData("powerFLBR", powerFLBR);
            telemetry.addData("powerFRBL", powerFRBL);
        }






        // Gate Controls //

        // Opens gate when b is pressed
        if (gamepad2.b) {
            gate.setPosition(0); // Last set at 0
        }
        // Closes gate when x is pressed
        else if (gamepad2.x) {
            gate.setPosition(.3); // Last set at .3
        }



        // Collector Controls //

        // Collector in when dpad_up is pressed
        if (gamepad2.dpad_up) {
            collector.setPower(1);
        }
        // Collector out when dpad_down is pressed
        else if (gamepad2.dpad_down) {
            collector.setPower(-1);
        }
        // Collector off by default
        else {
            collector.setPower(0);
        }


        // Flicker Controls //

        //Working note: flicker.setPower(-1) is the right direction

        if(gamepad2.y && (flicker.getPower() == 0)) {
            //flickState = "Targeting";
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;
            flicker.setPower(-1);
        }

        if (Math.abs(flicker.getCurrentPosition()) < (targetVal - flickError)) {
            //flickState = "Running";
            flicker.setPower(-1);
        }
        else {
            //flickState = "Idle";
            flicker.setPower(0);
        }

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
        headingDiffPercent = ((double)(gyro.getIntegratedZValue() - initialHeading)/100.0); //.02

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

        powerFL *=  (1/FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBR *= (1/FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));

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
        telemetry.addData("powerBL: ", powerBL);

        //headingCycles += 1;
        //telemetry.addData("headingCycles: ", headingCycles);
    }


    //Strafe left doesn't strafe perfectly left; signs are wrong on either fudgefactor (multiplied instead of divided) or the 1-() and 1+() in powers
    public void strafeStraightLeft() {
        double powerFL = .5, powerFR = -.5, powerBL = -.5, powerBR = .5;
        headingDiffPercent = ((double)(gyro.getIntegratedZValue() - initialHeading)/100.0); //.02

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

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
        telemetry.addData("powerBL: ", powerBL);

        //headingCycles += 1;
        //telemetry.addData("headingCycles: ", headingCycles);
    }






}
