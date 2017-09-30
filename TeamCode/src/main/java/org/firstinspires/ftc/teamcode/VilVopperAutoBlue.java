import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vec2F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by CCA on 12/5/2016.
 */

//@Autonomous(name="VilVopperAutoBlue", group="MyGroup")
public class VilVopperAutoBlue extends LinearOpMode{

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Drive variables
    int encFR = 0, encFL = 0, encBR = 0, encBL = 0, otherVar = 0;

    // Strafing variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 3, FUDGE_FACTOR = 1.10;


    // Flicker targeting variables
    //boolean targetSet = false;
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181

    // Start a timer, for any time-related functions
    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0;

    // Vuforia window bounds (Area checked for red/blue)
    int WINDOW_X_MIN = 600, WINDOW_X_MAX = 1200;
    int WINDOW_LEFT_Y = 719, WINDOW_MID_Y = 360, WINDOW_RIGHT_Y = 0;

    // Vuforia misc.
    int leftRedAvg = 0, leftBlueAvg = 0, rightRedAvg = 0, rightBlueAvg = 0;
    int leftRed = 0, leftBlue = 0, rightRed = 0, rightBlue = 0;
    //int c = 0, red = 0, blue = 0;
    long numImages = 0;
    Image rgb = null;
    Bitmap bm = null;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double driveEncAvg = 0, leftEncAvg = 0, rightEncAvg = 0;


    public void runOpMode() throws InterruptedException {

        initialize();

        telemetrize();

        waitForStart();

        //beacons.activate();

        while(opModeIsActive()) {

            //Launch two balls (Encoders)

            telemetrize();

            driveForTicks (9999,1);

            turnDegrees (180, .5);

            /*
            openGate();
            flickOnce();

            openGate();
            flickOnce();

            //Move forward (to clear wall) (Encoders)

            telemetrize();

            driveForTicks(1000, .5);

            telemetrize();

            //Turn right (Gyro)

            turnDegrees(40, .5);

            //Drive straight forward (Encoders)

            driveForTicks(4000, .5);

            //Turn left (should be level with beacon) (Gyro)

            turnDegrees(150, .5);
*/
            //Use Vuforia to check angle with beacon image + align robot to be perpendicular to that (Vuforia)

            //Use Vuforia to check color configuration of beacon (Vuforia)

            //Adjust forward/back x amount to align bopper with correct color (Encoders)

            //Strafe straight right to hit beacon (Gyro + Encoders)

            //(optional) use vuforia to check whether or not beacon is full blue, and repeat last few steps if not

            //Strafe left (to clear beacon) (Encoders + Gyro)

            //Move forward to the next beacon (Encoders)

            //Use Vuforia to align to be perpendicular (Vuforia)

            //Use Vuforia to check color configuration (Vuforia)

            //Adjust forward/back x amount (Encoders)

            //Strafe right to hit beacon (w/ optional after-check) (gyro + encoders)

            stop();
        }
    }

    public void initialize() {

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
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Reset drive encoders
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setDirection(DcMotor.Direction.REVERSE);
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Set gate to be closed at the start of the match
        gate.setPosition(0);

        /*
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        this.locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        this.beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        */

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        timeMark = runTime.time();

        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {

        }

        telemetry.addLine("Autonomous Initialized!");
    }

    // Get average of left drive encoders
    public double getLeftDriveEncAvg() {
        leftEncAvg = (frontLeft.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 2;
        return leftEncAvg;
    }

    // Get average of right drive encoders
    public double getRightDriveEncAvg() {
        rightEncAvg = (frontRight.getCurrentPosition()
                + backRight.getCurrentPosition()) / 2;
        return rightEncAvg;
    }

    // Get average of all drive encoders
    public double getAllDriveEncAvg() {
        driveEncAvg = (frontRight.getCurrentPosition()
                + frontLeft.getCurrentPosition()
                + backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 4;

        return driveEncAvg;
    }

    // Put telemetry values up on DS screen (for debugging)
    public void telemetrize() {
        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
        telemetry.addData("Drive Enc Avg: ", driveEncAvg);

        telemetry.addData("Enc FR: ", frontRight.getCurrentPosition());
        telemetry.addData("Enc FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("Enc BR: ", backRight.getCurrentPosition());
        telemetry.addData("Enc BL: ", backLeft.getCurrentPosition());

        telemetry.addData("leftRedAvg", leftRedAvg);
        telemetry.addData("leftBlueAvg", leftBlueAvg);

        telemetry.update();
    }

    // Telemetry for use in debugging
    public void badTimes() {
        telemetry.addLine("Something went terribly wrong.");
    }

    // Wait function that doesn't crash
    public void betterWait(double msec) {
        timeMark = runTime.time();

        msec /= 1000;

        while((runTime.time() < (timeMark + msec)) && opModeIsActive()) {
            telemetry.addData("Time: ", runTime.time());
            telemetry.addData("Target: ", (timeMark + msec));
        }
    }

    // Set left drive motors to a specified power
    public void setLeftDrivePower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    // Set right drive motors to a specified power
    public void setRightDrivePower(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    // Set all drive motors to a specified power
    public void setAllDrivePower (double power) {
        setLeftDrivePower(power);
        setRightDrivePower(power);
    }

    // Stop all drive motors
    public void stopDriveMotors() {
        setAllDrivePower(0);
    }

    // Sets all drive motors to a specified RunMode
    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    // Set drive encoders to 0
    public void resetDriveEncoders() {
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Drive straight for a specified number of encoder ticks
    public void driveForTicks(int encTicks, double power) {  // WORKING NOTE: Relying solely on RUN_TO_POSITION right now; if bot doesn't drive straight, modify to incorporate encoders and gyro
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

        initialHeading = gyro.getIntegratedZValue();

        frontLeft.setTargetPosition(encTicks);
        frontRight.setTargetPosition(encTicks);
        backLeft.setTargetPosition(encTicks);
        backRight.setTargetPosition(encTicks);

        while ((Math.abs(getAllDriveEncAvg()) < encTicks) && opModeIsActive()) {
            telemetrize();

            headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0);

            setRightDrivePower(power - (HEADING_CORRECTION_SCALE * headingDiffPercent));
            setLeftDrivePower(power + (HEADING_CORRECTION_SCALE * headingDiffPercent));


            /*
            // Turning left
            if (gyro.getIntegratedZValue() > initialHeading) {
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            }
            // Turning Right
            else if (gyro.getIntegratedZValue() < initialHeading) {
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            }*/

        }

        stopDriveMotors();
    }

    // Turn a specified number of degrees, using the gyro sensor
    public void turnDegrees(int degrees, double power) {
        initialHeading = Math.abs(gyro.getIntegratedZValue());
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turning Left
        if (degrees > 0) {
            while((Math.abs(gyro.getIntegratedZValue()) < (initialHeading + degrees)) && opModeIsActive()) {
                telemetrize();

                setRightDrivePower(power);
                setLeftDrivePower(-power);
            }
        }
        // Turning Right
        else if (degrees < 0) {
            while ((Math.abs(gyro.getIntegratedZValue()) > (initialHeading + degrees)) && opModeIsActive()) {
                telemetrize();

                setRightDrivePower(-power);
                setLeftDrivePower(power);
            }
        }
        else {
            badTimes();
        }

        stopDriveMotors();
    }

    // Rotate flicker once, to launch a loaded ball
    public void flickOnce() throws InterruptedException {
        encVal = Math.abs(flicker.getCurrentPosition());
        targetVal = encVal + 3360;

        flicker.setTargetPosition((targetVal - FLICKER_ERROR));
        flicker.setPower(1);

        betterWait(2000);
        flicker.setPower(0);
    }

    // Open gate, to load a ball from the hopper into the launcher
    public void openGate() throws InterruptedException {
        gate.setPosition(.3);

        betterWait(2000);

        gate.setPosition(0);
    }
}
