/**
 * Created Dec. 5, 2016 by Wil Orlando.
 *
 * 5233 Vector's second autonomous routine, for use alongside a partner.
 *
 * If a partner's autonomous conflicts with ours, this program is a fallback to ensure we can still
 * score some points.
 *
 * Here, the robot scoots forward, sits in place and fires two balls into the center vortex for a
 * total of 30 pts.
 */

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
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

@Autonomous(name="VopperAutoStationary", group="MyGroup")
public class VopperAutoStationary extends LinearOpMode {

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Strafing variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 3, FUDGE_FACTOR = 1.10;
    double beaconStrafeTicks = 0; //ticks run when trying to slam into beacon
    double Z_TO_STRAFE_SCALE = 1.4; // Multiply z distance from beacon with this to get encoder ticks to run

    // Turn variables
    int startingDegrees = 0;
    int TURN_DEG_TOLERANCE = 2;
    double TURN_SENSITIVITY = .04/(double)TURN_DEG_TOLERANCE;

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

    // Vuforia image alignment
    double X_ALIGNMENT_TOLERANCE = 3;
    double X_ALIGNMENT_SENSITIVITY =.002;

    // Vuforia color configuration check
    int leftRedAvg = 0, leftBlueAvg = 0, rightRedAvg = 0, rightBlueAvg = 0;
    int leftRed = 0, leftBlue = 0, rightRed = 0, rightBlue = 0;
    enum State {BLUE_RED, RED_BLUE}
    State colorConfig;

    // Vuforia misc.
    long numImages = 0;
    Image rgb = null;
    Bitmap bm = null;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double driveEncAvg = 0, leftEncAvg = 0, rightEncAvg = 0;
    String opState = "";


    public void runOpMode() throws InterruptedException {

        initialize();

        telemetrize();

        waitForStart();

        beacons.activate();

        while(opModeIsActive()) {

            startingDegrees = gyro.getIntegratedZValue();

            op("Adjusting for flick...");
            driveForTicks(500, .5);

            op("Flicking...");
            flickOnce();

            op("Opening Gate...");
            openGate();

            op("Flicking again...");
            flickOnce();

            op("Done!");
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

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        timeMark = runTime.time();

        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {
            idle();
        }


        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE;
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


        telemetry.addLine("Autonomous Initialized!");
    }

    // Put telemetry values up on DS screen (for debugging)
    public void telemetrize() {
        telemetry.addData("OpState: ", opState);
        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
        telemetry.addData("Drive Enc Avg: ", driveEncAvg);

        telemetry.addData("Enc FR: ", frontRight.getCurrentPosition());
        telemetry.addData("Enc FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("Enc BR: ", backRight.getCurrentPosition());
        telemetry.addData("Enc BL: ", backLeft.getCurrentPosition());

        telemetry.addData("leftRedAvg", leftRedAvg);
        telemetry.addData("leftBlueAvg", leftBlueAvg);
        telemetry.addData("FR power", frontRight.getPower());

        telemetry.addData("Color Config: ", colorConfig);
        telemetry.addData("leftBlueAvg: ", leftBlueAvg);
        telemetry.addData("leftRedAvg: ", leftRedAvg);
        telemetry.addData("rightBlueAvg: ", rightBlueAvg);
        telemetry.addData("rightRedAvg: ", rightRedAvg);
        telemetry.update();
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

    // Get average of all drive encoders
    public double getAllDriveEncAvg() {
        driveEncAvg = (frontRight.getCurrentPosition()
                + frontLeft.getCurrentPosition()
                + backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 4;

        return driveEncAvg;
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

    // Drive straight for a specified number of encoder ticks
    public void driveForTicks(int encTicks, double power) {

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);
        initialHeading = gyro.getIntegratedZValue();
        double rightPower, leftPower;

        frontLeft.setTargetPosition(encTicks);
        frontRight.setTargetPosition(encTicks);
        backLeft.setTargetPosition(encTicks);
        backRight.setTargetPosition(encTicks);

        while ((Math.abs(getAllDriveEncAvg()) < encTicks) && opModeIsActive()) {
            telemetrize();

            headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0);

            if (power > 0) {
                rightPower = power - (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(rightPower, 0 , 1);
                leftPower = power + (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(leftPower, 0 ,1);

                setRightDrivePower(rightPower);
                setLeftDrivePower(leftPower);
            }
            else {
                rightPower = power + (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(rightPower, -1 , 0);
                leftPower = power - (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(leftPower, -1 ,0);

                setRightDrivePower(rightPower);
                setLeftDrivePower(leftPower);
            }


        }

        stopDriveMotors();
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
    }

    // Rotate flicker once, to launch a loaded ball
    public void flickOnce() {
        encVal = Math.abs(flicker.getCurrentPosition());
        targetVal = encVal + 3360;

        flicker.setTargetPosition((targetVal - FLICKER_ERROR));
        flicker.setPower(1);

        sleep(2000);

        flicker.setPower(0);
    }

    // Open gate, to load a ball from the hopper into the launcher
    public void openGate() {
        gate.setPosition(.3);

        sleep(2000);

        gate.setPosition(0);
    }

    public void op(String state) {
        opState = state;
        telemetrize();
    }


}