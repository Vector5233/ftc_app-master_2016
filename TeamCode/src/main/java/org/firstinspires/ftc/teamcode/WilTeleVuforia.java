import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.Arrays;

@TeleOp(name="WilTeleVuforia", group = "myGroup")

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

public class WilTeleVuforia extends OpMode {

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate, bopper;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Strafing variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 10, FUDGE_FACTOR = 1.10;


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
    boolean imageFound = false;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    VuforiaTrackableDefaultListener myListener;

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double driveEncAvg = 0;

    // Initialize robot before match starts
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

        // Reset drive encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        }

        // Initialize Vuforia
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        this.locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        this.beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        beacons.activate();

        telemetry.addLine("Teleop Initialized!");
    }

    public void loop() {


        telemetry.addData(">", "Robot Heading Z = %d", gyro.getIntegratedZValue());
        // Drive Controls //

        frontRight.setPower(gamepad1.right_stick_y);
        backRight.setPower(gamepad1.right_stick_y);
        frontLeft.setPower(gamepad1.left_stick_y);
        backLeft.setPower(gamepad1.left_stick_y);

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

        //Working note: flicker.setPower(-1) is the right direction

        if (gamepad2.y && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }

        if ((runTime.time() - timeMark) >= 2) {
            flicker.setPower(0);
        }

        telemetry.addData("Flicker Enc: ", flicker.getCurrentPosition());
        telemetry.addData("Flicker Power: ", flicker.getPower());

        // Vuforia Magic


        // Get translation and degree values from phone to Vuforia printout images under beacons
        /*for (VuforiaTrackable beac : beacons) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

            if (pose != null) {

                //Matrix34F rawPose = new Matrix34F();
                //float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(2), translation.get(0)));
                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
            }
        }*/

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                imageFound = true;

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);
            }
            else {
                imageFound = false;
            }
        }



        /*
        // Grab a frame from the phone's camera
        grabFrame();

        // Modify leftRedAvg, etc to reflect the amount of red/blue on each half of the camera window
        checkCameraLeft();
        checkCameraRight();
        */

        // Autonomous-ing Telemetry

        telemetry.addData("Enc FR: ", frontRight.getCurrentPosition());
        telemetry.addData("Enc FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("Enc BR: ", backRight.getCurrentPosition());
        telemetry.addData("Enc BL: ", backLeft.getCurrentPosition());

        driveEncAvg = (0.25) * (frontRight.getCurrentPosition()
                + frontLeft.getCurrentPosition()
                + backRight.getCurrentPosition()
                + backLeft.getCurrentPosition());

        telemetry.addData("Drive Enc Avg: ", driveEncAvg);

        telemetry.addData("leftRedAvg", leftRedAvg);
        telemetry.addData("leftBlueAvg", leftBlueAvg);
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
        headingSet = true;
    }


    public void strafeStraightRight() {
        double powerFL = -.5, powerFR = .5, powerBL = .5, powerBR = -.5;
        headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0); //.02

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

        powerFL *= (1 / FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBR *= (1 / FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));

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
        headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0); //.02

        /*forwardPower *= 1 + (HEADING_CORRECTION_SCALE * headingDiffPercent); //.58
        backwardPower *= 1 - (HEADING_CORRECTION_SCALE * headingDiffPercent); //-.42*/

        powerFL *= (FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerFR *= (1 / FUDGE_FACTOR) * (1 - (HEADING_CORRECTION_SCALE * headingDiffPercent));
        powerBL *= (1 / FUDGE_FACTOR) * (1 + (HEADING_CORRECTION_SCALE * headingDiffPercent));
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


    // Grab a frame from the phone's camera, for use in color analyzation
    public void grabFrame() {
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch (InterruptedException e) {
            frame = null;
        }

        if (frame != null) {
            numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }//if
            }//for

            if (rgb != null) {
                bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());
            }
        }
    }

        // Find average amount of red and blue on left phone window

    public void checkCameraLeft() {
        leftRed = 0;
        leftBlue = 0;

        // Check all pixels in defined window sizes, and add up amount of red and blue in each
        for (int x = WINDOW_X_MIN; x < WINDOW_X_MAX; x++) {

            for (int y = WINDOW_MID_Y; y < WINDOW_LEFT_Y; y++) {
                leftRed += ((bm.getPixel(x, y)) & 0x00FF0000) >> 16;
                leftBlue += ((bm.getPixel(x, y)) & 0x000000FF);
            }

        }


        // Avg variables equal the amount of red/blue (leftRed/leftBlue) divided by the number of pixels scanned
        leftRedAvg = leftRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_LEFT_Y - WINDOW_MID_Y));
        leftBlueAvg = leftBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_LEFT_Y - WINDOW_MID_Y));
    }

    // Find average amount of red and blue on right phone window
    public void checkCameraRight() {
        rightRed = 0;
        rightBlue = 0;

        // Check all pixels in defined window sizes, and add up amount of red and blue in each
        for (int x = WINDOW_X_MIN; x < WINDOW_X_MAX; x++) {

            for (int y = WINDOW_RIGHT_Y; y < WINDOW_MID_Y; y++) {
                rightRed += ((bm.getPixel(x, y)) & 0x00FF0000) >> 16;
                rightBlue += ((bm.getPixel(x, y)) & 0x000000FF);
            }

        }

        // Avg variables equal the amount of red/blue (rightRed/rightBlue) divided by the number of pixels scanned
        rightRedAvg = leftRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
        rightBlueAvg = leftBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
    }


}

