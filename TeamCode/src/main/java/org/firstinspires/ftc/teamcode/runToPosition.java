/**
 * Created by CCA on 12/12/2016.
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

import java.util.Arrays;

/**
 * Created by CCA on 12/5/2016.
 */

//@Autonomous(name="runToPosition", group="MyGroup")
public class runToPosition extends LinearOpMode{

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
    double beaconStrafeTicks = 0; //ticks run when trying to slam into beacon
    double Z_TO_STRAFE_SCALE = 2; // Multiply z distance from beacon with this to get encoder ticks to run

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


    public void runOpMode() throws InterruptedException {

        initialize();

        telemetrize();

        waitForStart();

        beacons.activate();

        while(opModeIsActive()) {

            sleep(1000);

            newRunForDistance(10000000, 1);

            telemetry.addLine("Done!");

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

    // Modified encoder average for strafing; Strafing right will yield a positive value, strafing left a negative one
    public double getStrafeEncAvg() {
        driveEncAvg = (frontRight.getCurrentPosition()
                - frontLeft.getCurrentPosition()
                - backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 4;

        return driveEncAvg;
    }


    //set same target for all drive motors
    public void setAllTarget(int target) {
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);
    }

    //return True if any drive motor busy
    public boolean isAllBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
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
        telemetry.addData("FR power", frontRight.getPower());

        telemetry.addData("Color Config: ", colorConfig);
        telemetry.addData("leftBlueAvg: ", leftBlueAvg);
        telemetry.addData("leftRedAvg: ", leftRedAvg);
        telemetry.addData("rightBlueAvg: ", rightBlueAvg);
        telemetry.addData("rightRedAvg: ", rightRedAvg);
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

        }

        stopDriveMotors();
    }

    //go forward a specified number of encoder ticks
    public void newRunForDistance(int ticks, double power) {
        int target;

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);
        target = frontLeft.getCurrentPosition() + ticks;

        setAllTarget(target);
        setAllDrivePower(power);

        while(opModeIsActive() && isAllBusy()) {
            idle();
        }

        setAllDrivePower(0);
    }

    public void runForEnc(int ticks, double power) {
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftRightDiff;

        while(getAllDriveEncAvg() <= ticks) {
            leftRightDiff = Math.abs(getRightDriveEncAvg() - getLeftDriveEncAvg()) * .01;

            frontLeft.setPower(Range.clip(power - leftRightDiff, -1, 1));
            frontRight.setPower(Range.clip(power - leftRightDiff, -1, 1));
            backLeft.setPower(Range.clip(power - leftRightDiff, -1, 1));
            backRight.setPower(Range.clip(power - leftRightDiff, -1, 1));

        }

        stopDriveMotors();
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
    }

    public void strafeStraightLeft() {
        double powerFL = -.5, powerFR = .5, powerBL = .5, powerBR = -.5;
        headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0); //.02

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

        telemetrize();
    }


    //Strafe left doesn't strafe perfectly left; signs are wrong on either fudgefactor (multiplied instead of divided) or the 1-() and 1+() in powers
    public void strafeStraightRight() {
        double powerFL = .5, powerFR = -.5, powerBL = -.5, powerBR = .5;
        headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0); //.02

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

        telemetrize();
    }

    // Turn a specified number of degrees, using the gyro sensor
    public void turnDegrees(int degrees) {
        int initialHeading = gyro.getIntegratedZValue();
        double power;

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(gyro.getIntegratedZValue() - degrees) >= TURN_DEG_TOLERANCE && opModeIsActive()) {
            power = (TURN_SENSITIVITY * (double)(gyro.getIntegratedZValue() - initialHeading - degrees));
            if (Math.abs(power) < 0.15 && power != 0) {
                power = 0.15 * (power / Math.abs(power));
            }
            Range.clip(power, -1, 1);

            setLeftDrivePower(power);
            setRightDrivePower(-power);
            telemetrize();
        }

        stopDriveMotors();
        telemetrize();
    }

    // Rotate bot to be parallel with wall, 180 degrees from starting position
    public void turnToFaceWall() {
        turnDegrees (startingDegrees - gyro.getIntegratedZValue() + 180);
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

    public boolean isLeftBlue() {
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

        if (leftBlueAvg > leftRedAvg) {
            return true;
        }
        else {
            return false;
        }
    }

    // Find average amount of red and blue on right phone window
    public boolean isRightBlue() {
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
        rightRedAvg = rightRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
        rightBlueAvg = rightBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));

        if (rightBlueAvg > rightRedAvg) {
            return true;
        }
        else {
            return false;
        }
    }

    // Align camera with an x value from the beacon image
    public void alignCameraWithImageX(int targetX) {

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double xDistance = translation.get(0);
                double alignmentPower;

                while((Math.abs(xDistance - targetX)) > X_ALIGNMENT_TOLERANCE && opModeIsActive()) {
                    alignmentPower = Range.clip(((xDistance - targetX) * X_ALIGNMENT_SENSITIVITY), -1, 1);

                    if (Math.abs(alignmentPower) < 0.1 && alignmentPower != 0) {
                        alignmentPower = 0.1 * (alignmentPower / Math.abs(alignmentPower));
                    }

                    setAllDrivePower(alignmentPower);

                    pose = myListener.getPose();
                    translation = pose.getTranslation();
                    xDistance = translation.get(0);

                    telemetry.addData(beac.getName() + "xDistance", xDistance);
                    telemetrize();
                }

                break;
            }
        }

        stopDriveMotors();
    }

    public void checkColorConfiguration(){
        grabFrame();

        if (!isLeftBlue() && isRightBlue()) {
            colorConfig = State.RED_BLUE;
        }
        else if (isLeftBlue() && !isRightBlue()) {
            colorConfig = State.BLUE_RED;
        }
        else {
            if (leftBlueAvg < rightBlueAvg) {
                colorConfig = State.RED_BLUE;
            }
            else {
                colorConfig = State.BLUE_RED;
            }
        }
    }

    public void showColorConfigurationToDebuggingCoder() {
        telemetrize();
        sleep(30000);
    }

    public void alignBopperWithButtons() {
        if (colorConfig == State.BLUE_RED) {
            //inch forward (because we're red)
            //align with x = 0
            alignCameraWithImageX(0);

        }
        else { //RED_BLUE or
            //inch back (because we're red)
            //align with x = ~140
            alignCameraWithImageX(140);
        }
    }

    public void strafeToBeacon() {
        // We're red, so we'll be strafing right

        VuforiaTrackableDefaultListener myListener;

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        setInitialHeading();

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double zDistance = Math.abs(translation.get(2));
                beaconStrafeTicks = zDistance * Z_TO_STRAFE_SCALE;

                timeMark = runTime.time();

                while(getStrafeEncAvg() < beaconStrafeTicks && (runTime.time() - timeMark) < 4 &&opModeIsActive()) {

                    strafeStraightRight();

                    telemetrize();
                }

                break;
            }
        }
    }

    public void strafeAwayFromBeacon() {
        // We're red, so we need to strafe left to get away from the beacon
        strafeStraightLeft();
    }


}


