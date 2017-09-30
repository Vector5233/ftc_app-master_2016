/**
 * Created Dec. 5, 2016 by Wil Orlando.
 *
 * This is the main autonomous for 5233 Vector's red start.
 *
 * This program makes use of Vuforia, along with the RC phone's camera, to find and process beacons
 * on the field.
 *
 * It scores 90 points overall: two 15-pt particles in the center vortex, and two 30-pt beacons.
 *
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

@Autonomous(name="VopperAutoRed", group="MyGroup")
public class VopperAutoRed extends LinearOpMode{

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector, bopper;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Drive variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    final double HEADING_CORRECTION_SCALE = 3, FUDGE_FACTOR = 1.10;
    double DRIVE_SENSITIVITY = 0.002;

    // Turn variables
    int startingDegrees = 0;
    int TURN_DEG_TOLERANCE = 1;
    double TURN_SENSITIVITY = .05/(double)TURN_DEG_TOLERANCE;

    // Flicker targeting variables
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181

    // Start a timer, for any time-related functions
    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0;

    // Vuforia window bounds (Area checked for red/blue)
    int WINDOW_X_MIN = 600, WINDOW_X_MAX = 1200;
    int WINDOW_LEFT_Y = 719, WINDOW_LEFT_MID_Y = 360, WINDOW_RIGHT_MID_Y = 360, WINDOW_RIGHT_Y = 0;

    /*
    int WINDOW_X_MIN = 900, WINDOW_X_MAX = 1100;
    int WINDOW_LEFT_Y = 720, WINDOW_LEFT_MID_Y = 480, WINDOW_RIGHT_MID_Y = 240, WINDOW_RIGHT_Y = 0;
    */

    // Vuforia image alignment
    double X_ALIGNMENT_TOLERANCE = 3;
    double X_ALIGNMENT_SENSITIVITY =.001;
    boolean imageFound = false;
    double ALIGN_MIN_POWER = 0.10;
    String firstBeacon = "";

    // Vuforia color configuration check
    int leftRedAvg = 0, leftBlueAvg = 0, rightRedAvg = 0, rightBlueAvg = 0;
    int leftRed = 0, leftBlue = 0, rightRed = 0, rightBlue = 0;
    int redAvg = 0, blueAvg = 0;
    enum State {BLUE_RED, RED_BLUE, DEBUG}
    State colorConfig;

    // Vuforia misc.
    long numImages = 0;
    Image rgb = null;
    Bitmap bm = null;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double xDistance = 0, zDistance = 0;

    int LEFT_BEACON_MM = -98, RIGHT_BEACON_MM = -230;
    int beaconAlignTicks = 0;

    // General misc.
    double driveEncAvg = 0, leftEncAvg = 0, rightEncAvg = 0;
    double BOP_OVERKILL = 100;
    double currentTime = 0;
    String opState = "";


    public void runOpMode() {

        initialize();

        telemetrize();

        waitForStart();

        beacons.activate();

        while(opModeIsActive()) {

            startingDegrees = gyro.getIntegratedZValue();

            op("Adjusting for flick...");
            driveForTicks(300, .3);
            superTurn(0);

            op("Flicking...");
            flickTwice();

            op("First drive...");
            driveForTicks(1200, .5);

            op("First turn...");
            broadTurn(60, 1);
            superTurn(60);

            op("Second drive...");
            driveForTicks(4200, 1);

            op("Turning to face wall...");
            turnToFaceWall();

            op("Sleeping...");
            sleep(200);

            op("Aligning with image...");
            alignCameraWithImageX(0);

            op("Verifying image found...");
            imageSafety();

            op("Checking beacon color...");
            checkColorConfiguration();

            op("Aligning with button...");
            alignBopperWithButtons();

            op("Bopping button...");
            bopButton(8);

            op("Realigning with wall...");
            superTurn(180);

            op("Driving to second beacon...");
            driveToSecondBeacon();

            op("Realigning with wall...");
            superTurn(180);

            op("Sleeping...");
            sleep(200);

            op("Aligning with second image...");
            alignCameraWithImageX(0);

            op("Verifying second image found...");
            imageSafety();

            op("Checking second beacon color...");
            checkColorConfiguration();

            op("Aligning with second button...");
            alignBopperWithButtons();

            op("Bopping second button...");
            bopButton(12);

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
        bopper = hardwareMap.dcMotor.get("bopper");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Reset drive encoders
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setDirection(DcMotor.Direction.REVERSE);
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Set max speed for flicker, to improve consistency
        //flicker.setMaxSpeed(4000);

        // Set gate to be closed at the start of the match
        gate.setPosition(0);

        // Reset bopper encoder
        bopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        timeMark = runTime.time();

        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {
            idle();
        }

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
        telemetry.addData("OpState: ", opState);
        telemetry.addData("CurrentTime: ", currentTime);
        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
        telemetry.addData("What even", opState);
        telemetry.addData("blueAvgPos: ", blueAvg);
        telemetry.addData("redAvgPos: ", redAvg);
        //telemetry.addData("Drive Enc Avg: ", driveEncAvg);
        //telemetry.addData("startingDegrees:", startingDegrees);

        //telemetry.addData("Enc FR: ", frontRight.getCurrentPosition());
        //telemetry.addData("Enc FL: ", frontLeft.getCurrentPosition());
        //telemetry.addData("Enc BR: ", backRight.getCurrentPosition());
        //telemetry.addData("Enc BL: ", backLeft.getCurrentPosition());
        //telemetry.addData("FR power", frontRight.getPower());

        telemetry.addData("Color Config: ", colorConfig);
        telemetry.addData("xDistance: ", xDistance);
        telemetry.addData("zDistance: ", zDistance);
        telemetry.addData("Bopper Enc: ", bopper.getCurrentPosition());
        //telemetry.addData("leftBlueAvg: ", leftBlueAvg);
        //telemetry.addData("leftRedAvg: ", leftRedAvg);
        //telemetry.addData("rightBlueAvg: ", rightBlueAvg);
        //telemetry.addData("rightRedAvg: ", rightRedAvg);
        telemetry.update();
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
    public void driveForTicks(int encTicks, double power) {

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        /*double encDiff;

        while ((Math.abs(getAllDriveEncAvg()) < encTicks) && opModeIsActive()) {

            encDiff = (getLeftDriveEncAvg() - getRightDriveEncAvg()) * Math.signum(power) * DRIVE_SENSITIVITY;
            telemetry.addData("encDiff: ", encDiff);
            telemetrize();
            Range.clip(encDiff, -1, 1);

            setLeftDrivePower(power - encDiff);
            setRightDrivePower(power + encDiff);

        }*/

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
            Range.clip(headingDiffPercent, -.1, .1);

            if (power > 0) {
                rightPower = power - (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(rightPower, ALIGN_MIN_POWER , 1);
                leftPower = power + (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(leftPower, ALIGN_MIN_POWER , 1);

                setRightDrivePower(rightPower);
                setLeftDrivePower(leftPower);
            }
            else {
                rightPower = power + (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(rightPower, -1 , -ALIGN_MIN_POWER);
                leftPower = power - (HEADING_CORRECTION_SCALE * headingDiffPercent);
                Range.clip(leftPower, -1 , -ALIGN_MIN_POWER);

                setRightDrivePower(rightPower);
                setLeftDrivePower(leftPower);
            }


        }

        stopDriveMotors();
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
    }

    // Turn a specified number of degrees (relative to the starting position, using the gyro sensor
    public void turnDegrees(int degrees) {
        int initialHeading = gyro.getIntegratedZValue();
        double power;
        double absDegrees = startingDegrees - gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(gyro.getIntegratedZValue() - initialHeading - absDegrees) >= TURN_DEG_TOLERANCE && opModeIsActive()) {
            power = (TURN_SENSITIVITY * (double)(gyro.getIntegratedZValue() - initialHeading - absDegrees));
            if (Math.abs(power) < 0.2 && power != 0) {
                power = 0.2 * (power / Math.abs(power));
            }
            else if (Math.abs(power) > 0.4 && power != 0) {
                power = 0.4 * (power / Math.abs(power));
            }


            Range.clip(power, -1, 1);

            setLeftDrivePower(power);
            setRightDrivePower(-power);
            telemetrize();
        }

        stopDriveMotors();
        telemetrize();
        sleep(100);

        superTurn(degrees);
    }

    // Broad turn function with encoders, for turns with a wide margin of error
    public void broadTurn (int degrees, double power) {
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        // Making degrees absolute (measured from the starting position)
        degrees += startingDegrees - gyro.getIntegratedZValue();

        // 360 degrees (one full bot rotation) is about 6100 ticks for each motor
        double ticks = (6100/360) * degrees;
        double motorEncAvg = 0;
        if (ticks != 0) {
            // Positive ticks means left, negative means right

            power *= (Math.signum(ticks));
        }

        while (opModeIsActive() && motorEncAvg < Math.abs(ticks)) {
            setLeftDrivePower(-power);
            setRightDrivePower(power);

            motorEncAvg = ((Math.abs(frontLeft.getCurrentPosition()) +
                            Math.abs(frontRight.getCurrentPosition()) +
                            Math.abs(backLeft.getCurrentPosition()) +
                            Math.abs(backRight.getCurrentPosition()))/4);
        }

        stopDriveMotors();
        sleep(100);
    }

    // More precise turn function with gyro, for use in turns that must be within a degree
    public void superTurn (int degrees) {
        int initialHeading = gyro.getIntegratedZValue();
        double power;

        degrees = startingDegrees - gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(gyro.getIntegratedZValue() - initialHeading - degrees) >= 1 && opModeIsActive()) {
            power = (0.01 * (double)(gyro.getIntegratedZValue() - initialHeading - degrees));
            if (Math.abs(power) < 0.25 && power != 0) {
                power = 0.25 * (power / Math.abs(power));
            }
            else if (Math.abs(power) > 0.4 && power != 0) {
                power = 0.4 * (power / Math.abs(power));
            }


            Range.clip(power, -1, 1);

            setLeftDrivePower(power);
            setRightDrivePower(-power);
            telemetrize();
        }

        stopDriveMotors();
        telemetrize();
        sleep(200);
    }

    // Rotate bot to be parallel with wall, 180 degrees from starting position (because we're red)
    public void turnToFaceWall() {
        broadTurn(180, 1);
        superTurn(180);
    }

    // Rotate flicker twice, to launch both loaded balls
    public void flickTwice() {
        encVal = Math.abs(flicker.getCurrentPosition());
        targetVal = encVal + 3360;

        flicker.setTargetPosition(targetVal);

        while (opModeIsActive() && flicker.getCurrentPosition() < targetVal / 2) {
            flicker.setPower(1);
        }

        gate.setPosition(.3);

        while (opModeIsActive() && flicker.getCurrentPosition() < (targetVal * 3) / 2) {
            flicker.setPower(1);
        }

        flicker.setPower(0);
    }

    // Extends bopper for specified number of ticks
    public void extendBopper(int ticks) {

        // Bopper runs 229 ticks/inch at ~13 in/sec, aka 2938 ticks/sec
        bopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double maxSeconds = (ticks / (226 * 13)) + 1;
        timeMark = runTime.time();

        while (opModeIsActive() && (bopper.getCurrentPosition() < ticks) && (runTime.time() - timeMark) < maxSeconds) {
            bopper.setPower(1);
            telemetry.addData("maxSeconds :", maxSeconds);
            telemetrize();
        }

        bopper.setPower(0);
    }

    // Retract bopper for specified number of ticks
    public void retractBopper(int ticks) {

        // Bopper runs 229 ticks/inch at ~13 in/sec, aka 2938 ticks/sec
        bopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double maxSeconds = (ticks / (226 * 13)) + 1;
        timeMark = runTime.time();

        while (opModeIsActive() && (bopper.getCurrentPosition() > -ticks) && (runTime.time() - timeMark) < maxSeconds) {
            bopper.setPower(-1);
            telemetry.addData("maxSeconds :", maxSeconds);
            telemetrize();
        }

        bopper.setPower(0);
    }

    // Super function that controls the boppage of the beacon button
    public void bopButton(int inchesOut) {

        extendBopper(inchesOut * 226);
        telemetrize();
        sleep(500);
        retractBopper(inchesOut * 3 / 4 * 226);
        telemetrize();
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
                }
            }

            if (rgb != null) {
                bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());
            }
        }
    }

    // Checks average horizontal position of red and blue pixels, and returns true if left is blue
    public boolean isLeftBlue() {
        int pixel, pixelRed, pixelBlue;
        int redPos = 0, bluePos = 0;
        int redPixels = 0, bluePixels = 0;

        if (opModeIsActive()) {
            // Check all pixels in defined window sizes, and add up horizontal positions of red ones in each
            for (int x = WINDOW_X_MIN; x < WINDOW_X_MAX; x++) {

                for (int y = WINDOW_RIGHT_Y; y < WINDOW_LEFT_Y; y++) {
                    pixel = bm.getPixel(x, y);
                    pixelRed = ((pixel) & 0x00FF0000) >> 16;
                    pixelBlue = ((pixel) & 0x000000FF);

                    if (pixelBlue < pixelRed) { //pixel is red
                        redPos += y;
                        redPixels++;
                    }
                    else if (pixelRed < pixelBlue) { //pixel is blue
                        bluePos += y;
                        bluePixels++;
                    }
                }

            }

        }

        if (redPixels == 0) { redPixels++; }
        if (bluePixels == 0) { bluePixels++; }

        redAvg = redPos / redPixels;
        blueAvg = bluePos / bluePixels;

        telemetrize();

        return (blueAvg > redAvg);
    }

    // Drive straight for a specified number of encoder ticks
    public void driveWithEnc(int encTicks, double power) {

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(getAllDriveEncAvg()) < encTicks) && opModeIsActive()) {

            setLeftDrivePower(power);
            setRightDrivePower(power);

        }

        stopDriveMotors();

        sleep(100);
    }

    // Drive until phone camera is at a specified X value (relative to the image)
    public void alignCameraWithImageX(int targetX) {
        sleep(100);

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null && !beac.getName().equals(firstBeacon)) {

                imageFound = true;
                if (firstBeacon.equals("")) {
                    firstBeacon = beac.getName();
                }

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);
                xDistance = translation.get(0);
                zDistance = Math.abs(translation.get(2));
                telemetry.addData(beac.getName() + "xDistance", xDistance);

                runMM((int)(xDistance - targetX));

                break;
            }
        }

        telemetrize();
        stopDriveMotors();
    }

    // Check to see if the phone's camera can see an image
    public void checkForImage() {
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null && !beac.getName().equals(firstBeacon)) {

                imageFound = true;

                break;
            }
        }

        telemetrize();
        stopDriveMotors();
    }

    // Runs a set number of millimeters on the field (using wheels with a radius of 5 cm)
    public void runMM(int mm) {
        // mm / 1 x 1 rot / 100pi mm x 1120 ticks / 1 rot

        int ticks = (int)((1120 * mm) / (100 * 3.14159));
        double power = ALIGN_MIN_POWER * Integer.signum(ticks);

        driveWithEnc(Math.abs(ticks), power);
    }

    // Function that displays information about a field image, for use in debugging
    public void imageTest() {
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                timeMark = runTime.time();

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double xDistance;

                while (runTime.time() - timeMark < 10 && opModeIsActive()){
                    pose = myListener.getPose();
                    translation = pose.getTranslation();
                    xDistance = translation.get(0);
                    telemetry.addData(beac.getName() + "xDistance", xDistance);
                    telemetrize();
                }

                telemetrize();
                sleep(2000);
            }
        }
    }

    // Failsafe function that searches for an image if one is not found
    public void imageSafety() {

        if (!imageFound) {
            op("Couldn't find image. Searching...");

            broadTurn(170, .3);
            op("Turned once...");
            sleep(500);
            checkForImage();


            if (!imageFound) {
                op("Turning twice...");
                broadTurn(190, .3);
                op("Turned twice...");
                sleep(500);
                checkForImage();
            }

            if (!imageFound) {
                op("Turning thrice...");
                broadTurn(180, .3);
                sleep(500);
                op("Turned thrice...");
                checkForImage();
            }

            if (!imageFound) {
                op("Search failed. Ending program.");
                stop();
            }

            superTurn(180);
            alignCameraWithImageX(0);
            op("Found it!");
        }

        imageFound = false;
    }

    // Determines which color configuration a beacon is in
    public void checkColorConfiguration(){
        grabFrame();
        boolean isLeftBlue = isLeftBlue();

        if (isLeftBlue) {
            colorConfig = State.BLUE_RED;
        }
        else {
            colorConfig = State.RED_BLUE;
        }
    }

    // Drives forward or backward to align the bopper with the correct button
    public void alignBopperWithButtons() {

        sleep(200);

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        if (colorConfig == State.BLUE_RED) {
            // We're red, so we're aiming for the left button (from the front)

            runMM(LEFT_BEACON_MM); //-98
        }
        else { //RED_BLUE or weird decision
            // Aiming for the right button (from the front)

            runMM(RIGHT_BEACON_MM); //-230
        }

        beaconAlignTicks = Math.abs((int)getAllDriveEncAvg());

        sleep(300);
    }

    // Drives from first beacon to second
    public void driveToSecondBeacon() {
        // We're red, so if RED_BLUE, we need to drive a shorter distance to the next image

        //4437 is the number of ticks from the center of one image to the center of the next
        driveForTicks((4278 - beaconAlignTicks), -1);
    }

    // Displays current action in telemetry
    public void op(String state) {
        opState = state;
        telemetrize();
    }


}