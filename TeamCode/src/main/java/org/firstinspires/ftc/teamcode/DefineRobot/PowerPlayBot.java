package org.firstinspires.ftc.teamcode.DefineRobot;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PowerPlayBot {

    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public DcMotorEx frontLeft   = null;
    public DcMotorEx frontRight  = null;
    public DcMotorEx backLeft    = null;
    public DcMotorEx backRight   = null;
    public DcMotorEx slideRight  = null;
    public DcMotorEx slideLeft   = null;
    public DcMotorEx turret      = null;
    public Servo Claw            = null;
    public NormalizedColorSensor colorSensor;
    public NormalizedRGBA colors;
    public BNO055IMU imu         = null;  // Control Hub IMU
    Orientation lastAngles = new Orientation();
    double globalAngle, startAngle, endAngle, currentAngle;
    public VoltageSensor batteryVoltageSensor;

    // Variables for using the IMU for heading-based robot driving
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public double          robotHeading  = 0;
    public double          headingOffset = 0;
    public double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    public double  targetHeading = 0;
    public double  driveSpeed    = 0;
    public double  turnSpeed     = 0;
    public double  leftSpeed     = 0;
    public double  rightSpeed    = 0;
    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double drive1 = 0.0;
    public double drive2 = 0.0;
    public double turn1 = 0.0;
    public double turn2 = 0.0;
    public int     frontLeftTarget  = 0;
    public int     frontRightTarget = 0;
    public int     backLeftTarget   = 0;
    public int     backRightTarget  = 0;
    public int startMotorCounts = 0;
    public int stopMotorCounts = 0;


    // Constants for the goBilda Drivetrain: 18108-A
    /*
    public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;   // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     STANDARD_COUNTS_PER_INCH    = 166.667;
    public static final double     STRAFE_COUNTS_PER_INCH      = 166.667;
    public static final int        STANDARD_COUNTS_PER_1_TILE  = 4000;
    public static final int        STRAFE_COUNTS_PER_1_TILE    = 3600;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.1;     // Max driving speed for better distance accuracy.
    // public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    //public static final double     DRIVE_SPEED             = 0.8;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 0.5;     // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
     */

    // Constants for the TETRIX Drivetrain: 18108-C
    public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;   // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     STANDARD_COUNTS_PER_INCH    = 166.667;
    public static final double     STRAFE_COUNTS_PER_INCH      = 166.667;
    public static final int        STANDARD_COUNTS_PER_1_TILE  = 1075;
    public static final int        STRAFE_COUNTS_PER_1_TILE    = 3600;
    public static final double     WHEEL_COUNTS_PER_INCH   = 142.3636363625;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.8;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.5;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 0.5;     // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Variables for GamePad1 button states
    public boolean gp1ButtonACurrentState = false;
    public boolean gp1ButtonALastState = false;
    public boolean gp1ButtonBCurrentState = false;
    public boolean gp1ButtonBLastState = false;
    public boolean gp1ButtonXCurrentState = false;
    public boolean gp1ButtonXLastState = false;
    public boolean gp1ButtonYCurrentState = false;
    public boolean gp1ButtonYLastState = false;
    public boolean gp1DpadUpCurrentState = false;
    public boolean gp1DpadUpLastState = false;
    public boolean gp1DpadRightCurrentState = false;
    public boolean gp1DpadRightLastState = false;
    public boolean gp1DpadDownCurrentState = false;
    public boolean gp1DpadDownLastState = false;
    public boolean gp1DpadLeftCurrentState = false;
    public boolean gp1DpadLeftLastState = false;
    public boolean gp1LeftBumperCurrentState = false;
    public boolean gp1LeftBumperLastState = false;
    public boolean gp1RightBumperCurrentState = false;
    public boolean gp1RightBumperLastState = false;

    // Variables for GamePad2 button states
    public boolean gp2ButtonACurrentState = false;
    public boolean gp2ButtonALastState = false;
    public boolean gp2ButtonBCurrentState = false;
    public boolean gp2ButtonBLastState = false;
    public boolean gp2ButtonXCurrentState = false;
    public boolean gp2ButtonXLastState = false;
    public boolean gp2ButtonYCurrentState = false;
    public boolean gp2ButtonYLastState = false;
    public boolean gp2DpadUpCurrentState = false;
    public boolean gp2DpadUpLastState = false;
    public boolean gp2DpadRightCurrentState = false;
    public boolean gp2DpadRightLastState = false;
    public boolean gp2DpadDownCurrentState = false;
    public boolean gp2DpadDownLastState = false;
    public boolean gp2DpadLeftCurrentState = false;
    public boolean gp2DpadLeftLastState = false;
    public boolean gp2LeftBumperCurrentState = false;
    public boolean gp2LeftBumperLastState = false;
    public boolean gp2RightBumperCurrentState = false;
    public boolean gp2RightBumperLastState = false;

    // Constants define movement of claw; Min and Max Positions
    public static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    public static final int    CYCLE_MS    =   30;     // period of each cycle

    public static final double AMAX_POS = 1.4;     // Maximum rotational position ---- Phil Claw: 1.4; GoBilda Claw: 1.4
    public static final double AMIN_POS = 0.7;     // Minimum rotational position ---- Phil Claw: 0.7; GoBilda Claw: 0.61
    public double  Aposition = AMIN_POS;                 // Start position

    public static final double BMAX_POS     =  1.00;     // Maximum rotational position
    public static final double BMIN_POS     =  0.50;     // Minimum rotational position
    public double  Bposition = BMIN_POS;                 // Start position

    public static final double CLAW_OPEN_SETTING = AMAX_POS;
    public static final double CLAW_CLOSED_SETTING = AMIN_POS;

    // Constants that define junction height at tile intersections in units of linear slide motor encoder counts
    public static final int     JUNCTION_HIGH             = 2400;    // Height of junctions - highest
    public static final int     JUNCTION_MEDIUM           = 1600;    // Height of junctions - medium
    public static final int     JUNCTION_LOW              = 800;     // Height of junctions - shortest
    public static final int     JUNCTION_GROUND           = 100;     // Height of junctions with no pole

    // Linear Slide encoder count height for the Stack of 5 cones
    public static final int     STACK_CONE_5             = 180;
    public static final int     STACK_CONE_4             = 160;
    public static final int     STACK_CONE_3             = 140;
    public static final int     STACK_CONE_2             = 120;
    public static final int     STACK_CONE_1             = 100;

    // Coordinates of the four robot starting positions
    public final int     RED_RIGHT_START_COL       = 0;       // Red Right starting X
    public final int     RED_RIGHT_START_ROW       = 2;       // Red Right starting Y
    public final int     RED_LEFT_START_COL        = 0;       // Red Left starting X
    public final int     RED_LEFT_START_ROW        = 8;       // Red Left starting Y
    public final int     BLUE_LEFT_START_COL       = 10;      // Blue Left starting X
    public final int     BLUE_LEFT_START_ROW       = 2;       // Blue Left starting Y
    public final int     BLUE_RIGHT_START_COL      = 10;      // Blue Right starting X
    public final int     BLUE_RIGHT_START_ROW      = 8;       // Blue Right starting Y

    // Coordinates of the four Alliance Terminals
    public final int     RED_FRONT_TERMINAL_COL    = 10;      // Red Right starting X
    public final int     RED_FRONT_TERMINAL_ROW    = 0;       // Red Right starting Y
    public final int     RED_BACK_TERMINAL_COL     = 0;       // Red Left starting X
    public final int     RED_BACK_TERMINAL_ROW     = 10;      // Red Left starting Y
    public final int     BLUE_FRONT_TERMINAL_COL   = 0;       // Blue Left starting X
    public final int     BLUE_FRONT_TERMINAL_ROW   = 0;       // Blue Left starting Y
    public final int     BLUE_BACK_TERMINAL_COL    = 10;      // Blue Right starting X
    public  final int     BLUE_BACK_TERMINAL_ROW    = 10;      // Blue Right starting Y

    // Coordinates of the four stacks of 5 cones
    public final int     RED_FRONT_CONE_STACK_COL  = 4;       // Red Front cone stack X
    public final int     RED_FRONT_CONE_STACK_ROW  = 0;       // Red Front cone stack Y
    public final int     RED_BACK_CONE_STACK_COL   = 4;       // Red Back cone stack X
    public final int     RED_BACK_CONE_STACK_ROW   = 10;      // Red Back cone stack Y
    public final int     BLUE_FRONT_CONE_STACK_COL = 6;       // Blue Right starting X
    public final int     BLUE_FRONT_CONE_STACK_ROW = 0;       // Red Right starting Y
    public final int     BLUE_BACK_CONE_STACK_COL  = 6;       // Blue Right starting X
    public final int     BLUE_BACK_CONE_STACK_ROW  = 10;      // Blue Right starting Y

    // Define X/Y coordinate system to represent the tiles and junctions on the PowerPlay field
    // All even coordinates are for tile centers where the robot can drive without colliding with junctions
    // All odd coordinates are for tile corners that hold junctions, the integer stored at coordinates is the junction height in motor encoder counts
    public static final int numGridRows = 10;
    public static final int numGridCols = 10;
    public int[][] grid = new int[numGridRows][numGridCols];

    // Variables to store current Robot position on the field
    public int currentBotRow = 0;
    public int currentBotCol = 0;
    public double startingHeading;
    public double currentHeading = 0.0;

    public enum Alliance {
        RED,
        BLUE,
    }
    public Alliance currentAlliance = Alliance.RED;

    private static final String TAG = "PowerPlayBot";

    public PowerPlayBot(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    }


    // **************************************************************
    // **********  Initialization functions  ************************
    // **************************************************************


    public void initGridValues() {
        // Initialize ground-level junctions
        grid[1][1] = grid[1][5] = grid[1][9] = JUNCTION_GROUND;  // Row 1
        grid[5][1] = grid[5][5] = grid[5][9] = JUNCTION_GROUND;  // Row 5
        grid[9][1] = grid[9][3] = grid[9][5] = JUNCTION_GROUND;  // Row 9

        // Initialize low height junctions
        grid[1][3] = grid[1][7] = JUNCTION_LOW;                  // Row 1
        grid[3][1] = grid[3][9] = JUNCTION_LOW;                  // Row 3
        grid[7][1] = grid[7][9] = JUNCTION_LOW;                  // Row 7
        grid[9][3] = grid[9][7] = JUNCTION_LOW;                  // Row 9

        // Initialize medium height junctions
        grid[3][3] = grid[3][7] = JUNCTION_MEDIUM;               // Row 3
        grid[7][3] = grid[7][7] = JUNCTION_MEDIUM;               // Row 7

        // Initialize high height junctions
        grid[3][5] = JUNCTION_HIGH;                              // Row 3
        grid[5][3] = grid[5][7] = JUNCTION_HIGH;                 // Row 5
        grid[7][5] = JUNCTION_HIGH;                              // Row 7
    }

    public void init() throws Exception {
        // Initialize all robot components
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        Claw = hardwareMap.get(Servo.class, "Claw");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize values for IMU
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        try {
            // initCalibData();
            initGridValues();
            resetEncoders();
            //resetHeading();
            startingHeading = robotHeading;

            // Motor directions for the goBilda bot: 18108-A
            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (turret != null) {
                turret.setDirection(DcMotor.Direction.FORWARD);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            /*
            // Motor directions for the Tetrix bot: 18108-C
            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
             */

            stop();
        }
        catch (Exception ex) {
            //issues accessing drive resources
            Log.e(TAG, "Init error", ex);
            throw new Exception("Issues accessing hardware on your bot. Check Configure Robot", ex);
        }
    }

    public void resetEncoders() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void gamepadSetCurrentBotCoordinates() {

        while (!opMode.gamepad1.right_stick_button) {

            // Buttons for +/- X Coord and +/- Y Coord
            gp1ButtonXLastState = gp1ButtonXCurrentState;
            gp1ButtonXCurrentState = opMode.gamepad1.x;
            gp1ButtonYLastState = gp1ButtonYCurrentState;
            gp1ButtonYCurrentState = opMode.gamepad1.y;
            gp1ButtonBLastState = gp1ButtonBCurrentState;
            gp1ButtonBCurrentState = opMode.gamepad1.b;
            gp1ButtonALastState = gp1ButtonACurrentState;
            gp1ButtonACurrentState = opMode.gamepad1.a;

            // Increment X Coord each time gampepad1 B button is pressed
            if (gp1ButtonBCurrentState && !gp1ButtonBLastState) {
                if (currentBotCol < numGridCols) {
                    currentBotCol++;
                }
            }
            // Decrement X Coord each time gampepad1 X button is pressed
            if (gp1ButtonXCurrentState && !gp1ButtonXLastState) {
                if (currentBotCol > 0) {
                    currentBotCol--;
                }
            }
            // Increment Y Coord each time gampepad1 Y button is pressed
            if (gp1ButtonYCurrentState && !gp1ButtonYLastState) {
                if (currentBotRow < numGridRows) {
                    currentBotRow++;
                }
            }
            // Decrement Y Coord each time gampepad1 A button is pressed
            if (gp1ButtonACurrentState && !gp1ButtonALastState) {
                if (currentBotRow > 0) {
                    currentBotRow--;
                }
            }

            opMode.telemetry.addData("Bot Coordinates", "X: %2d Y: %2d", currentBotCol, currentBotRow);
            opMode.telemetry.update();
        }
    }


    // **************************************************************
    // **********  HIGH Level driving functions  ********************
    // **************************************************************

    /**
     *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * STANDARD_COUNTS_PER_INCH);
            frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = backRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opMode.opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        opMode.telemetry.update();

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public void strafeRight(double maxDriveSpeed,
                           double distance) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            /*
            opMode.telemetry.addData("In strafeRight", " ");
            opMode.telemetry.addData("Parameter: ", "mDS: %0.2f", maxDriveSpeed);
            opMode.telemetry.addData("Parameter: ", "dist: %0.2f", distance);
            opMode.telemetry.update();
            opMode.sleep(5000);
             */

            startMotorCounts = frontLeft.getCurrentPosition();
            stopMotorCounts = startMotorCounts + (int)(distance * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = -1.0;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            frontLeft.setPower(rightPower);
            frontRight.setPower(leftPower);
            backLeft.setPower(leftPower);
            backRight.setPower(rightPower);

            // Stop when the bot has driven the requested distance
            while (frontLeft.getCurrentPosition() <= stopMotorCounts) {
            }
            stop();
        }
    }

    public void strafeLeft(double maxDriveSpeed,
                           double distance) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            startMotorCounts = frontLeft.getCurrentPosition();
            stopMotorCounts = startMotorCounts - (int)(distance * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = 1.0;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            frontLeft.setPower(rightPower);
            frontRight.setPower(leftPower);
            backLeft.setPower(leftPower);
            backRight.setPower(rightPower);

            // Stop when the bot has driven the requested distance
            while (frontLeft.getCurrentPosition() >= stopMotorCounts) {
            }
            stop();
        }
    }


    /**
     *  Method for mecanum strafing in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void strafeStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (heading == -90.0) {  // Strafing to the right
                frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                frontRightTarget = frontRight.getCurrentPosition() - moveCounts;
                backLeftTarget = backLeft.getCurrentPosition() - moveCounts;
                backRightTarget = backRight.getCurrentPosition() + moveCounts;
            } else {
                if (heading == 90.0) {  // Strafing to the left
                    frontLeftTarget = frontLeft.getCurrentPosition() - moveCounts;
                    frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                    backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
                    backRightTarget = backRight.getCurrentPosition() - moveCounts;
                }
            }

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            strafeRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opMode.opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                strafeRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void setCurrentBotCoordinates() {}

    public void driveToCoordinate(int targetBotCol, int targetBotRow) {
        int changeCols;
        int changeRows;

        // Algorithm: Drive away from Alliance wall first (change Columns/X), then Strafe to Front or Back (change Rows/Y)

        // Determine how many Rows & Columns needed to move
        changeCols = targetBotCol - currentBotCol;
        changeRows = targetBotRow - currentBotRow;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine if still facing the starting Heading for either Alliance color
            // Use that to determine which directions are positive & negative to move Rows & Columns
            if (currentHeading == 0.0) {  // Bot has not turned from starting Heading
                if (currentAlliance == Alliance.RED) {

                    // Change COLUMNS
                    // Check if moving in positive or negative Column / X direction
                    if (changeCols > 0) {
                        driveStraight(DRIVE_SPEED,changeCols*12.0,0.0);
                        currentBotCol += changeCols;
                        holdHeading(TURN_SPEED, 0.0, 0.25);       // Hold 0 Deg heading for 0.25 second
                    } else if (changeCols < 0) {
                        driveStraight(DRIVE_SPEED,changeCols*12.0,0.0);
                        currentBotCol += changeCols;
                        holdHeading(TURN_SPEED, 0.0, 0.25);       // Hold 0 Deg heading for 0.25 second
                    }

                    // Change ROWS
                    // Check if moving in positive or negative Rows / Y direction

                    if (changeRows > 0) {
                        strafeLeft(DRIVE_SPEED,changeRows*12.0);
                        currentBotRow += changeRows;
                        holdHeading(TURN_SPEED, 0.0, 0.5);       // Hold 0 Deg heading for a 0.25 second
                    } else if (changeRows < 0) {
                        strafeRight(DRIVE_SPEED,changeRows*-12.0);
                        currentBotRow += changeRows;
                        holdHeading(TURN_SPEED, 0.0, 0.5);       // Hold 0 Deg heading for a 0.25 second
                    }

                } else {  // Alliance is BLUE, positive & negative directions are swapped compared to RED

                    // Change COLUMNS
                    // Check if moving in positive or negative Column / X direction
                    if (changeCols > 0) {
                        driveStraight(DRIVE_SPEED,changeCols*12.0,0.0);
                        currentBotCol += changeCols;
                        holdHeading(TURN_SPEED, 0.0, 0.25);       // Hold 0 Deg heading for 0.25 second
                    } else if (changeCols < 0) {
                        driveStraight(DRIVE_SPEED,changeCols*-12.0,0.0);
                        currentBotCol += changeCols;
                        holdHeading(TURN_SPEED, 0.0, 0.25);       // Hold 0 Deg heading for 0.25 second
                    }

                    // Change ROWS
                    // Check if moving in positive or negative Rows / Y direction
                    if (changeRows > 0) {
                        strafeRight(DRIVE_SPEED,changeRows*12.0);
                        currentBotRow += changeRows;
                        holdHeading(TURN_SPEED, 0.0, 0.5);       // Hold 0 Deg heading for a 0.25 second
                    } else if (changeRows < 0) {
                        strafeLeft(DRIVE_SPEED,changeRows*12.0);
                        currentBotRow += changeRows;
                        holdHeading(TURN_SPEED, 0.0, 0.5);       // Hold 0 Deg heading for a 0.25 second
                    }
                }
            }
        }
    }


    public void switchDriveToJunction() {}

    public void switchJunctionToDrive() {}


    // **************************************************************
    // **********  LOW Level driving functions  *********************
    // **************************************************************


    public void stop() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            this.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        //robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void strafeRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backLeft.setPower(rightSpeed);
        backRight.setPower(leftSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    public void sendTelemetry(boolean straight) {

        if (straight) {
            opMode.telemetry.addData("Motion", "Drive Straight");
            opMode.telemetry.addData("Target Front Pos L:R",  "%7d:%7d",      frontLeftTarget, backRightTarget);
            opMode.telemetry.addData("Target Back  Pos L:R",  "%7d:%7d",      backLeftTarget, frontRightTarget);
            opMode.telemetry.addData("Actual Front Pos L:R",  "%7d:%7d",      frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            opMode.telemetry.addData("Actual Back  Pos L:R",  "%7d:%7d",      backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
        } else {
            opMode.telemetry.addData("Motion", "Turning");
        }

        opMode.telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        opMode.telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        opMode.telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        opMode.telemetry.update();
    }


    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    /*
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    /*
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    */

    // **************************************************************
    // **********  Claw functions  **********************************
    // **************************************************************

    public void openClaw() {
        Claw.setPosition(CLAW_OPEN_SETTING);
        opMode.sleep(1000);
    }

    public void closeClaw() {
        Claw.setPosition(CLAW_CLOSED_SETTING);
        opMode.sleep(1000);
    }

    public void clawPosition() {

        if (opMode.gamepad2.left_bumper) {
            Claw.setPosition(AMIN_POS);
        }
        if (opMode.gamepad2.right_bumper) {
            Claw.setPosition(AMAX_POS);
        }
    }


    // **************************************************************
    // **********  Linear Slide functions  **************************
    // **************************************************************

    public void moveSlides() {
        double slidePower = Range.clip(opMode.gamepad2.right_stick_y, -1.0, 1.0);

        slideLeft.setPower(-0.7 * slidePower);
        slideRight.setPower(0.7 * slidePower);
    }

    public void moveSlidesToHeight(int motorSlideEncoderCounts) {

        slideLeft.setTargetPosition(motorSlideEncoderCounts);
        slideRight.setTargetPosition(-motorSlideEncoderCounts);

        slideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void driveStraightGyro(double inchesToDrive, double drivePower) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction = 0.02;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();

        opMode.sleep(250);

        if (motorDistance >= 0) {
            // Start driving forward
            frontLeft.setPower(0.25);
            frontRight.setPower(0.25);
            backLeft.setPower(0.25);
            backRight.setPower(0.25);

            while (true) {
                // telemetry.addData("frontLeft",  "Distance: %3d", frontLeft.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                frontLeft.setPower(power - correction);
                frontRight.setPower(power + correction);
                backLeft.setPower(power - correction);
                backRight.setPower(power + correction);

                //slideLeft.setPower(slidePower);
                //slideRight.setPower(-slidePower);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        frontLeft.getCurrentPosition(),
                        -frontRight.getCurrentPosition(),
                        -backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                opMode.telemetry.addData("slideHeight", slideLeft.getCurrentPosition());
                opMode.telemetry.update();

                /* if ((slidePower > 0 && slideLeft.getCurrentPosition() >= slideHeight) || (slidePower < 0 && slideLeft.getCurrentPosition() <= slideHeight)){
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                 */

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) +
                        Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) / 4.0
                        >= motorDistance)) {
                    frontLeft.setPower(0.0);
                    frontRight.setPower(0.0);
                    backLeft.setPower(0.0);
                    backRight.setPower(0.0);
                    break;
                }
            }
        }
        if (motorDistance < 0) {
            // Start driving backward
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(-power);

            while (true) {
                // telemetry.addData("frontLeft",  "Distance: %3d", frontLeft.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                frontLeft.setPower(-1 * (power + correction));
                frontRight.setPower(-1 * (power - correction));
                backLeft.setPower(-1 * (power + correction));
                backRight.setPower(-1 * (power - correction));

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                opMode.telemetry.update();

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) +
                        Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) / 4.0
                        >= Math.abs(motorDistance))) {
                    frontLeft.setPower(0.0);
                    frontRight.setPower(0.0);
                    backLeft.setPower(0.0);
                    backRight.setPower(0.0);
                    break;
                }
            }
        }
    }


    public void coneStackDetect(boolean isRed, boolean isLeft) { //120-180 gray, 12-60 red, 210-225 blue
        float[] hsvValues = new float[3];
        int leftEdge = 0;
        int rightEdge = 0;



        colors = colorSensor.getNormalizedColors();
        if ((isRed && !isLeft) || (!isRed && isLeft)) { //red right and blue left
            driveStraightGyro(3,0.25);
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] < 110 || hsvValues[0] > 190) {
                rightEdge = frontLeft.getCurrentPosition();
            } if (hsvValues[0] > 110 && hsvValues[0] < 190) {
                leftEdge = frontLeft.getCurrentPosition();
            }
            double avgPos = (double)(leftEdge + rightEdge)/2;
            strafeRight(0.5, 12);
        } else { //red left and blue right
            driveStraightGyro(3,0.25);
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] < 110 || hsvValues[0] > 190) {
                rightEdge = frontLeft.getCurrentPosition();
            } if (hsvValues[0] > 110 && hsvValues[0] < 190) {
                leftEdge = frontLeft.getCurrentPosition();
            }
            double avgPos = (double)(leftEdge + rightEdge)/2;
            strafeLeft(0.5, 12);
        }


    }



}


