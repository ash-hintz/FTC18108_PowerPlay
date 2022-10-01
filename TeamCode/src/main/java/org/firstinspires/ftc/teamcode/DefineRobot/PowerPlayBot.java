package org.firstinspires.ftc.teamcode.DefineRobot;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public DcMotor frontLeft   = null;
    public DcMotor frontRight  = null;
    public DcMotor backLeft    = null;
    public DcMotor backRight   = null;
    public BNO055IMU imu       = null;  // Control Hub IMU

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

    // ADDED: For the TETRIX Drivetrain
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.953 ;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_COUNTS_PER_INCH  = 166.667;
    public static final int        COUNTS_PER_1_TILE_STANDARD  = 3500;
    public static final int        COUNTS_PER_1_TILE_STRAFE    = 4000;


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    // static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    // static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     DRIVE_SPEED             = 0.8;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.5;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Variables to detect if Letter buttons newly pressed or already pressed
    public boolean gp1ButtonACurrentState = false;
    public boolean gp1ButtonALastState = false;
    public boolean gp1ButtonBCurrentState = false;
    public boolean gp1ButtonBLastState = false;
    public boolean gp1ButtonXCurrentState = false;
    public boolean gp1ButtonXLastState = false;
    public boolean gp1ButtonYCurrentState = false;
    public boolean gp1ButtonYLastState = false;

    // Variables to detect if DPAD buttons newly pressed or already pressed
    public boolean gp1DpadUpCurrentState = false;
    public boolean gp1DpadUpLastState = false;
    public boolean gp1DpadRightCurrentState = false;
    public boolean gp1DpadRightLastState = false;
    public boolean gp1DpadDownCurrentState = false;
    public boolean gp1DpadDownLastState = false;
    public boolean gp1DpadLeftCurrentState = false;
    public boolean gp1DpadLeftLastState = false;

    // Constants define junction height at tile intersections in units of linear slide motor encoder counts
    public static final int     JUNCTION_HIGH             = 2400;    // Height of junctions - highest
    public static final int     JUNCTION_MEDIUM           = 1600;    // Height of junctions - medium
    public static final int     JUNCTION_LOW              = 800;     // Height of junctions - shortest
    public static final int     JUNCTION_GROUND           = 100;     // Height of junctions with no pole

    // Coordinates of the four robot starting positions
    public static final int     RED_RIGHT_START_COL       = 0;       // Red Right starting X
    public static final int     RED_RIGHT_START_ROW       = 2;       // Red Right starting Y
    public static final int     RED_LEFT_START_COL        = 0;       // Red Left starting X
    public static final int     RED_LEFT_START_ROW        = 8;       // Red Left starting Y
    public static final int     BLUE_LEFT_START_COL       = 10;      // Blue Left starting X
    public static final int     BLUE_LEFT_START_ROW       = 2;       // Blue Left starting Y
    public static final int     BLUE_RIGHT_START_COL      = 10;      // Blue Right starting X
    public static final int     BLUE_RIGHT_START_ROW      = 8;       // Blue Right starting Y

    // Coordinates of the four Alliance Terminals
    public static final int     RED_FRONT_TERMINAL_COL    = 10;      // Red Right starting X
    public static final int     RED_FRONT_TERMINAL_ROW    = 0;       // Red Right starting Y
    public static final int     RED_BACK_TERMINAL_COL     = 0;       // Red Left starting X
    public static final int     RED_BACK_TERMINAL_ROW     = 10;      // Red Left starting Y
    public static final int     BLUE_FRONT_TERMINAL_COL   = 0;       // Blue Left starting X
    public static final int     BLUE_FRONT_TERMINAL_ROW   = 0;       // Blue Left starting Y
    public static final int     BLUE_BACK_TERMINAL_COL    = 10;      // Blue Right starting X
    public static final int     BLUE_BACK_TERMINAL_ROW    = 10;      // Blue Right starting Y

    // Coordinates of the four stacks of 5 cones
    public static final int     RED_FRONT_CONE_STACK_COL  = 4;       // Red Front cone stack X
    public static final int     RED_FRONT_CONE_STACK_ROW  = 0;       // Red Front cone stack Y
    public static final int     RED_BACK_CONE_STACK_COL   = 4;       // Red Back cone stack X
    public static final int     RED_BACK_CONE_STACK_ROW   = 10;      // Red Back cone stack Y
    public static final int     BLUE_FRONT_CONE_STACK_COL = 6;       // Blue Right starting X
    public static final int     BLUE_FRONT_CONE_STACK_ROW = 0;       // Red Right starting Y
    public static final int     BLUE_BACK_CONE_STACK_COL  = 6;       // Blue Right starting X
    public static final int     BLUE_BACK_CONE_STACK_ROW  = 10;      // Blue Right starting Y

    // Define X/Y coordinate system to represent the tiles and junctions on the PowerPlay field
    // All even coordinates are for tile centers where the robot can drive without colliding with junctions
    // All odd coordinates are for tile corners that hold junctions, the integer stored at coordinates is the junction height in motor encoder counts
    public static final int numGridRows = 10;
    public static final int numGridCols = 10;
    public int[][] grid = new int[numGridRows][numGridCols];

    // Variables to store current Robot position on the field
    public int currentBotRow;
    public int currentBotCol;
    public double startingHeading;
    public double currentHeading;

    public enum Alliance {
        RED,
        BLUE,
    }
    public Alliance currentAlliance = Alliance.RED;

    private static final String TAG = "PowerPlayBot";

    public PowerPlayBot(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
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
        frontLeft = hardwareMap.get(DcMotor.class, "motor0");
        frontRight = hardwareMap.get(DcMotor.class, "motor1");
        backLeft = hardwareMap.get(DcMotor.class, "motor2");
        backRight = hardwareMap.get(DcMotor.class, "motor3");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize values for IMU
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        try {
            // initCalibData();
            initGridValues();
            resetEncoders();
            resetHeading();
            startingHeading = robotHeading;

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
            int moveCounts = (int)(distance * STRAFE_COUNTS_PER_INCH);
            frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = backRight.getCurrentPosition() + moveCounts;

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

    public void strafeRight(double maxDriveSpeed,
                           double distance) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            startMotorCounts = frontLeft.getCurrentPosition();
            stopMotorCounts = startMotorCounts + COUNTS_PER_1_TILE_STRAFE;
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
            stopMotorCounts = startMotorCounts - COUNTS_PER_1_TILE_STRAFE;
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

        opMode.telemetry.addData("Current Position", "X: %2d Y: %2d", currentBotCol, currentBotRow);
        opMode.telemetry.addData("Target Position ", "X: %2d Y: %2d", targetBotCol, targetBotRow);
        opMode.telemetry.addData("Move Values     ", "X: %2d Y: %2d", changeCols, changeRows);
        opMode.telemetry.addData("Current Alliance", "%s", currentAlliance);
        opMode.telemetry.update();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine if still facing the starting Heading for either Alliance color
            // Use that to determine which directions are positive & negative to move Rows & Columns
            if (currentHeading == 0.0) {  // Bot has not turned from starting Heading
                if (currentAlliance == Alliance.RED) {

                    // Change COLUMNS
                    // Checking if moving in positive or negative Column / X direction
                    if (changeCols > 0) {
                        for (int n = 0; n < changeCols; n++) {
                            driveStraight(DRIVE_SPEED, 24.0, 0.0);     // Drive Forward 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);     // Hold 0 Deg heading for 0.25 second
                        }
                    } else if (changeCols < 0) {
                        for (int X = 0; X < Math.abs(changeCols); X++) {
                            driveStraight(DRIVE_SPEED, -24.0, 0.0);    // Drive Forward 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);     // Hold 0 Deg heading for 0.25 second
                        }
                    }

                    // Change ROWS
                    // Checking if moving in positive or negative Rows / Y direction
                    if (changeRows > 0) {
                        for (int n = 0; n < changeRows; n++) {
                            strafeLeft(DRIVE_SPEED, 24.0);                    // Drive Left 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second
                        }
                    } else if (changeRows < 0) {
                        for (int X = 0; X < Math.abs(changeRows); X++) {
                            strafeRight(DRIVE_SPEED, 24.0);                    // Drive Left 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second
                        }
                    }

                } else {  // Alliance is BLUE, positive & negative directions are swapped compared to RED

                    // Change COLUMNS
                    // Checking if moving in positive or negative Column / X direction
                    if (changeCols > 0) {
                        for (int n = 0; n < changeCols; n++) {
                            driveStraight(DRIVE_SPEED, -24.0, 0.0);     // Drive Forward 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);     // Hold 0 Deg heading for 0.25 second
                        }
                    } else if (changeCols < 0) {
                        for (int X = 0; X < Math.abs(changeCols); X++) {
                            driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);     // Hold 0 Deg heading for 0.25 second
                        }
                    }

                    // Change ROWS
                    // Checking if moving in positive or negative Rows / Y direction
                    if (changeRows > 0) {
                        for (int n = 0; n < changeRows; n++) {
                            strafeRight(DRIVE_SPEED, 24.0);                    // Drive Left 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second
                        }
                    } else if (changeRows < 0) {
                        for (int X = 0; X < Math.abs(changeRows); X++) {
                            strafeLeft(DRIVE_SPEED, 24.0);                    // Drive Left 24"
                            holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second
                        }
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
            this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        robotHeading = getRawHeading() - headingOffset;

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
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
