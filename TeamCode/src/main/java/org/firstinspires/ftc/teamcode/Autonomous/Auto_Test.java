/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.TURN_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;

/**
 Autonomous code for competition - PowerPlay season
 */

@Autonomous(name="Auto_Test", group="Robot")
// @Disabled
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Make accessible the methods defined in PowerPlayBot
        // Pass the objects that are only defined once the OpMode starts running
        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        // Initialize the drive system variables
        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }

        ppb.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ppb.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ppb.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ppb.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial Bot Coordinates on the field
        ppb.currentBotCol = ppb.RED_RIGHT_START_COL;
        ppb.currentBotRow = ppb.RED_RIGHT_START_ROW;
        ppb.currentHeading = 0.0;
        ppb.currentAlliance = PowerPlayBot.Alliance.RED;

        // Wait for the game to start (driver presses PLAY) Display IMU value while waiting
        while (opModeInInit()) {
            telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
            //telemetry.addData("Robot Heading ", "= %4.0f", ppb.getRawHeading());
            telemetry.addData("Start Position",  "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
            telemetry.update();
        }

        // PLAY just pressed on Driver Station, reset Heading and game timer
        ppb.runtime.reset();

        /*
         Autonomous code goes here
        */


        // ppb.driveStraightGyro(24,0.1);

        ppb.frontLeft.setPower(0.1);
        sleep(2000);
        ppb.frontLeft.setPower(-0.1);
        sleep(2000);
        ppb.frontLeft.setPower(0);

        ppb.frontRight.setPower(0.1);
        sleep(2000);
        ppb.frontRight.setPower(-0.1);
        sleep(2000);
        ppb.frontRight.setPower(0);

        ppb.backLeft.setPower(0.1);
        sleep(2000);
        ppb.backLeft.setPower(-0.1);
        sleep(2000);
        ppb.backLeft.setPower(0);

        ppb.backRight.setPower(0.1);
        sleep(2000);
        ppb.backRight.setPower(-0.1);
        sleep(2000);
        ppb.backRight.setPower(0);

        /*
        // Move from starting position against the wall to the center of the current tile
        ppb.driveStraight(DRIVE_SPEED, 4.0, 0.0);    // Drive Forward 4"
        ppb.holdHeading(TURN_SPEED, 0.0, 0.5);      // Hold 0 Deg heading for a 0.25 second

        ppb.strafeRight(DRIVE_SPEED,2*12.0);
        ppb.holdHeading(TURN_SPEED, 0.0, 0.5);      // Hold 0 Deg heading for a 0.25 second

        ppb.strafeLeft(DRIVE_SPEED,2*12.0);
        ppb.holdHeading(TURN_SPEED, 0.0, 0.5);      // Hold 0 Deg heading for a 0.25 second

        ppb.driveStraight(DRIVE_SPEED, -4.0, 0.0);    // Drive Forward 4"
        ppb.holdHeading(TURN_SPEED, 0.0, 0.5);      // Hold 0 Deg heading for a 0.25 second
         */

        //ppb.driveStraightGyro(24,0.1);

        // Vision Detect for the Sleeve goes here

        telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
        telemetry.addData("Current Position", "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
        telemetry.addData("Red Right Path", "Complete");
        telemetry.update();
        sleep(10000);  // Pause to display last telemetry message.

    }
}
