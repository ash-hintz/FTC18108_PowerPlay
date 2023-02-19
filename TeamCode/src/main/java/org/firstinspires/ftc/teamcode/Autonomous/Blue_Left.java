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

import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;

/**
 Autonomous code for competition - PowerPlay season
 */

@Autonomous(name="Blue_Left", group="Robot")
// @Disabled
public class Blue_Left extends LinearOpMode {

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

        // Set initial Bot Coordinates on the field
        ppb.currentBotCol = ppb.BLUE_LEFT_START_COL;
        ppb.currentBotRow = ppb.BLUE_LEFT_START_ROW;
        ppb.currentAlliance = PowerPlayBot.Alliance.BLUE;

        // Wait for the game to start (driver presses PLAY) Display IMU value while waiting
        while (opModeInInit()) {
            telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
            telemetry.addData("Start Position",  "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
            telemetry.update();
        }

        // PLAY just pressed on Driver Station, reset Heading and game timer
        ppb.runtime.reset();

        /*
         Autonomous code goes here
        */

        // Move from starting position against the wall to the center of the current tile
        ppb.driveStraight(DRIVE_SPEED, 4.0, 0.0);    // Drive Forward 4"
        ppb.holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second

        telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
        telemetry.addData("Current Position", "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
        telemetry.update();

        // Vision Detect for the Sleeve goes here

        // Drive to the closest stack of cones for our Alliance color
        // Drive to all the High Junctions + closest Medium Junction to deposit cones
        ppb.driveToCoordinate(ppb.BLUE_FRONT_CONE_STACK_COL, ppb.BLUE_FRONT_CONE_STACK_ROW);
        sleep(2000);
        ppb.driveToCoordinate(6, 6);
        ppb.driveToCoordinate(5, 6);
        sleep(2000);
        ppb.driveToCoordinate(6, 6);
        ppb.driveToCoordinate(ppb.BLUE_FRONT_CONE_STACK_COL, ppb.BLUE_FRONT_CONE_STACK_ROW);
        sleep(2000);
        ppb.driveToCoordinate(4, 4);
        ppb.driveToCoordinate(3, 4);
        sleep(2000);
        ppb.driveToCoordinate(4, 4);
        ppb.driveToCoordinate(ppb.BLUE_FRONT_CONE_STACK_COL, ppb.BLUE_FRONT_CONE_STACK_ROW);
        sleep(2000);
        ppb.driveToCoordinate(6, 4);
        ppb.driveToCoordinate(7, 4);
        sleep(2000);
        ppb.driveToCoordinate(6, 4);
        ppb.driveToCoordinate(ppb.BLUE_FRONT_CONE_STACK_COL, ppb.BLUE_FRONT_CONE_STACK_ROW);
        sleep(2000);
        ppb.driveToCoordinate(6, 2);
        ppb.driveToCoordinate(5, 2);
        sleep(2000);
        ppb.driveToCoordinate(6, 2);
        ppb.driveToCoordinate(ppb.BLUE_FRONT_CONE_STACK_COL, ppb.BLUE_FRONT_CONE_STACK_ROW);
        sleep(2000);
        ppb.driveToCoordinate(6, 2);
        ppb.driveToCoordinate(7, 2);
        sleep(2000);
        ppb.driveToCoordinate(8, 2);

        telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
        telemetry.addData("Current Position", "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
        telemetry.addData("Blue Left Path", "Complete");
        telemetry.update();
        sleep(10000);  // Pause to display last telemetry message.
    }
}
