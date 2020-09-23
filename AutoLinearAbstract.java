/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Abstract Linear Autonomous OpMode
 * Created 19Nov2017 for the Greenwood FTC Robotics Club.
 * This class provides relicRobot status and methods typical to all Relic Game autonomous OpModes
 *
 * Revised 18Dec2017 - Center Grove competition adjustments
 * Revised 23Jan2018 - Add JewelArm2 servo
 * Revised 13Feb2018 - Modified Glyph Arm gear ratio and speed
 * Revised 15Feb2018 - Increased servo speed increments
 *                   - Added servoRelicLinear and initialization logic*/

//@Disabled
public abstract class AutoLinearAbstract extends LinearOpMode {

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
    mecanumDrive
            driveTrain;

    //  DeviceTargetMotor
    ;

    //DeviceTargetServo


    DigitalChannel liftSafetySwitch;

    boolean safeStop;

    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer


    // CONSTANTS
    //static final int


    final static double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = .25,

    DRIVE_TRAIN_PIVOT_SPEED = 0.2,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.35,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.6,
            SCISSOR_LIFT_PICK_POS = 325,
            SCISSOR_LIFT_CARRY_HEIGHT = 500,
            SCISSOR_LIFT_POS = 900,
            SCISSOR_LIFT_POSITION_ERROR = 100,
            BOX_MOVER_PICK_POS_OUT = 7700,
            BOX_MOVER_POSITION_ERROR = 100,
            CLAW_SERVO_LEFT_UP = 0.55,
            CLAW_SERVO_LEFT_DOWN = 0.95,
            CLAW_SERVO_RIGHT_UP = 0.47,
            CLAW_SERVO_RIGHT_DOWN = 0.08;


    final static boolean
            FORWARD = false,
            REVERSE = true;



    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        // Noti.update();

        // get a reference to our digitalTouch object.
        liftSafetySwitch = hardwareMap.get(DigitalChannel.class, "lift_safety_switch");

        // set the digital channel to input.
        liftSafetySwitch.setMode(DigitalChannel.Mode.INPUT);
        safeStop = false;

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new mecanumDrive(hardwareMap, "front_left_drive", REVERSE, "front_right_drive", FORWARD, "rear_left_drive", REVERSE, "rear_right_drive", FORWARD, 960, 1.0, 2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */

        //=====================================
        // Example of Motor setup in AutoLinearAbstract
        //=======================================

        // scissorLift = new DeviceTargetMotor(hardwareMap,"scissor_lift",FORWARD, motorEncoderCounts Per Rev: 1680);

        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        // colorLeftJewel = new DeviceColorSensor(hardwareMap,"left_jewel_color",0x3c);
        //   colorRightJewel = new DeviceColorSensor(hardwareMap,"right_jewel_color",0x30);

        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */

        //=====================================
        //Example of Servo Setup in AutoLinearAbstract
        //=====================================
        // boxGrabber = new DeviceTargetServo(hardwareMap,"box_grabber",BOX_GRABBER_OPEN);

        /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/


        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();


        /* Lock drive train at current position */
        driveTrain.front.motorLeft.goToAbsoluteDistance(driveTrain.front.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.front.motorRight.goToAbsoluteDistance(driveTrain.front.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorLeft.goToAbsoluteDistance(driveTrain.rear.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorRight.goToAbsoluteDistance(driveTrain.rear.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);

        // Note: Servo initialization is completed in the respective object constructors


        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */


        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer


        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }


    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */
    void driveTrainTelemetry() {
        telemetry.addLine("Left Drive Front Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.leftSpeed);


        telemetry.addLine("Left Drive Rear Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.leftSpeed);

        telemetry.addLine("Right Drive Front Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.rightSpeed);

        telemetry.addLine("Right Drive Rear Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.rightSpeed);
    }



//Example of telemetry with servos
      //  telemetry.addData("Claw Servo Left",clawServoLeft.currentPosition);

   // }



    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
}

    void motorTelemetry (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in EU", "%.2f EU ", motor.getPosition());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }

    void DigitalSwitchStatus (DigitalChannel DSW) {
        telemetry.addLine();
        telemetry.addLine(DSW.getDeviceName());
        if (DSW.getState()) {
            telemetry.addLine("Switch is Open");
        }
        else
            telemetry.addLine("Switch is Closed");
    }

    boolean Kill ( double autoTime) {
        boolean eStop;

        if (liftSafetySwitch.getState() == false) {
            safeStop = true;
        }

        if (!opModeIsActive() || autoTimer.seconds() >= autoTime || safeStop) {

          driveTrain.stop();
        //    scissorLift.stop();
          //  boxMover.goToAbsoluteDistance(0,.3);
         //   scissorLift.goToAbsoluteDistance(0,.3);

            eStop = true;

        }
        else
            eStop = false;

     /*   while (!boxMover.isMoveDone(BOX_MOVER_POSITION_ERROR) || !scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Safe Stopping lift");
            telemetry.update();
        }

*/
        return eStop;


    }

  /*  void TelemetryRobot () {
        driveTrainTelemetry();
        motorTelemetry(scissorLift);
        motorTelemetry(boxMover);
        DigitalSwitchStatus(liftSafetySwitch);
    }



*/
        }
