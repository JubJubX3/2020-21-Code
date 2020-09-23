package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;



@TeleOp(name="GM TeleOpMecanum", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.
//@Disabled


public class GMTeleOpMecanum extends OpMode{


    double  //declares all double variables and their values
            //trackServoPosition = robot.TRACK_SERVO_EXTEND_SPEED,
            pullupspeed = 0;


    private boolean //declares all private booleans and their initial values (true or false)
            scissorLiftMove = false,
            bPressed = false,
            xPressed = false,
            moveBoxMover = false;

    final int
            LIFT_MAX = 6,
            LIFT_MIN = 0,
            BOX_MOVER_OUT = 10500,
            BOX_MOVER_IN = 0,
            SCISSOR_DOWN_POS = 0,
            SCISSOR_SAFE_POS = 2500,
            SCISSOR_UP_POS = 12500;



    final double
            BOX_GRABBER_CLOSED = 0.52,
            BOX_GRABBER_OPEN = 0.7,
    // JERRY_TURNER_REGULAR = 0.3,
    // JERRY_TURNER_TWIST = 0.8,
    // CAPSTONE_CLOSE = 0.55,
    SCISSOR_MOTOR_SPEED_FACTOR = 1,
            BOX_MOVER_SAFE_SPEED_FACTOR = .25,
            SCISSOR_SAFE_SPEED_FACTOR = .25,
            SCISSOR_MOTOR_INC = 1800,
            SCISSOR_CURRENT_POS = 0,
            BOX_MOTOR_SPEED_FACTOR = 0.9,
            NULL_MOTOR_MOVE_SPEED = 0.1,
            CLAW_SERVO_LEFT_UP = 0.55,
            CLAW_SERVO_LEFT_DOWN = 0.95,
            CLAW_SERVO_RIGHT_UP = 0.47,
            CLAW_SERVO_RIGHT_DOWN = 0.08;


    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */

    DcMotor
    frontLeftDrive = null,
    frontRightDrive = null,
    rearLeftDrive = null,
    rearRightDrive = null;

    HardwareMap hwMap =  null;



   // @Override

    public void init() { //initialization class to be used at start of tele-op



        // these are our motors and what they are called
        rearLeftDrive = hwMap.dcMotor.get("rear_left_drive");
        rearRightDrive = hwMap.dcMotor.get("rear_right_drive");
        frontLeftDrive = hwMap.dcMotor.get("front_left_drive");
        frontRightDrive = hwMap.dcMotor.get("front_right_drive");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);



        frontRightDrive.setTargetPosition(0);
        frontLeftDrive.setTargetPosition(0);
        rearRightDrive.setTargetPosition(0);
        rearLeftDrive.setTargetPosition(0);

        //Resetting encoder numbers to zero
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Setting motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // other motors use encoders


        //this will send a telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        bPressed = false;
        xPressed = false;
    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    @Override

    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */

    @Override

    public void start() {
    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    @Override

    public void loop() {

//==========================================================
//						GamePad One
//==========================================================

        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;


        FLspeed = Range.clip(FLspeed, -1, 1);
        BLspeed = Range.clip(BLspeed, -1, 1);
        FRspeed = Range.clip(FRspeed, -1, 1);
        BRspeed = Range.clip(BRspeed, -1, 1);



        rearLeftDrive.setPower(BLspeed);
        rearRightDrive.setPower(BRspeed);
        frontLeftDrive.setPower(FLspeed);
        frontRightDrive.setPower(FRspeed);
        //=========================================================//
        //						GamePad Two                        //
        //=========================================================//
//nothing
        //==========================================================//
        //						Telemetry                           //
        //==========================================================//



        telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);

        telemetry.addData("Pull Up Speed", "%.2f", pullupspeed);

        telemetry.addData("G2 Left Bumper", gamepad2.left_bumper);

        telemetry.addData("G2 Right Bumper", gamepad2.right_bumper);

        telemetry.addData("BRMotor", rearRightDrive.getCurrentPosition());
        telemetry.addData("FRMotor", frontRightDrive.getCurrentPosition());

        telemetry.addData("BLMotor", rearLeftDrive.getCurrentPosition());
        telemetry.addData("FLMotor", frontLeftDrive.getCurrentPosition());


        telemetry.addData("Dpad Down",gamepad2.dpad_down);
        telemetry.addData("Dpad Up",gamepad2.dpad_up);

//Controls the pull-up system of robot

//An Example how to move a servo
       // if (gamepad1.y) {
        //    robot.clawServoLeft.setPosition(robot.CLAW_SERVO_LEFT_UP);
       //     robot.clawServoRight.setPosition(robot.CLAW_SERVO_RIGHT_UP);
       // }



        }









        //==========================================================
        //						Telemetry
        //==========================================================


    /*
     * Code will run ONCE after the driver hits STOP
     */

    @Override

    public void stop(){


        // Sets all motors to zero power

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);

        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        //Ask and see if needed
        //robot.boxMover.setPower(0);
        //robot.scissorLift.setPower(0);


    }

} //end main