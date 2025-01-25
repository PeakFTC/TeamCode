package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Test_GoBilda_DriveTrain extends OpMode {
    private boolean isLoopforBreak;
    private boolean isHandkeIntake;
    private int prepareSmaple =0;
    private boolean isHandleDiposit;
    private int dropSampeState=0;
    private boolean isHandleSpecPrep;
    private int specDropState = 0;

    public enum BucketLevel{
        BUCKET_LEVEL_1,
        BUCKET_LEVEL_2,
        NONE
    }
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Pick;
    private DcMotor Drop;
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;
    private Servo hand;
    private Servo claw;
    private Servo RoClaw;
    private Servo SpecClaw;
    private Servo arm;
    private peakFTCServo handServo;
    private peakFTCServo clawServo;
    private peakFTCServo SpecClawServo;
    private peakFTCServo armServo;
    private peakFTCServo outtakeServo;

    //New classes for color sensor

    private AutoColorSensor ColorSensor;
    private Servo outake;
    private double roClowPos;
    private double currentClwRotatePos;
    static final double COUNTS_PER_MOTOR_REV_Gobilda_5203 = 384.5;    // eg: gobilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double     COUNTS_PER_INCH_5203         = (COUNTS_PER_MOTOR_REV_Gobilda_5203 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void init() {
        FLeft = hardwareMap.dcMotor.get("FLeft");
        BLeft = hardwareMap.dcMotor.get("BLeft");
        Fright = hardwareMap.dcMotor.get("FRight");
        Bright = hardwareMap.dcMotor.get("BRight");
        hand = hardwareMap.get(Servo.class, "hand");
        claw = hardwareMap.get(Servo.class, "claw");
        RoClaw = hardwareMap.get(Servo.class, "RoClaw");
        SpecClaw = hardwareMap.get(Servo.class, "SpecClaw");
        arm = hardwareMap.get(Servo.class, "arm");
        outake = hardwareMap.get(Servo.class, "outake");
        Pick = hardwareMap.dcMotor.get("pick");
        Drop = hardwareMap.dcMotor.get("drop");
        //DriveTrain
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //Intake
        Pick.setDirection(DcMotorSimple.Direction.FORWARD);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo = new peakFTCServo(claw, Servo.Direction.REVERSE, 0.34, 0.8, 0.34);
        handServo = new peakFTCServo(hand, Servo.Direction.FORWARD, 0, 0.68, 0);
        SpecClawServo = new peakFTCServo(SpecClaw, Servo.Direction.FORWARD, 0, 1, 0);
        outtakeServo = new peakFTCServo(outake, Servo.Direction.REVERSE, 0.65, 0.2, 0.5);
        armServo = new peakFTCServo(arm, Servo.Direction.FORWARD, 0.8, 0.2, 0.8);

        //Color sensor
        ColorSensor=new AutoColorSensor(hardwareMap, telemetry);

        //Outake
        Drop.setDirection(DcMotorSimple.Direction.FORWARD);
        Drop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo Powers
        RoClaw.setPosition(0.45);
        roClowPos = 0.45;
        isLoopforBreak = false;
        isHandkeIntake = false;
        isHandleDiposit =false;
        isHandleSpecPrep = false;
    }
    private void moveDriveTrain() {
        double Position = gamepad1.right_stick_y;
        double SidePosition = gamepad1.right_stick_x;
        double strafePosition = gamepad1.left_stick_x;
        MoveForward(Position);
        MoveSide(SidePosition);
        Strafe(strafePosition);
    }
    private void sampleIntakeCommand() {
        GoToTheSample();
    }

    @Override
    public void loop() {
        telemetry.addData("Linear slid Currently at", " at %7d",
                Pick.getCurrentPosition());
        telemetry.addData("Dropper slid Currently at", " at %7d",
                Drop.getCurrentPosition());

/*
Gamepad-1 uses
    * Right_Trigger -- prepare for sample pickup
    * Left_Trigger -- pick the sample and prepare for drop in to the bucket
    * .y -- grab the sample
    * .x -- available
    * .a -- prepare intake claw to grab the sample
    * .b -- drop the sample into the bucket
    * Bumper_Left & Bumper_Right rotate the intake claw
    * DPad_Left -- Grab Specimen
    * DPad_Right -- drop Specimen
    * DPad_UP -- move up drop linear slide to hang the specimen
    * DPad_Down-- move down drop linear slide after hanging the specimen
    * Right_Stick_x/y -- operate  drivetrain left/Right forward/backward
    * Left_Stick_x -- Strafe drive train
    * Left_Stick_y -- operate Drop linear slide in emergency case
*/
        if (gamepad1.right_trigger > 0) {
            if(!isHandkeIntake) {
                isHandkeIntake = true;
                prepareSmaple = 0;
            }
        }
        if(isHandkeIntake) {
            sampleIntakeCommand();
        }
        // bring the sample to outtake claw
        if (gamepad1.left_trigger > 0) {
            if(!isHandleDiposit){
            isHandleDiposit = true;
            dropSampeState=0;
            }
        }
        if(isHandleDiposit){
            sampleDepositCommand(BucketLevel.BUCKET_LEVEL_2);
        }
        if (gamepad1.x) {
            // can be use for level-1 sample drop

        }
        // pick the sample
        if (gamepad1.y) {
            GrabTheSample();
        }
        // go and get ready to pick the sample
        if (gamepad1.a) {
            //ClawOpen();
            prepareHandToGrab();
        }
        //drop the sample in the bucket
        if (gamepad1.b) {
            // this button is used in the while loop so no need to handle here!
           // isLoopforBreak = false;
           // dropTheSample();
        }
        // claw and linear slide go to the sample transfer position
        if (gamepad1.left_bumper) {
            roClowPos += 0.015;
            RoClaw.setPosition(roClowPos);
        }
        // claw and linear slide go to the sample pickup position
        if (gamepad1.right_bumper) {
            roClowPos -= 0.015;
            RoClaw.setPosition(roClowPos);
        }
        if (gamepad1.dpad_left) {
            GrabSpec();
        }
        if (gamepad1.dpad_right) {
            DropSpec();
        }
        if (gamepad1.dpad_up) {
            if(!isHandleSpecPrep) {
                isHandleSpecPrep = true;
                specDropState = 0;
            }
        }
        if(isHandleSpecPrep){
            specHangingCommand();
        }
        if (gamepad1.dpad_down) {
            //used in the gamepad1.dpad_up to break the while look drop linear slide break;
            // code moved under state machine
        }
        moveDriveTrain();
      //  Drop.setPower(-gamepad1.left_stick_y); // reverse


// Gamepad-2 function reserve for Testing purpose or emergency control

       // Drop.setPower(-gamepad2.left_stick_y);

       if (gamepad2.dpad_down){outake.setPosition(0.9);}
        if (gamepad2.dpad_up){outake.setPosition(0.2);}
        if(gamepad2.dpad_left){arm.setPosition(0.9);}
        if(gamepad2.right_bumper){
            currentClwRotatePos+=0.0015;
            if(currentClwRotatePos > 1){
                currentClwRotatePos=1;
            }
            arm.setPosition(currentClwRotatePos);
        }
        if(gamepad2.left_bumper){
            currentClwRotatePos-=0.0015;
            if(currentClwRotatePos <0){
                currentClwRotatePos=0;
            }
            arm.setPosition(currentClwRotatePos);
        }
//        telemetry.addData("Arm Servo current position", " at %7f",currentPos);
        // gamepad 2-for manual control in case asign task failed to acheive the result
        if (gamepad2.x) {
            DropExtendINCH(20);
            Drop.setPower(0.005);
            Delay(10);
            Drop.setPower(0);
        }
        if (gamepad2.y) {
            DropExtendINCH(0);
        }
       // Pick.setPower(gamepad2.right_trigger);

        telemetry.addData("goto sample state","%d",prepareSmaple);
        telemetry.addData("drop sample state","%d",dropSampeState);
        telemetry.addData("drop Spec state","%d",specDropState);
        telemetry.update();
    }

    private void specHangingCommand() {

        switch (specDropState){
            case 0:
                DropExtendINCH(22);
                specDropState=1;
                break;
            case 1:
                if(!Drop.isBusy()){
                    specDropState=2;
                    Drop.setPower(0.005);
                }
                break;
            case 2:
                if(gamepad1.dpad_down){
                    specDropState=3;
                    DropExtendINCH(-8);
                }
                break;
            case 3:
                if(!Drop.isBusy()){
                    Delay(0.1);
                    SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
                    Delay(0.3);
                    Drop.setPower(0.005);
                    DropExtendINCH(0);
                    specDropState=4;
                }
                break;
            case 4:
                if(!Drop.isBusy()){
                    isHandleSpecPrep=false;
                    Drop.setPower(0);
                }
                break;

            default:
                break;
        }
    }

    private void prepareHandToGrab() {
        ClawOpen();
        handGrab();
    }

    //DRIVETRAIN MOVEMENT CODE/ FUNCTIONS
    private void MoveForward(double Position) {
        FLeft.setPower(ScalePower(Position));
        Fright.setPower(ScalePower(Position));
        BLeft.setPower(ScalePower(Position));
        Bright.setPower(ScalePower(Position));
    }
    private void MoveSide(double position) {
        FLeft.setPower(-ScalePower(position));
        BLeft.setPower(-ScalePower(position));
        Fright.setPower(ScalePower(position));
        Bright.setPower(ScalePower(position));
    }
    private void Strafe(double slide) {
        FLeft.setPower(-ScalePower(slide));
        Fright.setPower(ScalePower(slide));
        BLeft.setPower(ScalePower(slide));
        Bright.setPower(-ScalePower(slide));
    }
    private void handDeposit() {
        handServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
    }
    private void handGrab() {
        handServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
    }

    private void PickExtendINCH(double Inch) {

        double pwr = 0;
        double TargetPosition = Inch * COUNTS_PER_INCH_5203;
        int currentPosition = Pick.getCurrentPosition();
        int NewPosition = currentPosition + (int) TargetPosition;
        if (Inch > 0) {
            pwr = 1;
            Pick.setTargetPosition(NewPosition);// move up/forward to given inches.
        } else {
            pwr = -1;
            Pick.setTargetPosition(0); // go back to the starting position
        }
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pick.setPower(pwr);
    }

    private void DropExtendINCH(double Inch) {
        double pwr = 0;
        double TargetPosition = Inch * COUNTS_PER_INCH_5203;
        int currentPosition = Drop.getCurrentPosition();
        int NewPosition = currentPosition + (int) TargetPosition;
        if (Inch > 0) {
            pwr = 1;
            Drop.setTargetPosition(NewPosition);// move up/forward to given inches.
        } else if (Inch < 0) {
            pwr = -1;
            Drop.setTargetPosition(NewPosition); // go back to the starting position
        } else {
            pwr = -1;
            Drop.setTargetPosition(0); // go back to the starting position
        }
        Drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drop.setPower(pwr);
    }

    private void ClawOpen() {
        clawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
    }

    private void ClawClose() {
        clawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
    }

    //0. ensure that the hand is at the resting position
    //1. Extend linear slide
    //2. Set hand set position to grab
    //3. open claw ready to grab specimen
    //ensure that the RoClaw is in the correct position
    private void GoToTheSample() {
        switch( prepareSmaple) {
            case 0:
                handDeposit();
                armIntake();
                prepareSmaple = 1;
                runtime.reset();
                PickExtendINCH(13);
                break;
            case 1:
                if (!Pick.isBusy()) {
                    prepareSmaple = 2;
                    Pick.setPower(0);
                }
                break;
            case 2:
                handGrab();
                ClawOpen();
                prepareSmaple = 3;
                isHandkeIntake =false;
                break;
            default:
        }
    }

    private void GrabTheSample() {
        AutoColorSensor.DetectedColor ColorDeteceted;
        ClawClose();
        Delay(0.3);
        ColorDeteceted =ColorSensor.getDetetedColor();
        if(ColorDeteceted == AutoColorSensor.DetectedColor.RED || ColorDeteceted == AutoColorSensor.DetectedColor.YELLOW)
        {
            handDeposit();
        }
        else{
            ClawOpen();
        }

    }

    private void setRotateClawAtReset() {
        RoClaw.setPosition(0.45);
    }

    /*
        * Command to perform the following actions
        * pick the sample in the intake clow and move to delivery position
        * Rotate the claw at delevery position to aline the sample with outtake claw
        * move back the pick linear slide to sample delivery position
        * sample hold by the outtake claw
        * released the sample from intake clow
        * call Score the sample ( ScoreSampleAtLevel2/ScoreSampleAtLevel1) depend on the bucket choice
     */
    private void sampleDepositCommand( BucketLevel level) {
        switch (dropSampeState){
            case 0:
                armIntake();
                setRotateClawAtReset();
                PickExtendINCH(-13);
                dropSampeState =1;
                break;
            case 1:
                if(!Pick.isBusy()){
                    dropSampeState=2;
                    Pick.setPower(0);
                }
                break;
            case 2:
                outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
                Delay(0.1);
                ClawOpen();
                Delay(0.1);
                armServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);

                if(level == BucketLevel.BUCKET_LEVEL_2)
                    DropExtendINCH(27);
                else if (level==BucketLevel.BUCKET_LEVEL_1) {
                    DropExtendINCH(13);
                }
                dropSampeState=3;
                runtime.reset();
                break;
            case 3:
                //if(!Drop.isBusy() || runtime.seconds()>5){
                if(!Drop.isBusy()){
                    dropSampeState=4;
                    Drop.setPower(0.005); // set break
                }
                else{
                    Drop.setPower(1);
                }
                break;
            case 4:
                if(gamepad1.b) {
                    dropSampeState = 5;
                    isHandleDiposit = false;
                    outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
                    armIntake();
                    DropExtendINCH(0); // reset to 0 position
                    runtime.reset();
                }
                break;
            case 5:
                //if(!Drop.isBusy()|| runtime.seconds()>3)
                if(!Drop.isBusy()){
                    Drop.setPower(0);
                    isHandleDiposit=false;
                }
                else{
                    Drop.setPower(-1);
                }
                break;

            default:
        }

    }
    /*
     * Function to perform the following actions
     * extend the drop linear slide to lavel2 bucket
     * hole the drop linear slide position until drop command release from user
     * in the wait loop wait for the drop command from the user while allow user to drive the drive train
     * and keep checking the drop command
     * */
    private void ScoreSampleAtLevel2() {


        isLoopforBreak = true;

    }
    /*
     * Function to perform the following actions
     * extend the drop linear slide to lavel1 bucket
     * hole the drop linear slide position until drop command release from user
     * in the wait loop wait for the drop command from the user while allow user to drive the drive train
     * and keep checking the drop command
     * */
    private void ScoreSampleAtLevel1() {

        isLoopforBreak = true;
    }
    /*
     * Function to perform the following actions
     * drop the sample from outtake claw to bucket
     * give break for small time period to stable drop
     * change the outtake claw (arm) from drop-in to pick-up
     * get the drop linear slide at start position
     * */
    private void dropTheSample() {
        outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
        Drop.setPower(0.005);
        Delay(1);
        armIntake();
        DropExtendINCH(0); // reset to 0 position
    }
    /*
     * Function to perform the following actions
     * provide a delay in seconds
     * while in delay allow user to move the drive train
     * */
    private void Delay(double Seconds) {
        runtime.reset();
        while (runtime.seconds() < Seconds) {
            moveDriveTrain();
        }
    }

    /*
     * Function to perform the following actions
     * Grab specimen
     * */
    private void GrabSpec() {
        SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
    }
    /*
     * Function to perform the following actions
     * Hang the specimen on submersible wall
     * */
    private void DropSpec() {
        SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
    }
    /*
     * Function to perform the following actions
     * Keep the outtake clow and it arm at rest/pickup position
     * */
    private void armIntake() {
        armServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
        outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_REST);
    }
    /*
     * Function to perform the following actions
     * scale the input power between -1 to 1 of given any real value
     * */
    private double ScalePower(double input) {
        return Math.max(-1, Math.min(1, input));
    }
}
