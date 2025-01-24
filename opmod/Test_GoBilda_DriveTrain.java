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
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Pick;

    private DcMotor Drop;
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;

    private boolean ExtendDrop;

    private double DropInchToGo;
    private boolean isDriveTranDirectionSamples;
    private boolean IsLeftTrigger;
    private boolean IsRightTrigger;
    private boolean isHold;
    private Thread SampleTransferThread;
    private Thread SampleIntakingthread;

    private Thread DriveTrainThread;

    private Thread DropLinearSlideThread;

    private Thread DropSampleIntoHighBasket;

    private Thread ScoreSpecThread;

    private boolean ScoreSpec;

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
    static final double COUNTS_PER_MOTOR_REV_Gobilda_5203 = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double     COUNTS_PER_INCH_5203         = (COUNTS_PER_MOTOR_REV_Gobilda_5203 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private boolean IsDriveTrainThreadAlive;
    private boolean IsSampleTransferThreadAlive;
    private boolean IsSampleIntakingThreadAlive;
    private boolean IsDropLinearSlideThreadAlive;
    private boolean IsDropSampleIntoHighBasketThreadAlive;
    private boolean BClicked;
    private boolean IsScoreSpecThreadAlive;
    private double currentPos;


    @Override
    public void init() {
        IsLeftTrigger = false;
        IsRightTrigger = false;
        ExtendDrop = false;
        BClicked = false;
        ScoreSpec=false;
        DropInchToGo = 0;
        currentPos =0;
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
        isDriveTranDirectionSamples = true;
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
        // create/initialize all the required threads
        CreateSampleTransferThread();
        CreatSampleIntakeThread();
        CreateDriveTrainThread();
        CreateDropLinearSlideThread();
        CreateDropSampleIntoHighBasketThread();
        CreateScoreSpecThread();


    }

    private void CreateScoreSpecThread(){  ScoreSpecThread = new Thread(() -> {
        while (IsScoreSpecThreadAlive) {
            if (ScoreSpec) {
                ExtendDrop = false;
                isHold = false;
                DropExtendINCH(-8);
                Delay(0.1);
                SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
                Delay(0.3);
                DropExtendINCH(0);
                ScoreSpec = false;
            }
        }
    });}

    private void CreateDropSampleIntoHighBasketThread() {
        DropSampleIntoHighBasket = new Thread(() -> {
            while (IsDropSampleIntoHighBasketThreadAlive) {
                if (BClicked) {
                    dropTheSample();
                    BClicked = false;
                }
            }
        });
    }

    private void CreateDropLinearSlideThread() {
        DropLinearSlideThread = new Thread(() -> {
            while (IsDropLinearSlideThreadAlive) {
                if (ExtendDrop) {
                    DropExtendINCH(DropInchToGo);
                    while (ExtendDrop) {
                        Drop.setPower(0.001);
                    }
                }
            }
        });
    }

    private void CreateDriveTrainThread() {
        DriveTrainThread = new Thread(() -> {
            while (IsDriveTrainThreadAlive) {
                double Position = gamepad1.right_stick_y;
                double SidePosition = gamepad1.right_stick_x;
                double strafePosition = gamepad1.left_stick_x;
                MoveForward(Position);
                MoveSide(SidePosition);
                Strafe(strafePosition);
                telemetry.addData("FW Motor power",
                        Position);
                telemetry.addData("side Motor power",
                        SidePosition);
                telemetry.update();
            }
        });
    }


    private void CreateSampleTransferThread() {
        SampleTransferThread = new Thread(() -> {
            while (IsSampleTransferThreadAlive) {
                if (IsLeftTrigger) {
                    sampleDeposit();
                    IsLeftTrigger = false;
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });
    }

    private void CreatSampleIntakeThread() {
        SampleIntakingthread = new Thread(() -> {
            while (IsSampleIntakingThreadAlive) {
                if (IsRightTrigger) {
                    GoToTheSample();
                    IsRightTrigger = false;
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);

                }
            }
        });
    }

    @Override
    public void start() {
        IsSampleIntakingThreadAlive = true;
        IsDropLinearSlideThreadAlive = true;
        IsSampleTransferThreadAlive = true;
        IsDriveTrainThreadAlive = true;
        IsDropSampleIntoHighBasketThreadAlive = true;
        IsScoreSpecThreadAlive=true;
        ScoreSpecThread.start();
        SampleTransferThread.start();
        SampleIntakingthread.start();
        DriveTrainThread.start();
        DropLinearSlideThread.start();
        DropSampleIntoHighBasket.start();

    }

    @Override
    public void stop() {
        IsDriveTrainThreadAlive = false;
        IsSampleIntakingThreadAlive = false;
        IsDropLinearSlideThreadAlive = false;
        IsSampleTransferThreadAlive = false;
        IsDropSampleIntoHighBasketThreadAlive = false;
        IsScoreSpecThreadAlive=false;

    }

    @Override
    public void loop() {
        telemetry.addData("Linear slid Currently at", " at %7d",
                Pick.getCurrentPosition());
        telemetry.addData("Dropper slid Currently at", " at %7d",
                Drop.getCurrentPosition());



      /*  if(gamepad1.a){
            if(isDriveTranDirectionSamples){
                //DriveTrain directions for sample
                Fright.setDirection(DcMotorSimple.Direction.FORWARD);
                Bright.setDirection(DcMotorSimple.Direction.FORWARD);
                BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                isDriveTranDirectionSamples =false;
            }


             //direction specimen
            else{
                //DriveTrain
                Fright.setDirection(DcMotorSimple.Direction.REVERSE);
                Bright.setDirection(DcMotorSimple.Direction.REVERSE);
                BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                FLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                isDriveTranDirectionSamples = e;
            }

       */
        if (gamepad1.right_trigger > 0) {
          if (IsLeftTrigger==false) {
              IsRightTrigger = true;
          }
        }
        // pick the sample
        if (gamepad1.y) {
            GrabTheSample();
            //ClawClose();
        }
        // bring the sample to outtake claw
        if (gamepad1.left_trigger > 0) {
            //sampleDeposit();
            if (IsRightTrigger==false){
            IsLeftTrigger = true;}
        }
        // go and get ready to pick the sample
        if (gamepad1.a) {
            //ClawOpen();
            prepareHandToGrab();
        }
        //drop the sample in the bucket
        if (gamepad1.b) {
            //ClawOpen();
            ExtendDrop = false;
            BClicked = true;
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
            DropInchToGo = 19.5;
            ExtendDrop = true;
        }
        if (gamepad1.dpad_down) {
           ScoreSpec=true;
            //TO get it back to the zero position
        }

        double temp = gamepad1.left_stick_y;
        if (temp != 0) {
            ExtendDrop = false;
            telemetry.addData("Dropper slid Currently at manually", " at %7d",
                    Drop.getCurrentPosition());
        }

        Drop.setPower(-gamepad1.left_stick_y); // reverse
        Drop.setPower(-gamepad2.left_stick_y);

       if (gamepad2.dpad_down){outake.setPosition(0.9);}
        if (gamepad2.dpad_up){outake.setPosition(0.2);}
        if(gamepad2.dpad_left){arm.setPosition(0.9);}
        if(gamepad2.right_bumper){
            currentPos+=0.0015;
            if(currentPos > 1){
                currentPos=1;
            }
            arm.setPosition(currentPos);
        }
        if(gamepad2.left_bumper){
            currentPos-=0.0015;
            if(currentPos <0){
                currentPos=0;
            }
            arm.setPosition(currentPos);
        }
        telemetry.addData("Arm Servo current position", " at %7f",
                currentPos);


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
        Pick.setPower(gamepad2.right_trigger);

        telemetry.update();
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
        if (isDriveTranDirectionSamples) {
            //Drivetain deraction for sample
            FLeft.setPower(-ScalePower(position));
            BLeft.setPower(-ScalePower(position));
            Fright.setPower(ScalePower(position));
            Bright.setPower(ScalePower(position));
        } else {
            //Drivetrain direction for specimen
            FLeft.setPower(ScalePower(position));
            BLeft.setPower(ScalePower(position));
            Fright.setPower(-ScalePower(position));
            Bright.setPower(-ScalePower(position));
        }
    }

    private void Strafe(double slide) {
        FLeft.setPower(-ScalePower(slide));
        Fright.setPower(ScalePower(slide));
        BLeft.setPower(ScalePower(slide));
        Bright.setPower(-ScalePower(slide));
    }

    private void SpecScore() {

    }

    private void handDeposit() {
        handServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
    }

    private void handGrab() {
        handServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
    }

    private void PickExtendINCH(double Inch) {
        runtime.reset();
        double pwr = 0;
        int timeout = 2;
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
        while (Pick.isBusy() && runtime.seconds() < timeout) {
            Pick.setPower(pwr);
        }
        Pick.setPower(0);
    }

    private void DropExtendINCH(double Inch) {
        runtime.reset();
        int timeout = 3;
        double pwr = 0;
        double TargetPosition = Inch * COUNTS_PER_INCH_5203;
        int currentPosition = Drop.getCurrentPosition();
        int NewPosition = currentPosition + (int) TargetPosition;
        if (Inch > 0) {
            pwr = 1;
            timeout = 5;
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
        while (Drop.isBusy() && runtime.seconds() < timeout) {
            Drop.setPower(pwr);
        }

        if (Inch > 0) {
            Drop.setPower(0.001 * pwr);
        } else {
            Drop.setPower(0);
        }

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
        handDeposit();
        armIntake();
        PickExtendINCH(13);
        handGrab();
        ClawOpen();
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

    private void sampleDeposit() {
        armIntake();
        setRotateClawAtReset();
        PickExtendINCH(-13);
        outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
        Delay(0.1);
        ClawOpen();
        Delay(0.1);
        armServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
        ScoreSample();

    }

    private void ScoreSample() {
        ExtendDrop = false;
        DropExtendINCH(27);
        isHold = true;
        while (isHold) {
            Drop.setPower(0.001);
        }

    }

    private void dropTheSample() {
        outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
        Drop.setPower(0.005);
        Delay(1);
        armIntake();
        Drop.setPower(0.005);
        Delay(1.5);
        isHold = false;
        DropExtendINCH(0); // reset to 0 position
    }

    private void Delay(double Seconds) {
        runtime.reset();
        while (runtime.seconds() < Seconds) ;
    }

    //\\\//\*:)(:
    private void GrabSpec() {
        SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_CLOSE);
    }

    private void DropSpec() {
        SpecClawServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
    }

    private void armIntake() {
        armServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
        outtakeServo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_REST);
    }

    private double ScalePower(double input) {
        return Math.max(-1, Math.min(1, input));
    }
}
