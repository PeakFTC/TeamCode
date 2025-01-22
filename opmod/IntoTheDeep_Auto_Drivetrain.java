package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntoTheDeep_Auto_Drivetrain {
    HardwareMap hardwareMap;
    Telemetry telemetry;
//
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;

    private DcMotor Pick;

    private DcMotor Drop;


// Servos
    private Servo hand;
    private  Servo claw;
    private Servo RoClaw;
    private Servo SpecClaw;

    private Servo arm;

    private Servo outake;

    private ElapsedTime runtime;

    private Thread DropThread;

    private Thread PickThread;

    public   peakFTCServo SpecClawSrvo;
    public peakFTCServo ClawServo;
    public peakFTCServo OutakeServo;
    public peakFTCServo HandServo;

    public peakFTCServo armServo;



    //Calculation for motor to calculate inches
    static final double     COUNTS_PER_MOTOR_REV_5202    = 751.8 ;    // eg: go bilda 5202 Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV_5203    = 537.7 ;    // eg: go bilda 5203 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES_GB_MW   = 4.03 ;     // For figuring circumference
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_5203         = (COUNTS_PER_MOTOR_REV_5203 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_GB_MW * 3.1415);
    static final double     COUNTS_PER_INCH_5202         = (COUNTS_PER_MOTOR_REV_5202 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private boolean IsBreak;

    private double DropInch;
    private double PickInch;

    private boolean DropExtendStart;
    private boolean PickExtendStart;


    public IntoTheDeep_Auto_Drivetrain(HardwareMap hMap, Telemetry tel) {

        hardwareMap = hMap;
        telemetry = tel;

    }
    public void configDriveTrain()
    {
        // group DriveTranin
        FLeft = hardwareMap.dcMotor.get("FLeft");
        BLeft = hardwareMap.dcMotor.get("BLeft");
        Fright = hardwareMap.dcMotor.get("FRight");
        Bright = hardwareMap.dcMotor.get("BRight");
        hand=hardwareMap.get(Servo.class,"hand");
        claw=hardwareMap.get(Servo.class,"claw");
        RoClaw=hardwareMap.get(Servo.class,"RoClaw");
        SpecClaw=hardwareMap.get(Servo.class,"SpecClaw");
        arm=hardwareMap.get(Servo.class,"arm");
        outake=hardwareMap.get(Servo.class,"outake");
        Pick = hardwareMap.dcMotor.get("pick");
        Drop = hardwareMap.dcMotor.get("drop");

//DriveTrain Motor directions
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake and outake motor
        Pick.setDirection(DcMotorSimple.Direction.FORWARD);
        Drop.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo positions
        RoClaw.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.FORWARD);
        SpecClaw.setDirection(Servo.Direction.FORWARD);
        outake.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.FORWARD);

        //Encoder STOPING and RESETING
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Encoders for Drop and Pick slides
        Drop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//Running DriveTrain Encoders
        FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Running Drop and pick encoders
        Pick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo creation group
        SpecClawSrvo= new peakFTCServo(SpecClaw,Servo.Direction.FORWARD, 0, 1,0);
        ClawServo= new peakFTCServo(claw, Servo.Direction.REVERSE, 0.4,0.8,0.4);
        OutakeServo= new peakFTCServo(outake, Servo.Direction.REVERSE,0.75,0.2,0.5);
        HandServo= new peakFTCServo(hand, Servo.Direction.FORWARD,0,1,0);
        armServo=new peakFTCServo(arm, Servo.Direction.FORWARD,0.2,0.85,0.2);

        runtime = new ElapsedTime();

        //Thread for dropper slide

        CreateDropThread();
        startDropThread();
        CreatePickThread();
        startPickThread();
        IsBreak=false;
        PickExtendStart=false;
        DropExtendStart=false;
    }

    private void CreateDropThread(){
        DropThread = new Thread(() -> {
            //minumum things that we need
            DropExtendHold();
        });
    }
    private void CreatePickThread(){
        PickThread = new Thread(() -> {
            //minumum things that we need
            PickExtendHold();
        });
    }
    private void startDropThread(){
        DropThread.start();
    }
    private void startPickThread(){
        PickThread.start();
    }
  private void setSlidesEncoder(double inch){
        double tar = inch*COUNTS_PER_INCH_5202;

  }

    private void setDriveTrainEncoder(double inch){
        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()+(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()+(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()+(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=1;

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void stopDriveTrain(){
        FLeft.setPower(0);
        Fright.setPower(0);
        Bright.setPower(0);
        BLeft.setPower(0);
    }

    public void moveForwardAndBackword(double inch){

        setDriveTrainEncoder(inch);
        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
            FLeft.setPower(1);
            Fright.setPower(1);
            Bright.setPower(1);
            BLeft.setPower(1);
        }
       stopDriveTrain();

    }
    public void moveLeftRight(double inch) {

        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()-(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()+(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()-(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=1;
        if(inch<0){
            pwr= pwr *(-1);
        }

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
          FLeft.setPower(- pwr);
          Fright.setPower(pwr);
          BLeft.setPower(-pwr);
          Bright.setPower(pwr);

      }
      stopDriveTrain();
    }
    public void SlideLeftAndRight(double inch) {

        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()+(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()-(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()-(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=1;
        if(inch<0){
            pwr= pwr *(-1);
        }

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
            FLeft.setPower(- pwr);
            Fright.setPower(pwr);
            BLeft.setPower(-pwr);
            Bright.setPower(pwr);

        }
        stopDriveTrain();



    }
    private void DropExtendHold(){
        while (true) {
            if (DropExtendStart){
                DropExtendStart=false;
            runtime.reset();
            int timeout = 3;
            double pwr = 0;
            double TargetPosition = DropInch * COUNTS_PER_INCH_5202;
            int currentPosition = Drop.getCurrentPosition();
            int NewPosition = currentPosition + (int) TargetPosition;
            if (DropInch > 0) {
                pwr = 1;
                timeout = 5;
                Drop.setTargetPosition(NewPosition);// move up/forward to given inches.
            } else if (DropInch < 0) {
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

            while (IsBreak) {
                Drop.setPower(0.0012 * pwr);
            }
            Drop.setPower(0);

        }}

    }

    private void PickExtendHold(){
        while (true) {
            if (PickExtendStart){
                PickExtendStart=false;
                runtime.reset();
                int timeout = 3;
                double pwr = 0;
                double TargetPosition = PickInch * COUNTS_PER_INCH_5202;
                int currentPosition = Pick.getCurrentPosition();
                int NewPosition = currentPosition + (int) TargetPosition;
                if (PickInch > 0) {
                    pwr = 1;
                    timeout = 5;
                    Pick.setTargetPosition(NewPosition);// move up/forward to given inches.
                } else if (PickInch < 0) {
                    pwr = -1;
                    Pick.setTargetPosition(NewPosition); // go back to the starting position
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

            }}

    }


   public void DropExtendINCH(double Inches) {
       DropInch=Inches;
       DropExtendStart=true;
       //tmpExtend();
    }
    public void PickExtendINCH(double Inches) {
        PickInch=Inches;
        PickExtendStart=true;
        //tmpExtend();
    }

    public void OparateBreak(boolean IsSetBreak){
        IsBreak=IsSetBreak;
    }


}



//class
