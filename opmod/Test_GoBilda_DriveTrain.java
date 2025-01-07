package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

   private  Thread SampleIntakingthread;

   private Thread DriveTrainThread;

   private Thread DropLinearSlideThread;



    private Servo hand;
    private  Servo claw;
    private Servo RoClaw;
    private Servo SpecClaw;

    private Servo arm;

    private Servo outake;
    private double roClowPos;
    static final double     COUNTS_PER_MOTOR_REV    = 751.8 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void init() {

        IsLeftTrigger=false;
        IsRightTrigger=false;
        ExtendDrop=false;
        DropInchToGo=0;
        FLeft=hardwareMap.dcMotor.get("FLeft");
        BLeft=hardwareMap.dcMotor.get("BLeft");
        Fright=hardwareMap.dcMotor.get("FRight");
        Bright=hardwareMap.dcMotor.get("BRight");
        hand=hardwareMap.get(Servo.class,"hand");
        claw=hardwareMap.get(Servo.class,"claw");
        RoClaw=hardwareMap.get(Servo.class,"RoClaw");
        SpecClaw=hardwareMap.get(Servo.class,"SpecClaw");
        arm=hardwareMap.get(Servo.class,"arm");
        outake=hardwareMap.get(Servo.class,"outake");
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

        RoClaw.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.FORWARD);
        SpecClaw.setDirection(Servo.Direction.FORWARD);
        outake.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.FORWARD);
        //Outake
        Drop.setDirection(DcMotorSimple.Direction.FORWARD);
        Drop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo Powers
        hand.setPosition(0);
        RoClaw.setPosition(0.45);
        SpecClaw.setPosition(0);//
        arm.setPosition(0.093);
        outake.setPosition(0.45);
        //claw.setPosition(0);
        ClawOpen();

//DriveTrain Encoders
       // FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      // BLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       //Fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        roClowPos=0;
        // create/initialize all the required threads
        CreateSampleTransferThread();
        CreatSampleIntakeThread();
        CreateDriveTrainThread();
        CreateDropLinearSlideThread();

    }

    private void CreateDropLinearSlideThread() {
        DropLinearSlideThread=new Thread(() ->{
            while (true){
                if (ExtendDrop){
                    DropExtendINCH(DropInchToGo);
                    while (ExtendDrop) {
                        Drop.setPower(0.001);
                    }
                }
            }
        }
        );  }

    private void CreateDriveTrainThread() {
        DriveTrainThread=new Thread(() ->{
            while (true){
                double Position=gamepad1.right_stick_y;
                double SidePosition=gamepad1.right_stick_x;
                double strafePosition=gamepad1.left_stick_x;
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


    private void CreateSampleTransferThread(){
        SampleTransferThread=new Thread(() -> {
            while(true) {
                if (IsLeftTrigger) {
                    sampleDeposit();
                    IsLeftTrigger=false;
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });
    }

    private void CreatSampleIntakeThread(){
        SampleIntakingthread=new Thread(() -> {
            while (true) {
               if (IsRightTrigger) {
                   GoToTheSample();
                   IsRightTrigger=false;

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
    public void start (){
        SampleTransferThread.start();
        SampleIntakingthread.start();
        DriveTrainThread.start();
        DropLinearSlideThread.start();

    }
    @Override
    public void loop() {


        telemetry.addData("Linear slid Currently at", " at %7d",
                Pick.getCurrentPosition());

        telemetry.update();

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


        if (gamepad1.right_trigger >0) {
            IsRightTrigger=true;
        }

        if (gamepad1.y){
            GrabTheSample();
            //ClawClose();
        }

        if (gamepad1.left_trigger>0){
            //sampleDeposit();
            IsLeftTrigger=true;
        }
        if (gamepad1.a) {
            //ClawOpen();
            prepareHandToGrab();
        }
        if (gamepad1.b){
            //ClawOpen();
            ExtendDrop=false;
            isHold=false;
            dropTheSample();
        }
        if(gamepad1.left_bumper)
        {
            roClowPos+=0.025;
            RoClaw.setPosition(roClowPos);
        }
        if(gamepad1.right_bumper)
        {
            roClowPos-=0.025;
            RoClaw.setPosition(roClowPos);
        }
        Pick.setPower(gamepad2.right_trigger);
         if (gamepad1.dpad_left){
             GrabSpec();
         }
         if (gamepad1.dpad_right){
             DropSpec();
         }

         if (gamepad1.dpad_up){
             DropInchToGo=22;
             ExtendDrop=true;
         }
         if (gamepad1.dpad_down){
             ExtendDrop=false;
             DropExtendINCH(-4);//TO get it back to the zero position
         }
         if(gamepad2.x){
             DropExtendINCH(20);
             Drop.setPower(0.005);
             Delay(10);
             Drop.setPower(0);
         }
         if(gamepad2.y){
             DropExtendINCH(0);
         }

         double temp=gamepad1.left_stick_y;
         if (temp !=0) {
             ExtendDrop=false;
         }

         Drop.setPower(-gamepad1.left_stick_y); // reverse

    }

    private void prepareHandToGrab() {
        ClawOpen();
        handGrab();
    }


    //DRIVETRAIN MOVEMENT CODE/ FUNCTIONS
    private void MoveForward( double Position){
        FLeft.setPower(ScalePower(Position));
        Fright.setPower(ScalePower(Position));
        BLeft.setPower(ScalePower(Position));
        Bright.setPower(ScalePower(Position));
    }
    private void MoveSide(double position){
        if (isDriveTranDirectionSamples) {
            //Drivetain deraction for sample
            FLeft.setPower(-ScalePower(position));
            BLeft.setPower(-ScalePower(position));
            Fright.setPower(ScalePower(position));
            Bright.setPower(ScalePower(position));
        }
        else {
            //Drivetrain direction for specimen
            FLeft.setPower(ScalePower(position));
            BLeft.setPower(ScalePower(position));
            Fright.setPower(-ScalePower(position));
            Bright.setPower(-ScalePower(position));}
    }

    private void Strafe(double slide) {
        FLeft.setPower(-ScalePower(slide));
        Fright.setPower(ScalePower(slide));
        BLeft.setPower(ScalePower(slide));
        Bright.setPower(-ScalePower(slide));
    }
    private void SpecScore (){

    }

    private void handDeposit (){
        hand.setPosition(0);
    }
    private void handGrab (){
        hand.setPosition(1);
    }
    private void PickExtendINCH(double Inch){
        double pwr=0;
        double TargetPosition= Inch*COUNTS_PER_INCH;
        int currentPosition= Pick.getCurrentPosition();
        int NewPosition  = currentPosition+(int) TargetPosition;
        if(Inch> 0){
            pwr = 1;
            Pick.setTargetPosition(NewPosition);// move up/forward to given inches.
        }else{
            pwr=-1;
            Pick.setTargetPosition(0); // go back to the starting position
        }
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pick.setPower(pwr);
        while(Pick.isBusy()){
            Pick.setPower(pwr);
        }
        Pick.setPower(0);


        /*
        double TargetPosition= Inch*COUNTS_PER_INCH;
        int currentPosition= Pick.getCurrentPosition();
        int NewPosition  = currentPosition+(int) TargetPosition;

       if (Inch<0){
           Pick.setPower(-1);
           runtime.reset();
           while(NewPosition < Pick.getCurrentPosition() && runtime.seconds()<3){
               Pick.setPower(-1);
           }
       }
       else if (Inch>0){

           Pick.setPower(1);
           runtime.reset();
           while(NewPosition > Pick.getCurrentPosition() && runtime.seconds()<3){
               Pick.setPower(1);
           }
       }
       else {
           Pick.setPower(0);
       }

        Pick.setPower(0);
         */

    }

    private void DropExtendINCH(double Inch){
        double pwr=0;
        double TargetPosition= Inch*COUNTS_PER_INCH;
        int currentPosition= Drop.getCurrentPosition();
        int NewPosition  = currentPosition+(int) TargetPosition;
        if(Inch> 0){
            pwr = 1;
            Drop.setTargetPosition(NewPosition);// move up/forward to given inches.
        }else if(Inch<0){
            pwr=-1;
            Drop.setTargetPosition( NewPosition); // go back to the starting position
        }
        else{
            pwr=-1;
            Drop.setTargetPosition( 0); // go back to the starting position
        }
        Drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drop.setPower(pwr);
        while(Drop.isBusy()){
            Drop.setPower(pwr);
        }

        if(Inch>0) {
            Drop.setPower(0.001 * pwr);
        }else{
            Drop.setPower(0);
        }
        /*
        if (Inch<0){
            NewPosition=0;
            Drop.setPower(-1);
            runtime.reset();
            while(NewPosition < Drop.getCurrentPosition() && runtime.seconds()<3){
                Drop.setPower(-1);
            }
            Drop.setPower(0);
        }
        else if (Inch>0){

            Drop.setPower(1);
            runtime.reset();
            while(NewPosition > Drop.getCurrentPosition() && runtime.seconds()<5){
                Drop.setPower(1);
            }
            Drop.setPower(0.001); // just to hold the position until sample drop completes
        }
        else {
            Drop.setPower(0);
        }
         */



    }
    private void ClawOpen(){
        claw.setPosition(0.4);// Open 50% or less
    }
    private void ClawClose(){
        claw.setPosition(0.7);// close the claw all the way
    }
    //0. ensure that the hand is at the resting position
    //1. Extend linear slide
    //2. Set hand set position to grab
    //3. open claw ready to grab specimen
    //ensure that the RoClaw is in the correct position
    private void GoToTheSample (){
        handDeposit();
        armIntake();
        PickExtendINCH(10);
        handGrab();
        ClawOpen();
    }
    private void GrabTheSample() {
        ClawClose();
        Delay(0.5);
        handDeposit();
    }
    private void setRotateClawAtReset(){
        RoClaw.setPosition(0.45);
    }
    private void sampleDeposit () {
        armIntake();
        setRotateClawAtReset();
        PickExtendINCH(-10);
        outake.setPosition(0);
        Delay(0.5);
        ClawOpen();
        Delay(0.25);
        arm.setPosition(0.75);
        ScoreSample();
    }

    private void ScoreSpec (){
        DropExtendINCH(23);
        isHold=true;
        while (isHold) {
            Drop.setPower(0.001);
        }
    }
    private void ScoreSample (){
        ExtendDrop=false;
        DropExtendINCH(27);
        isHold=true;
        while(isHold) {
            Drop.setPower(0.001);
        }

    }
    private void dropTheSample(){
        outake.setPosition(0.8);
        Drop.setPower(0.005);
        Delay(0.5);
        armIntake();
        Drop.setPower(0.005);
        Delay(.5);
        isHold=false;
        DropExtendINCH(0); // reset to 0 position
    }

    private void Delay (double Seconds){
        runtime.reset();
        while (runtime.seconds()<Seconds);
    }
//\\\//\*:)(:
    private void GrabSpec (){
        SpecClaw.setPosition(1);
    }
    private void DropSpec () {
        SpecClaw.setPosition(0);
    }

    private void armIntake () {
        arm.setPosition(0.099);
        outake.setPosition(0.5);
    }
    private void armout () {
        outake.setPosition(1);
        Delay(2);
        arm.setPosition(0.65);
    }

    /*private void DepositSpecimen() {
        handGrab();
        ClawOpen();
    }
 */

    private double ScalePower(double input) {
        return Math.max(-1, Math.min(1, input));
    }
}
