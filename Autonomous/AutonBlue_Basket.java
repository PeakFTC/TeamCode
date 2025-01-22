package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonBlue_Basket extends LinearOpMode {
    IntoTheDeep_Auto_Drivetrain DriveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new IntoTheDeep_Auto_Drivetrain(hardwareMap,telemetry);
        DriveTrain.configDriveTrain();
        waitForStart();
        DriveTrain.PickExtendINCH(9.5);
        sleep(10000);
        DriveTrain.PickExtendINCH(-9.5);
       // DriveTrain.moveForwardAndBackword(20);
      //  DriveTrain.stopDriveTrain();
DriveTrain.SpecClawSrvo.setPosition(peakFTCServo.SERVO_POSITION.POSITION_OPEN);
        //DriveTrain.SlideLeftAndRight(15);//slide to the left
       // DriveTrain.stopDriveTrain();

        DriveTrain.OparateBreak(true);
        DriveTrain.DropExtendINCH(19);

        sleep(1000);

        //DriveTrain.moveForwardAndBackword(6.5);//slide to the right
        //DriveTrain.stopDriveTrain();

        DriveTrain.OparateBreak(false);
        sleep(10);
        DriveTrain.DropExtendINCH(-6);

        DriveTrain.OparateBreak(false);
        sleep(10);
        DriveTrain.DropExtendINCH(0);

        sleep(10000);



    }
}
