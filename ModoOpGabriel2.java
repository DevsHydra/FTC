package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class ModoOpGabriel2 extends LinearOpMode {
    private DcMotor mr;
    private DcMotor ml;
    boolean claiton, cleberson;
    double robert,bebeto;

    int i,j;
    double[][] vetpower = new double[350][2];


    public void teleopInit(){
        for(i=0;i<350;i++){
            for(j=0;j<2;j++){
                vetpower[i][j]=0;
            }
        }
        i=0;
        j=0;
    }


    @Override
    public void runOpMode() {

        ml = hardwareMap.get(DcMotor.class, "ml");
        mr = hardwareMap.get(DcMotor.class, "mr");


        waitForStart();
        teleopInit();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //double d = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            double d = Math.sqrt(Math.pow(x,2) + Math.pow(-y,2));
            double cos = 0;
            cos = x/d;
            //double sen = y/d;
            double sen = -y/d;
            /*
            boolean grB = gamepad1.dpad_up;
            boolean glB = gamepad1.dpad_down;
            boolean grF = gamepad1.dpad_right;
            boolean glF = gamepad1.dpad_left;
            */


            double forceMotorR = 0;
            double forceMotorL = 0;

            // RIGHT QUADRANTE
            if(gamepad1.dpad_right){
                forceMotorR = -1; //
                forceMotorL = 1;
            }
            // LEFT QUADRANTE
            else if(gamepad1.dpad_left){
                forceMotorR = 1;
                forceMotorL = -1;
            }
            // 1º QUADRANTE
            if((x >= 0 ) && (y >= 0)){
                forceMotorR=y-x;
                forceMotorL=y-x;

                /*
                forceMotorR = ((2 * sen) - 1) * d;
                forceMotorL = d;

                 */
            }
            // 2º QUADRANTE
            else if((x < 0 ) && (y >= 0)) {
                forceMotorR=y-x;
                forceMotorL=y-x;
                /*
                forceMotorR = d;
                forceMotorL = ((2 * sen) - 1) * d;

                 */
            }
            // 3º QUADRANTE
            else if((x < 0 ) && (y < 0)){
                forceMotorR=y-x;
                forceMotorL=y-x;
                /*
                forceMotorR = -d;
                forceMotorL = ((2 * sen) + 1) * d;

                 */
            }
            // 4º QUADRANTE
            else if((x >= 0 ) && (y < 0)) {
                forceMotorR=y-x;
                forceMotorL=y-x;
                /*
                forceMotorR = ((2 * sen) + 1) * d;
                forceMotorL = -d;

                 */
            }
            if(gamepad1.a){bebeto++;}
            if(bebeto%2!=0){claiton = true;}else{ claiton = false;}
            if(claiton){i++;
            }
            if(i==1){
                telemetry.addData("Gravador:", true);
                vetpower[j][0] = forceMotorL;
                vetpower[j][1] = forceMotorR;
                j++;
            }else{
                telemetry.addData("Gravador:", false);
            }
            if(i>1){
                telemetry.addData("motor esquerdo", vetpower[i-2][0]);
                telemetry.addData("motor direito", vetpower[i-2][1]);
            }

            if(gamepad1.b){robert++;}
            if(robert%2!=0){
                mr.setPower((vetpower[i][0]));
                ml.setPower((vetpower[i][1]));
                i++;
            }else{cleberson = false;}

            telemetry.addData("Motor Esquerdo", forceMotorL);
            telemetry.addData("Motor Direito", forceMotorR);

            telemetry.addData("Magnitude", d);
            telemetry.addData("Seno", sen);

            mr.setPower(forceMotorR);
            ml.setPower(-forceMotorL);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
