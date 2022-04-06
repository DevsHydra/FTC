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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Controle 1")
//@Disabled                            // Comment this out to add to the opmode list
public class Controle1 extends LinearOpMode
{
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
//    private DcMotor Esteira;

    double speed =1;
    double angulofinalE;
    double angulofinalD;
    double angulorealE = 0;
    double angulorealD = 0;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    @Override public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
//        Esteira = hardwareMap.get(DcMotor.class, "MMT");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
//        Esteira.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();



        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double y2 = -gamepad1.right_stick_y;
            double d = Math.hypot(x,y);
            double d2 = Math.hypot(x2, y2);
            double angulorad = (d!=0)?Math.asin(y/d):0;
            double angulo = Math.toDegrees(angulorad);
            double angulorad2 = (d2!=0)?Math.asin(y2/d2):0;
            double anguloD = Math.toDegrees(angulorad2);
            final double PI = Math.PI;
            double fMD = 0;
            double fME = 0;
            double forcax = 0;
            double forcax2 = 0;
            double forcay = 0;
            boolean r1 = gamepad1.right_bumper;
            boolean l1 = gamepad1.left_bumper;
            double currangle = angles.firstAngle;
            double anguloRobo = 0;



            //Control Speed
            speed = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ?0.25:speed ;



            // Pegar ângulo Joy Esquerdo
            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x >= 0)) {
                angulofinalE = angulo;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x <= 0)) {
                angulofinalE = 180-angulo;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x <= 0)) {
                angulofinalE = -angulo-180;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x >= 0)) {
                angulofinalE = angulo;
            }

            // Pegar ângulo Joy Direito
            //1º QUADRANTE
            if((x2 >= 0 ) && (y2 >= 0)){
                angulofinalD = anguloD;
                telemetry.addData("Condicao","1 quadrante");
            }
            // 2º QUADRANTE
            else if((x2 < 0 ) && (y2 >= 0)) {
                angulofinalD = 180-anguloD;
                telemetry.addData("Condicao","2 quadrante");
            }
            // 3º QUADRANTE
            else if((x2 < 0 ) && (y2 < 0)){
                angulofinalD = -anguloD-180;
                telemetry.addData("Condicao","3 quadrante");
            }
            // 4º QUADRANTE
            else if((x2 >= 0) && (y2 < 0)) {
                angulofinalD = anguloD;
                telemetry.addData("Condicao","4 quadrante");
            }
            // MOVIMENTAÇÃO POR ÂNGULO


            anguloRobo = getAngle();
            //Analógico Esquerdo
            angulorealE =  angulofinalE - anguloRobo;
            forcay = Math.sin(Math.toRadians(angulorealE)) * d;
            forcax = Math.cos(Math.toRadians(angulorealE)) * d;
            telemetry.addData("forcay: ",forcay);

            //Analógico Direito
            angulorealD =  angulofinalD - anguloRobo;
            telemetry.addData("AnguloREALD: ", angulorealD);
            telemetry.addData("D2: ", d2);
            forcax2 = Math.cos(Math.toRadians(angulorealD)) * d2;
            telemetry.addData("forcax2: ",forcax2);

            //1º QUADRANTE FORÇA

            angulorad = (d!=0)?Math.asin(forcay/d):0;

            if((forcax>= 0) && (forcay>=0)) {
                fME = d;
                fMD = (((4 / PI) * angulorad) - 1) * d;

            }
            //2º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay>=0)) {
                fME = (((4 / PI) * angulorad) - 1) * d;
                fMD = d;
            }
            //3º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay<0)) {
                fME = -d;
                fMD = (((4 / PI) * angulorad) + 1) * d ;
            }
            //4º QUADRANTE FORÇA
            else if ((forcax>= 0) && (forcay<0)) {
                fME = (((4 / PI) * angulorad) + 1) * d;
                fMD = -d;
            }

//             Set Force

//            if(r1){
//                Esteira.setPower(1);
//            }else if(l1){
//                Esteira.setPower(-1);
//            }else{
//                Esteira.setPower(0);
//            }
//            telemetry.addData("FMD" , fMD);
//            telemetry.addData("FME" , fME);
//            telemetry.addData("FMD _ forcax2: " , fMD - forcax2);
//            telemetry.addData("FME + forcax2: " , fME + forcax2);
//            telemetry.addData("forcax2: " , forcax2);
            telemetry.addData("primeiro angulo: ",angles.firstAngle);
            telemetry.addData("Curr angle: ",anguloRobo);
            telemetry.update();
            RMF.setPower((fMD-forcax2) * speed);
            LMF.setPower((fME+forcax2) * speed);
            RMB.setPower((fME-forcax2) * speed);
            LMB.setPower((fMD+forcax2) * speed);
        }

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------



    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

}