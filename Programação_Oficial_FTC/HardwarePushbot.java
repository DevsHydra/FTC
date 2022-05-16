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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {  // Classe de definição de motores e servos
    /* local OpMode members. */
    HardwareMap hwMap           =  null;

//==============================CRIAÇÃO DOS MOTORES E SERVOS========================================

//  Motores de locomoção:

    public DcMotor  LMF;    //Motor Mecanum - Esquerda Frontal
    public DcMotor  RMF;    //Motor Mecanum - Direita Frontal
    public DcMotor  LMB;    //Motor Mecanum - Esquerda Traseiro
    public DcMotor  RMB;    //Motor Mecanum - Direita Traseiro;
    public DcMotor Esteira; //Motor Core Hex - Esteira

//  Motores de sistema:

    public DcMotor  MB;     //Motor UltraPlanetary (100:1) - Sistema de elevação (Braço)
    public DcMotor  ML;     //Motor Core Hex - Lagosta
    public DcMotor  MC;     //Motor Core Hex - Coleta

//  Servos:

    public CRServo SPR;     //Servo Continuo Girar Carrossel - Direita
    public CRServo SPL;     //Servo Continuo Girar Carrossel - Esqueda

    public Servo SL;        //Servo Posição - Pinça

    BNO055IMU imu;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    private ElapsedTime period  = new ElapsedTime();




    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


//==========================DEFININDO INNICIALIZAÇÃO DOS MOTORES E SERVOS===========================

        LMF = hwMap.get(DcMotor.class, "LMF");
        LMB = hwMap.get(DcMotor.class, "LMB");
        RMF = hwMap.get(DcMotor.class, "RMF");
        RMB = hwMap.get(DcMotor.class, "RMB");

        Esteira = hwMap.get(DcMotor.class, "MMT");

        MB = hwMap.get(DcMotor.class, "MB");
        MC = hwMap.get(DcMotor.class,"MC");
        ML = hwMap.get(DcMotor.class, "ML");

        SL = hwMap.get(Servo.class, "SL");

        SPR = hwMap.crservo.get("SPR");
        SPL = hwMap.crservo.get("SPL");

//==========================DEFININDO DIREÇÃO DOS MOTORES E SERVOS==================================

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);      //REVERSE - Direção Reversa
        LMB.setDirection(DcMotorSimple.Direction.FORWARD);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);      //FORWARD - Direção Normal
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        Esteira.setDirection(DcMotorSimple.Direction.FORWARD);

        MB.setDirection(DcMotorSimple.Direction.REVERSE);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        ML.setDirection(DcMotorSimple.Direction.FORWARD);

        SL.setDirection(Servo.Direction.FORWARD);

//====================================TRAVANDO OS MOTORES===========================================

        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//==========================DEFININDO MOTORES QUE NÃO IRAM USAR ENCODER=============================

        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Esteira.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//==========================DEFININDO MOTORES QUE IRAM USAR ENCODER=================================

        MB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ML.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

////=======================================RESETANDO ENCODERS=========================================

        MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ML.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        setAllPower(0);     //Definindo força 0 para todos os motores

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //Set power to all motors
    public void setAllPower(double p){     //Metodo para definir a mesma força para todos os motores
        setMotorPower(p,p,p,p,p);
    }

    public void setMotorPower(double lF, double rF, double lB, double rB, double eS){
        LMF.setPower(lF);
        LMB.setPower(lB);
        RMB.setPower(rB);
        RMF.setPower(rF);
        Esteira.setPower(eS);
    }
}

