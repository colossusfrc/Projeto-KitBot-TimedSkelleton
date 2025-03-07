// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.contantes.constantesRodas;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class Robot extends TimedRobot {
private enum Estado {
  zero, duzentos, idle 
}
private enum Estado1 {
  parado, coleta, entrega
}
private DutyCycleEncoder enconderBraco = new DutyCycleEncoder(0);
private double AnguloBraco = 0, offset = -74;

private CANSparkMax motor = new CANSparkMax(2, MotorType.kBrushed);
private CANSparkMax motorFrente1 = new CANSparkMax(constantesRodas.motorFrente1, MotorType.kBrushed);
private CANSparkMax motorFrente2 = new CANSparkMax(constantesRodas.motorFrente2, MotorType.kBrushed);
private CANSparkMax motorCosta1 = new CANSparkMax(constantesRodas.motorCosta1, MotorType.kBrushed);
private CANSparkMax motorCosta2 = new CANSparkMax(constantesRodas.motorCosta2, MotorType.kBrushed);

private CANSparkMax motorBraco = new CANSparkMax(8, MotorType.kBrushed); //cima
private CANSparkMax motorBraco1 = new CANSparkMax(7, MotorType.kBrushed); //baixo

private CANSparkMax motorIntake1 = new CANSparkMax(6, MotorType.kBrushless);
private CANSparkMax motorIntake2 = new CANSparkMax(5, MotorType.kBrushless);
private RelativeEncoder intek1;
private Joystick joystick = new Joystick(0);
private DifferentialDrive tank;
private PIDController pid;
private double kp = 0.01;
private double ki = 0;
private double kd = 0;
private PIDController pidBraco;
private double[] k = {0.003,0,0};

private Estado estado;
private Estado1 estadoBraco;
public Robot() {
  motorFrente1.restoreFactoryDefaults();
  motorFrente2.restoreFactoryDefaults();
  motorCosta1.restoreFactoryDefaults();
  motorCosta2.restoreFactoryDefaults();

  motorCosta2.setInverted(true);

  motorFrente1.follow(motorFrente2);
  motorCosta1.follow(motorCosta2);
  tank = new DifferentialDrive(motorCosta2, motorFrente2);
  tank.setMaxOutput(0.3);
  estado = Estado.idle;
  estadoBraco = Estado1.parado;
  pid = new PIDController(kp, ki, kd);
  pidBraco = new PIDController(k[0], k[1], k[2]);
  pidBraco.enableContinuousInput(-180, 180);

  intek1 = motorIntake1.getEncoder();
}


  @Override
  public void robotInit() {
  intek1.setPosition(0);
  }

  @Override
  public void robotPeriodic() {
    AnguloBraco = enconderBraco.get()*360 + offset;
    AnguloBraco-=(AnguloBraco>360|| AnguloBraco<0)? Math.signum(AnguloBraco)*360:0;
    if (Math.abs(AnguloBraco)>180) {
      AnguloBraco = -Math.signum(AnguloBraco)* (360 - Math.abs(AnguloBraco));
    }
    
    SmartDashboard.putNumber("Encoder", intek1.getPosition());
    SmartDashboard.putNumber("Braço", AnguloBraco);
    
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    //controle rodas
    double frente = -joystick.getRawAxis(1);
    double giro = -joystick.getRawAxis(4);
    tank.arcadeDrive(frente, giro);

    //controle intek
    if (joystick.getRawButtonPressed(4)) {
      estado = Estado.duzentos;
    }
    if (joystick.getRawButtonPressed(1)) {
      estado = Estado.zero;
    }
    switch (estado) {
      case zero:
        double tempoinicial = Timer.getFPGATimestamp();
        double alvo = 0;
        /*//proporcional
        double erro = alvo - intek1.getPosition();
        double erroInicial = erro;
        //integral
        double integral = 0;
        integral+=erro*(Timer.getFPGATimestamp()-tempoinicial);
        //derivativo
        double derivativo = erro - erroInicial/(Timer.getFPGATimestamp()-tempoinicial);
        //
        double kp = 0.01; 
        double ki = 0;
        double kd = 0;
        //saida final da potencia
        double potencia = erro * kp + integral * ki + derivativo * kd;*/
        double potencia = pid.calculate(intek1.getPosition(), alvo);
        potencia = (Math.abs(potencia) > 0.5)?0.5*Math.signum(potencia):potencia;
        motorIntake1.set(potencia);
        if (estado!=Estado.zero) {
          estado = Estado.duzentos;
        }
        break;
      case duzentos:
        tempoinicial = Timer.getFPGATimestamp();
        alvo = 200;
        //proporcional
        /*erro = alvo - intek1.getPosition();
        erroInicial = erro;
        //integral
        integral = 0;
        inte  gral+=erro*(Timer.getFPGATimestamp()-tempoinicial);
        //derivativo
        derivativo = erro - erroInicial/(Timer.getFPGATimestamp()-tempoinicial);
        kp = 0.01; 
        ki = 0;
        kd = 0;
        potencia = erro * kp + integral * ki + derivativo * kd;*/
        //formula complet
        potencia = pid.calculate(intek1.getPosition(), alvo); //formula simplificada
        potencia = (Math.abs(potencia) > 0.5)?0.5*Math.signum(potencia):potencia;
        motorIntake1.set(potencia);
        if (estado!=Estado.duzentos) {
          estado = Estado.zero;
        }
        break;
      default:
      motorIntake1.set(0);
        break;

        //Braço
    }
    if (joystick.getRawButtonPressed(6)) {
      estadoBraco = Estado1.parado;
    }
    if (joystick.getRawButtonPressed(3)) {
      estadoBraco = Estado1.coleta;
    }
    if (joystick.getRawButtonPressed(2)) {
      estadoBraco = Estado1.entrega;
    }
    
    switch (estadoBraco) {
      case parado:
        motorBraco1.set(0);
        if (estadoBraco != Estado1.parado) {
          
        }
        break;
      
      case coleta:
      pidBraco.setSetpoint(-30);
        Double potencia = pidBraco.calculate(AnguloBraco, -20);
        potencia*=((Math.signum(potencia)<0&&AnguloBraco<-30.0)||(Math.signum(potencia)>0&&AnguloBraco>155.0))?-1.0:1.0;
        potencia = (Math.abs(potencia)>0.2)?Math.signum(potencia)*0.2:potencia;
        motorBraco1.set(potencia);
        if (estadoBraco != Estado1.coleta) {
          estadoBraco = Estado1.parado;
          pidBraco.reset();
        }
        break;

      case entrega:
        pidBraco.setSetpoint(150);
        Double potencia1 = pidBraco.calculate(AnguloBraco, 150);
        potencia1*=((Math.signum(potencia1)<0&&AnguloBraco<-30.0)||(Math.signum(potencia1)>0&&AnguloBraco>155.0))?-1.0:1.0;
        potencia = (Math.abs(potencia1)>0.2)?Math.signum(potencia1)*0.2:potencia1;
        motorBraco1.set(potencia1);
        if (estadoBraco != Estado1.entrega) {
          estadoBraco = Estado1.parado;
          pidBraco.reset();
        }
        break;
    }
    
    }

///////////////////////////////////////


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
