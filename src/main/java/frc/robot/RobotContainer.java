// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.Constants.Trajetoria;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.Coletor.ColectingCommand;
import frc.robot.commands.Coletor.EjectingCommand;
import frc.robot.commands.Coletor.PointLow;
import frc.robot.subsystems.ColetorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Aqui iniciamos o swerve
  private final  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ColetorSubsystem coletor = new ColetorSubsystem();


  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  // Controle de Xbox, troque para o qual sua equipe estará utilizando
  private Joystick DriverJoystick = new Joystick(Controle.DriverJoystick);

  public RobotContainer() {

    swerve.setDefaultCommand(getAutonomousCommand());

    //  Definimos o comando padrão como a tração
      swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(DriverJoystick.getRawAxis(1), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(DriverJoystick.getRawAxis(0), Constants.Controle.DEADBAND),
      () -> DriverJoystick.getRawAxis(4)));

      NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
    
    // Configure the trigger bindings
    configureBindings();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    /*
   new JoystickButton(DriverJoystick, 5) // Botão 1 do joystick (verifique se esse é o botão correto)
    .toggleOnTrue(new elevatorCommand(elevatorSubsystem, Constants.ElevatorContants.KuPlimit));

    new JoystickButton(DriverJoystick, 6 ) // Botão 1 do joystick (verifique se esse é o botão correto)
    .toggleOnTrue(new elevatorCommand(elevatorSubsystem, Constants.ElevatorContants.KcolectLimit));




    new JoystickButton(DriverJoystick, DriverJoystick.getPOV(1)) // Botão 1 do joystick (verifique se esse é o botão correto)
    .whileTrue(new IntakeCommand(intakeSubsystem, Constants.intakeConstants.kColectingIntake));

    new JoystickButton(DriverJoystick, DriverJoystick.getPOV(2)) // Botão 1 do joystick (verifique se esse é o botão correto)
    .whileTrue(new IntakeCommand(intakeSubsystem, Constants.intakeConstants.ksafePosition));




        new JoystickButton(DriverJoystick, 1) // Botão 1 do joystick (verifique se esse é o botão correto)
    .whileTrue(new ColectingCommand(coletor));

    new JoystickButton(DriverJoystick, 2) // Botão 1 do joystick (verifique se esse é o botão correto)
    .whileTrue(new EjectingCommand(coletor));

   // new JoystickButton(DriverJoystick, ) // Botão 1 do joystick (verifique se esse é o botão correto)
    //.whileTrue(new PointLow(coletor));
 */
  }

  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    // Aqui retornamos o comando que está no selecionador
    return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true);
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
