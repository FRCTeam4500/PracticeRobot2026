package frc.robot.subsystems.shooter;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.WiringConstants.ElevatorWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.function.Consumer;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Loggable {

    private Motor shooterMotor;

    private Consumer<IdleMode> setIdleMode;

    public Shooter() {
        
        shooterMotor = 
            Motor.fromSparkMax(
                /*THE CAD ID, should put in wiring constants then import those to this file */0, 
                false,
                (SparkMax spark) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1 / 63.1167979003);
              config.encoder.velocityConversionFactor(1 / 63.1167979003);
              config.smartCurrentLimit(60);
              REVLibError err =
                  spark.configure(
                      config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
              if (!err.equals(REVLibError.kOk)) {
                HoundLog.logFault(
                    "[ShooterMotor] Extension Motor Config Error: " + err.name(), AlertType.kError);
              }
              setIdleMode =
                  mode -> {
                    config.idleMode(mode);
                    REVLibError idlerr =
                        spark.configure(
                            config,
                            ResetMode.kResetSafeParameters,
                            PersistMode.kNoPersistParameters);
                    if (!idlerr.equals(REVLibError.kOk)) {
                      HoundLog.logFault(
                          "[Elevator] Idle Mode Config Error: " + err.name(), AlertType.kError);
                    } else {
                      HoundLog.clearFault("[Elevator] Idle Mode Config Error: " + err.name());
                    }
                  };
                }, 
                null, 
                0, 
                null, 
                null, 
                null);
        shooterMotor.getSysIDCommands("ShooterMotor", 0.5, 2, 5).putOnDashboard("ShooterMotor", this);
    }

    /**
     *  @return A command that runs the shooter motor
     */
    public Command runShooterMotor() {
        return Commands.runOnce(
                    () -> {
                        shooterMotor.setTarget(4.0);
                    }, this)
                .andThen(
                    Commands.waitUntil(
                        () -> {
                            return shooterMotor.atTarget();
                        }
                    )
                );
    }

    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}