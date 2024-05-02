import phoenix6.hardware
import rev
import wpimath
from wpimath import controller, trajectory, kinematics,geometry



class Module():
   def __init__(self,TurnCan,EncoderCan,DriveCan):

      self.driveMotor = rev.CANSparkMax(DriveCan, rev.CANSparkMax.MotorType.kBrushless)
      self.turnMotor = rev.CANSparkMax(TurnCan,  rev.CANSparkMax.MotorType.kBrushless)

      self.driveEncoder = self.driveMotor.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.turnEncoder = phoenix6.hardware.CANcoder(EncoderCan, "rio")


      self.turningPIDController = wpimath.controller.PIDController(1,0,0)


   def getState(self) -> wpimath.kinematics.SwerveModuleState:
      """Returns the current state of the module.

      :returns: The current state of the module.
      """
      return wpimath.kinematics.SwerveModuleState(
         self.driveEncoder.getVelocity(),
         wpimath.geometry.Rotation2d(self.turnEncoder.get_absolute_position().value_as_double)
      )

   def getPosition(self):
      return wpimath.kinematics.SwerveModulePosition(
         self.driveEncoder.getVelocity(),
         wpimath.geometry.Rotation2d(self.turnEncoder.get_absolute_position().value_as_double),
      )

   def setState(self, desiredState:wpimath.kinematics.SwerveModuleState):
      encoderRotation = wpimath.geometry.Rotation2d(self.turnEncoder.get_absolute_position().value_as_double)

      state = wpimath.kinematics.SwerveModuleState.optimize(desiredState, encoderRotation)

      state.speed *= (state.angle - encoderRotation).cos()

      turnOutput = self.turningPIDController.calculate(
         self.turnEncoder.get_absolute_position().value_as_double, state.angle.radians()
      )

      self.driveMotor.set(state.speed)
      self.turnMotor.set(turnOutput)


