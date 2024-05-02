import navx
import wpilib
import wpimath
from wpimath import kinematics
from wpimath.geometry import Translation2d, Rotation2d
from newSwerb import module as SwerbModule

class Drivetrain():
   def __init__(self):
      self.frontrightlocation = Translation2d(2.08, 2.08)  # 0.5 ,0.5, p, p
      self.frontleftlocation = Translation2d(-2.08, 2.08)  # 0.5 ,-0.5 n,p
      self.backleftlocation = Translation2d(-2.08, -2.75)  # -0.5 ,-0.75 n, n
      self.backrightlocation = Translation2d(2.08, -2.75)  # -0.5, 0.75 p, n

      self.frontLeft = SwerbModule.Module(3,13,6)
      self.frontRight = SwerbModule.Module(2,10,5)
      self.backLeft = SwerbModule.Module(1,12,7)
      self.backRight = SwerbModule.Module(8,11,4)

      self.gyro = navx.AHRS.create_i2c(wpilib.I2C.Port.kMXP)

      self.gyro.enableLogging(True)

      self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontleftlocation,
            self.frontrightlocation,
            self.backleftlocation,
            self.backrightlocation,
        )

      self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
         self.kinematics,
         Rotation2d().fromDegrees(-self.gyro.getAngle()),
         (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition(),
         ),
      )

   def drive(self,x,y,rot):
      swerveModuleStates = self.kinematics.toSwerveModuleStates(
         wpimath.kinematics.ChassisSpeeds.discretize(
            (
               wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                  x, y, rot, Rotation2d.fromDegrees(-self.gyro.getAngle())
               )
      ),
         0.01
      ))
      wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
         swerveModuleStates,12
      )
      self.frontLeft.setState(swerveModuleStates[0])
      self.frontRight.setState(swerveModuleStates[1])
      self.backLeft.setState(swerveModuleStates[2])
      self.backRight.setState(swerveModuleStates[3])

   def updateOdometry(self) -> None:
      """Updates the field relative position of the robot."""
      self.odometry.update(
         Rotation2d.fromDegrees(-self.gyro.getAngle()),
         (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition(),
         ),
      )
