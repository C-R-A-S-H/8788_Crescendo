import wpimath.geometry
import wpimath.trajectory
import wpimath.controller
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

class PathTest():
   def __init__(self):


      self.trajectoryConfig = wpimath.trajectory.TrajectoryConfig(
      2,
      1
         )

      self.path = [
      wpimath.geometry.Translation2d(1, 0)

      ]

      self.trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
         wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
         self.path,
         wpimath.geometry.Pose2d(2, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
         self.trajectoryConfig
      )

      xKp = 1
      yKp = 1
      zKp = 1

      self.xPID = wpimath.controller.PIDController(xKp,0,0)
      self.yPID = wpimath.controller.PIDController(yKp, 0, 0)
      self.zPID = wpimath.controller.PIDController(zKp, 0, 0)


   def timeToSpeeds(self,currentTime:float,currentPose:Pose2d,heading:float):
      wantedState = self.trajectory.sample(currentTime)

      wantedPose = wantedState.pose
      x = self.xPID.calculate(currentPose.X(),wantedPose.X())
      y = self.yPID.calculate(currentPose.Y(),wantedPose.Y())
      z = self.zPID.calculate(currentPose.rotation().degrees(), Rotation2d().fromDegrees(heading).degrees())

      return ChassisSpeeds.fromRobotRelativeSpeeds(x,-y,z,Rotation2d().fromDegrees(heading))

