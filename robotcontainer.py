from auto.threeNote import ThreeNote
from components.arm import Arm
from components.autoAim import AutoAim
from components.drivetrain import DriveTrain
from components.hanger import Hanger
from components.physics import Physics
from components.shooter import shooter
from components.state_handler import StateHandler
from components.vision import Vision
from auto.path_test import PathTest



class RobotContainer:
   def __init__(self) -> None:
      self.drivetrain = DriveTrain()
      self.vision = Vision()
      self.auto_aim = AutoAim()
      self.physics = Physics()
      self.hanger = Hanger()
      self.shooter = shooter()
      self.arm = Arm()
      self.auto = PathTest()

   def getAutoCommand(self):
      # return self.drivetrain.getAutonomousCommand()
      pass
