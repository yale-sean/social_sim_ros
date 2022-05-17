#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

import termios, sys , tty
def _getch():
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(fd)
      ch = sys.stdin.read(1)     #This number represents the length
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch

class KeyboardController(object):
  STEPSIZE = 1
  MAXVEL = 1000
  def __init__(self):
    rospy.init_node("keyboard_controller")
    self.lpub = rospy.Publisher("/wheel/left", Int32, queue_size=10)
    self.rpub = rospy.Publisher("/wheel/right", Int32, queue_size=10)

    self.leftcmd = 0
    self.rightcmd = 0

    print("Ready. Press q to quit.")

    while True:
      c = _getch()
      print(c)
      #print(c.encode('utf-8').hex())
      if c == 'q' or c.encode('utf-8').hex() == '03':
        print("Quitting")
        break
      elif c == 'w':
        if self.leftcmd != self.rightcmd:
          cmd = max(self.leftcmd, self.rightcmd)
          self.leftcmd = cmd
          self.rightcmd = cmd
        else:
          self.leftcmd += self.STEPSIZE
          self.rightcmd += self.STEPSIZE
      elif c == 's':
        if self.leftcmd != self.rightcmd:
          cmd = min(self.leftcmd, self.rightcmd)
          self.leftcmd = cmd
          self.rightcmd = cmd
        else:
          self.leftcmd -= self.STEPSIZE
          self.rightcmd -= self.STEPSIZE
      elif c == 'x':
        self.leftcmd = 0
        self.rightcmd = 0
      elif c == 'a':
        self.leftcmd -= self.STEPSIZE
        self.rightcmd += self.STEPSIZE
      elif c == 'd':
        self.leftcmd += self.STEPSIZE
        self.rightcmd -= self.STEPSIZE
      self.leftcmd = min(self.leftcmd, self.MAXVEL)
      self.rightcmd = min(self.rightcmd, self.MAXVEL)
      print(f"[{self.leftcmd}|{self.rightcmd}]")
      self.lpub.publish(self.leftcmd)
      self.rpub.publish(self.rightcmd)
      rospy.sleep(0.01)

if __name__ == "__main__":
  try:
    KeyboardController()
  except rospy.ROSInterruptException:
    pass