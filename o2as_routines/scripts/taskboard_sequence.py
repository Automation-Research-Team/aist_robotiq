import o2as_routines.ExampleClass

class TaskboardClass(object):
  """TaskboardClass"""
  def __init__(self):
    super(ExampleClass, self).__init__()
  

def main():
  try:
    tutorial = TaskboardClass()

    # print "============ Press `Enter` to start assembly test ..."
    # raw_input()
    # tutorial.insertion_demo()
    # tutorial.pick_place_demo()
    # tutorial.simple_taskboard_demo()
    tutorial.do_calibration()

    print "============ Demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
