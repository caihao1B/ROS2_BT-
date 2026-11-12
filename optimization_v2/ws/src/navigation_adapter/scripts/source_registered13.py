"""Reuse S08 source conversion; change ONLY raw map output topic before registration."""
from common13 import *
sys.path.insert(0,str(BASE/'ws/src/gnss_supervisor/scripts'))
import source_adapter
original=source_adapter.rclpy.init
source_adapter.rclpy.init=lambda:original(args=['--ros-args','-r','/s08/map:=/s13/unregistered_map'])
source_adapter.main()
