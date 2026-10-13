#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

class FlightCarrier(Node):
	def __init__(self):
		super().__init__('flight_carrier')

		self.cruise_speed = 3.0  # m/s forward travel
		self.x = 0.0
		self.z = 0.85            # Target cruise altitude AGL
		self.touchdown = False
		self.climbing = False
		self.armed = False

		self.create_subscription(Bool, '/flight/touchdown', self.touchdown_cb, 10)
		self.create_subscription(Float64, '/control/pitch_setpoint', self.pitch_sp_cb, 10)

		self.pub_x = self.create_publisher(Float64, '/flight/x_cmd', 10)
		self.pub_z = self.create_publisher(Float64, '/flight/z_cmd', 10)
		self.pub_prog = self.create_publisher(Float64, '/flight/progress_x', 10)

		self.sim_start_time = None
		self.timer = self.create_timer(0.02, self.step_flight) # 50 Hz
		self.get_logger().info("Flight Carrier Node active. Glideslope trajectory online.")

	def touchdown_cb(self, msg: Bool):
		self.touchdown = msg.data

	def pitch_sp_cb(self, msg: Float64):
		self.climbing = (msg.data > 0.10)

	def step_flight(self):
		now_sec = self.get_clock().now().nanoseconds / 1e9

		if self.sim_start_time is None:
			if now_sec > 0.1:
				self.sim_start_time = now_sec
			return

		elapsed_sim = now_sec - self.sim_start_time

		# Hold at spawn position while hoisting up to cruise altitude (0.85m)
		if elapsed_sim < 4.0:
			self.pub_x.publish(Float64(data=0.0))
			self.pub_z.publish(Float64(data=0.85))
			self.pub_prog.publish(Float64(data=0.0))
			return

		if not self.armed:
			self.armed = True
			self.get_logger().info("SIM TIME ARMED: Glideslope trajectory engaged at 3.0 m/s.")

		dt = 0.02

		if not self.touchdown:
			# 1. Forward progression along corridor (+X)
			self.x += self.cruise_speed * dt

			# 2. Continuous approach glideslope:
			# Cruise level at 0.85m over Sector 1 vegetation (X < 25m)
			# Descend smoothly from 0.85m to 0.16m between X = 25m and X = 85m
			if self.climbing:
				self.z = min(3.0, self.z + 0.60 * dt)
			elif self.x < 25.0:
				self.z = 0.85
			else:
				target_z = max(0.16, 0.85 - ((self.x - 25.0) * 0.0115))
				self.z = target_z

		self.pub_x.publish(Float64(data=self.x))
		self.pub_z.publish(Float64(data=self.z))
		self.pub_prog.publish(Float64(data=self.x))

def main():
	rclpy.init()
	node = FlightCarrier()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()