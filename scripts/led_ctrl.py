#!/usr/bin/env python3

import rospy
import board
import busio
import neopixel_spi as neopixel
from std_msgs.msg import String


class LEDControl:
    def __init__(self):

        # NeoPixel 설정
        self.num_of_pixels = 4           # 사용 중인 WS2812 LED 개수
        pixel_order = neopixel.GRB       # 색상 순서
        spi = board.SPI()
        self.pixels = neopixel.NeoPixel_SPI(spi, self.num_of_pixels, pixel_order=pixel_order, auto_write=True)

        rospy.Subscriber("/aims/led_color_cmd", String, self.ledCb, queue_size=1)

        rospy.loginfo("WS2812 LED Node Started. Listening to /aims/led_color_cmd topic.")

    def ledCb(self, msg):
        """수신된 색상 값을 기반으로 LED 스트립을 변경"""
        try:
            r, g, b = map(int, msg.data.split(","))
            # rospy.loginfo(f"Setting LED color to: R={r}, G={g}, B={b}")

            for i in range(self.num_of_pixels):
                self.pixels[i] = (r, g, b)

            self.pixels.show()

        except ValueError:
            rospy.logwarn("Invalid color format. Use 'R,G,B' format.")


    def shutdown_hook(self):
        """Turn off LEDs on exit"""
        rospy.loginfo("Shutting down... Turning off LEDs.")
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("led_ctrl_node", anonymous=True)
    led_ctrl = LEDControl()
    try:
        led_ctrl.run()
    
    except rospy.ROSInterruptException:
        pass

    finally:
        led_ctrl.shutdown_hook()
