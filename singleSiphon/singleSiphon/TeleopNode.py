import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import termios
import tty
import select

FREQ_STEP    = 0.1
MAX_FREQ     = 1.5

# Position in millidegrees  (360 000 mDeg = one full output-shaft revolution)
POS_STEP_mDEG = 10000.0   # 10 degrees per keypress
MAX_POS_mDEG  = 10*360000.0
MIN_POS_mDEG  = -10*360000.0

MODE_FREQ = 'freq'
MODE_POS  = 'pos'

best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

class ActuatorTeleop(Node):
    def __init__(self):
        super().__init__('actuator_teleop')
        # Frequency publishers
        self.pub_freq1 = self.create_publisher(Float32, 'actuator_1/freq', best_effort_qos)
        self.pub_freq2 = self.create_publisher(Float32, 'actuator_2/freq', best_effort_qos)
        # Mode publishers  (0.0 = FREQ_SWEEP, 1.0 = POSITION — matches ESP32 firmware)
        self.pub_mode1 = self.create_publisher(Float32, 'actuator_1/mode', best_effort_qos)
        self.pub_mode2 = self.create_publisher(Float32, 'actuator_2/mode', best_effort_qos)
        # Position setpoint publishers (millidegrees — matches ESP32 setpoint_callback)
        self.pub_setpoint1 = self.create_publisher(Float32, 'actuator_1/setpoint', best_effort_qos)
        self.pub_setpoint2 = self.create_publisher(Float32, 'actuator_2/setpoint', best_effort_qos)
        self.freq1 = 0.0
        self.freq2 = 0.0
        self.pos1  = 0.0   # millidegrees
        self.pos2  = 0.0
        self.mode  = MODE_FREQ
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 100ms timeout
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_help(self):
        mode_label = 'FREQUENCY' if self.mode == MODE_FREQ else 'POSITION'
        print(f"\n=== Mode: {mode_label} (SPACE to toggle) ===")
        print("  [FREQ]  q/a/z : actuator 1 freq up / stop / down")
        print("          r/f/v : actuator 2 freq up / stop / down")
        print("  [POS ]  w/s   : actuator 1 setpoint up / down  (45 deg steps)")
        print("          e/d   : actuator 2 setpoint up / down  (45 deg steps)")
        print("  Ctrl+C  : quit\n")

    def print_status(self):
        if self.mode == MODE_FREQ:
            print(f"\r[FREQ]  A1: {self.freq1:.1f} Hz  |  A2: {self.freq2:.1f} Hz    ",
                  end='', flush=True)
        else:
            print(f"\r[POS ]  A1: {self.pos1/1000:.1f} deg  |  A2: {self.pos2/1000:.1f} deg    ",
                  end='', flush=True)

    def publish_mode(self):
        mode_val = 1.0 if self.mode == MODE_POS else 0.0
        msg = Float32()
        msg.data = mode_val
        self.pub_mode1.publish(msg)
        self.pub_mode2.publish(msg)

    def run(self):
        self.print_help()

        while rclpy.ok():
            key = self.get_key()
            changed = False

            if key == '\x03':
                break

            elif key == ' ':
                self.mode = MODE_POS if self.mode == MODE_FREQ else MODE_FREQ
                if self.mode == MODE_POS:
                    self.pos1 = 0.0
                    self.pos2 = 0.0
                self.publish_mode()
                self.print_help()

            elif self.mode == MODE_FREQ:
                if   key == 'q': self.freq1 = min(self.freq1 + FREQ_STEP, MAX_FREQ); changed = True
                elif key == 'a': self.freq1 = 0.0;                                    changed = True
                elif key == 'z': self.freq1 = max(self.freq1 - FREQ_STEP, 0.0);       changed = True
                elif key == 'r': self.freq2 = min(self.freq2 + FREQ_STEP, MAX_FREQ);  changed = True
                elif key == 'f': self.freq2 = 0.0;                                    changed = True
                elif key == 'v': self.freq2 = max(self.freq2 - FREQ_STEP, 0.0);       changed = True

            elif self.mode == MODE_POS:
                if   key == 'w': self.pos1 = min(self.pos1 + POS_STEP_mDEG, MAX_POS_mDEG); changed = True
                elif key == 's': self.pos1 = max(self.pos1 - POS_STEP_mDEG, MIN_POS_mDEG); changed = True
                elif key == 'e': self.pos2 = min(self.pos2 + POS_STEP_mDEG, MAX_POS_mDEG); changed = True
                elif key == 'd': self.pos2 = max(self.pos2 - POS_STEP_mDEG, MIN_POS_mDEG); changed = True

            if changed:
                msg1, msg2 = Float32(), Float32()
                if self.mode == MODE_FREQ:
                    msg1.data, msg2.data = self.freq1, self.freq2
                    self.pub_freq1.publish(msg1)
                    self.pub_freq2.publish(msg2)
                else:
                    msg1.data, msg2.data = self.pos1, self.pos2
                    self.pub_setpoint1.publish(msg1)
                    self.pub_setpoint2.publish(msg2)
                self.print_status()

def main():
    rclpy.init()
    node = ActuatorTeleop()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
