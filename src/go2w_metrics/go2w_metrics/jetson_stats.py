"""tegrastats -> /jetson/stats (diagnostic_msgs/DiagnosticArray), on the NX.

ros2_vitals reads the GPU with NVML, which does not exist on Jetson: GPU load, memory
controller (EMC) load, temperatures of every thermal zone and power rails come from
tegrastats. One DiagnosticStatus per line, name 'jetson', hardware_id = hostname, values:
  ram_used_mb ram_total_mb swap_used_mb swap_total_mb
  cpu<i>_percent cpu<i>_mhz (cores 'off' are left out) cpu_percent (mean of the active)
  emc_percent gpu_percent gpu_mhz
  temp_<zone>_c      (cpu, gpu, soc0, tj... names depend on the module)
  power_<rail>_mw    (VDD_IN = whole module, VDD_CPU_GPU_CV, VDD_SOC...)

  ros2 run go2w_metrics jetson_stats --ros-args -p interval_ms:=500
"""

import re
import shutil
import socket
import subprocess
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

RAM = re.compile(r'\bRAM (\d+)/(\d+)MB')
SWAP = re.compile(r'\bSWAP (\d+)/(\d+)MB')
CPU = re.compile(r'\bCPU \[([^\]]*)\]')
CORE = re.compile(r'(\d+)%@(\d+)')
EMC = re.compile(r'\bEMC_FREQ (\d+)%')
GPU = re.compile(r'\bGR3D_FREQ (\d+)%(?:@\[?(\d+))?')
TEMP = re.compile(r'\b([A-Za-z][\w]*)@(-?\d+(?:\.\d+)?)C\b')
# JetPack 5/6: "VDD_IN 4762mW/4762mW"; JetPack 4: "VDD_IN 3721/3721" or "POM_5V_IN 3721/3721"
POWER = re.compile(r'\b((?:VDD|POM|VIN)_\w+) (\d+)(?:mW)?/(\d+)(?:mW)?')


def parse(line):
    """Values of one tegrastats line as {name: float}."""
    v = {}
    if m := RAM.search(line):
        v['ram_used_mb'], v['ram_total_mb'] = float(m[1]), float(m[2])
    if m := SWAP.search(line):
        v['swap_used_mb'], v['swap_total_mb'] = float(m[1]), float(m[2])
    if m := CPU.search(line):
        active = []
        for i, core in enumerate(m[1].split(',')):
            if c := CORE.search(core):
                v[f'cpu{i}_percent'], v[f'cpu{i}_mhz'] = float(c[1]), float(c[2])
                active.append(float(c[1]))
        if active:
            v['cpu_percent'] = sum(active) / len(active)
    if m := EMC.search(line):
        v['emc_percent'] = float(m[1])
    if m := GPU.search(line):
        v['gpu_percent'] = float(m[1])
        if m[2]:
            v['gpu_mhz'] = float(m[2])
    for name, value in TEMP.findall(line):
        v[f'temp_{name.lower()}_c'] = float(value)
    for rail, now, _avg in POWER.findall(line):
        v[f'power_{rail.lower()}_mw'] = float(now)
    return v


class JetsonStats(Node):
    def __init__(self):
        super().__init__('jetson_stats')
        interval = int(self.declare_parameter('interval_ms', 500).value)
        topic = self.declare_parameter('topic', '/jetson/stats').value
        self.hostname = socket.gethostname()
        self.pub = self.create_publisher(DiagnosticArray, topic, 10)
        exe = shutil.which('tegrastats') or '/usr/bin/tegrastats'
        try:
            self.proc = subprocess.Popen([exe, '--interval', str(interval)],
                                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                         text=True, bufsize=1)
        except OSError as e:
            raise SystemExit(f'Cannot run tegrastats ({e}): this node is for the Jetson (NX)')
        self.get_logger().info(f'tegrastats every {interval} ms -> {topic}')
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            values = parse(line)
            if not values:
                continue
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status.append(DiagnosticStatus(
                level=DiagnosticStatus.OK, name='jetson', hardware_id=self.hostname,
                values=[KeyValue(key=k, value=f'{x:g}') for k, x in values.items()]))
            self.pub.publish(msg)
        self.get_logger().error('tegrastats stopped')

    def destroy_node(self):
        self.proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = JetsonStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
