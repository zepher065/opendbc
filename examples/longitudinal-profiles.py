#!/usr/bin/env python3
import io
import time
import base64
import matplotlib.pyplot as plt
from enum import Enum
from collections import defaultdict
from dataclasses import dataclass, asdict

from panda import Panda
from opendbc.car.structs import CarControl, CarState
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData

DT = 0.01  # 100Hz step time

class PandaCan:
  def __init__(self, p: Panda) -> None:
    self.p = p

  def can_recv(self, wait_for_one: bool = False) -> list[list[CanData]]:
    recv = self.p.can_recv()
    while len(recv) == 0 and wait_for_one:
      recv = self.p.can_recv()
    return [[CanData(addr, dat, bus) for addr, dat, bus in recv], ]

  def can_send(self, msgs: list[CanData]) -> None:
    self.p.can_send_many(msgs)


class Setup(Enum):
  STOPPED = 0
  STEADY_STATE_SPEED = 1

@dataclass
class Maneuver:
  description: str
  setup: Setup         # initial state

  _log = defaultdict(list)

  def get_cc(self, t: float) -> CarControl:
    CC = CarControl(
      enabled=True,
      latActive=False,
      longActive=True,

      actuators=CarControl.Actuators(
        gas=0.,
        brake=0.,
        speed=0.,
        accel=0.,
        longControlState=CarControl.Actuators.LongControlState.off,
      ),

      cruiseControl=CarControl.CruiseControl(
        cancel=False,
        resume=False,
        override=False,
      ),
    )
    return CC

  def get_msgs(self):
    for t in range(0, int(1./DT)):
      yield t, self.get_cc(t)

  def log(self, t, cc: CarControl, cs: CarState) -> None:
    self._log["t"].append(t)
    to_log = {"carControl": cc, "carState": cs, "carControl.actuators": cc.actuators, "carControl.cruiseControl": cc.cruiseControl, "carState.cruiseState": cs.cruiseState}
    for k, v in to_log.items():
      for k2, v2 in asdict(v).items():
        self._log[f"{k}.{k2}"].append(v2)

MANEUVERS = [
  Maneuver(
    "start from stop",
    Setup.STOPPED,
  ),
]

def main():
  p = Panda()
  pc = PandaCan(p)
  try:
    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    CI = get_car(pc.can_recv, pc.can_send, p.set_obd, True)
    print("fingerprinted", CI.CP.carName)
    assert CI.CP.openpilotLongitudinalControl, "Longitudinal control not enabled"

    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    CI.init(CI.CP, pc.can_recv, pc.can_send)
    p.set_safety_mode(Panda.SAFETY_TOYOTA, CI.CP.safetyConfigs[0].safetyParam)

    m = MANEUVERS[0]
    for m in MANEUVERS:
      print(f"Running '{m.description}'")

      # cleanup and get into a good state
      cs = None
      for _ in range(int(3./DT)):
        cs = CI.update([0, pc.can_recv()[0]])
        _, can_sends = CI.apply(CarControl(enabled=False))
        p.can_send_many(can_sends, timeout=1000)
        time.sleep(DT)
      #assert not cs.cruiseState.enabled, "Cruise control not disabled"

      # run the maneuver
      for t, msg in m.get_msgs():
        cs = CI.update([0, pc.can_recv()[0]])
        #assert cs.canValid, f"CAN went invalid, check connections"

        _, can_sends = CI.apply(msg)
        #p.can_send_many(can_sends, timeout=20)

        m.log(t, msg, cs)
        time.sleep(DT)

        if len(m._log["t"]) > 100:
          break
  finally:
    p.set_safety_mode(Panda.SAFETY_NOOUTPUT)

  def plt2html():
    plt.legend()
    plt.tight_layout(pad=0)

    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)
    return f"<img src='data:image/png;base64,{base64.b64encode(buffer.getvalue()).decode()}' style='width:100%; max-width:800px;'>\n"

  # write out report
  with open("report.html", "w") as f:
    f.write(f"<h1>Longitudinal maneuver report</h1>\n")
    f.write(f"<h3>{CI.CP.carFingerprint}</h3>\n")
    for m in MANEUVERS:
      f.write(f"<div style='border-top: 1px solid #000; margin: 20px 0;'></div>\n")
      f.write(f"<h2>{m.description}</h2>\n")

      # accel plot
      plt.figure(figsize=(12, 4))
      plt.plot(m._log["t"], m._log["carState.aEgo"], label='aEgo')
      plt.plot(m._log["t"], m._log["carControl.actuators.accel"], label='actuators.accel')
      plt.xlabel('Time (s)')
      plt.ylabel('Acceleration (m/s^2)')
      plt.ylim(-2.2, 2.2)
      plt.title('Acceleration Profile')
      plt.grid(True)
      f.write(plt2html())
      plt.close()

      # binary plots
      for k in ("carControl.enabled", "carState.cruiseState.enabled"):
        plt.rcParams['lines.linewidth'] = 2
        plt.figure(figsize=(12, 1))
        plt.plot(m._log["t"], m._log[k], label=k)
        plt.ylim(0.1, 1.1)
        # plt.grid(False)
        # plt.axis('off')
        plt.ylabel('  ')   # for alignment
        f.write(plt2html())
        plt.close()


if __name__ == "__main__":
    main()