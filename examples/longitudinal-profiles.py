#!/usr/bin/env python3
import time

from panda import Panda
from opendbc.car.structs import CarControl
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData


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

  def obd_callback(self, obd_multiplexing: bool) -> None:
    self.p.set_obd(obd_multiplexing)


def main():
  p = Panda()
  pc = PandaCan(p)

  CI = get_car(pc.can_recv, pc.can_send, pc.obd_callback, True)
  print("fingerprinted", CI.CP.carName)
  assert CI.CP.openpilotLongitudinalControl, "Longitudinal control not enabled"

  try:
    # setup
    p.set_safety_mode(Panda.SAFETY_ELM327, CI.CP.safetyConfigs[-1].safetyParam)
    CI.init(CI.CP, pc.can_recv, pc.can_send)
    p.set_safety_mode(Panda.SAFETY_TOYOTA, CI.CP.safetyConfigs[0].safetyParam)

    # 100Hz controls
    while True:
      cs = CI.update([0, pc.can_recv()[0]])

      CC = CarControl()
      _, can_sends = CI.apply(CC)
      p.can_send_many(can_sends)
      
      #print(cs.vEgo)
      #print(can_sends)

      time.sleep(0.1)
  finally:
    p.set_safety_mode(Panda.SAFETY_NOOUTPUT)


if __name__ == "__main__":
    main()
