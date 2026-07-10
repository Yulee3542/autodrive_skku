from .road import RoadMission
from .traffic import TrafficMission
from .t_parking import TParkingMission
from .test_mission import TestMission

MISSIONS = {m.name: m for m in (RoadMission, TrafficMission, TParkingMission, TestMission)}
