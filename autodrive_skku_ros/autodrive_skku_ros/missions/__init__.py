from .road import RoadMission
from .traffic import TrafficMission
from .t_parking import TParkingMission

MISSIONS = {m.name: m for m in (RoadMission, TrafficMission, TParkingMission)}
