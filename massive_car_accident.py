import py_trees
import carla
import math
import time
import os
import datetime

from agents.navigation.local_planner import RoadOption
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (generate_target_waypoint,
                                           get_location_in_distance_from_wp,
                                           get_waypoint_in_distance,
                                           generate_target_waypoint_list,
                                           choose_at_junction)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (AtomicBehavior,
                                                                      StopVehicle,
                                                                      AccelerateToVelocity,
                                                                      ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      WaypointFollower,
                                                                      KeepVelocity,
                                                                      ChangeAutoPilot,
                                                                      SetInitSpeed,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      get_actor_control,
                                                                      calculate_distance)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               TriggerVelocity,
                                                                               StandStill)


class MassiveCarAccident(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=120):
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._accident_waypoint = None
        self.debug = world.debug
        self.timeout = timeout

        super(MassiveCarAccident, self).__init__("MassiveCarAccident",
                                                 ego_vehicles,
                                                 config,
                                                 world,
                                                 debug_mode,
                                                 criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        self._accident_waypoint = self._reference_waypoint.next(93)[0]

        actor_vehicle_0_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 30,
                           self._accident_waypoint.transform.location.y,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_vehicle_0_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 30,
                           self._accident_waypoint.transform.location.y,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_vehicle_1_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 30,
                           self._accident_waypoint.transform.location.y + 4,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_vehicle_1_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 30,
                           self._accident_waypoint.transform.location.y + 4,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_vehicle_2_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 37.5,
                           self._accident_waypoint.transform.location.y,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_vehicle_2_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 37.5,
                           self._accident_waypoint.transform.location.y,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_vehicle_3_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 37.5,
                           self._accident_waypoint.transform.location.y + 4,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_vehicle_3_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 37.5,
                           self._accident_waypoint.transform.location.y + 4,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        yaw_1 = self._reference_waypoint.transform.rotation.yaw + 90

        actor_vehicle_4_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x,
                           self._accident_waypoint.transform.location.y + 30,
                           self._accident_waypoint.transform.location.z - 500),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))
        self._actor_vehicle_4_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x,
                           self._accident_waypoint.transform.location.y + 30,
                           self._accident_waypoint.transform.location.z),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))

        actor_vehicle_5_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 4,
                           self._accident_waypoint.transform.location.y + 30,
                           self._accident_waypoint.transform.location.z - 500),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))
        self._actor_vehicle_5_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 4,
                           self._accident_waypoint.transform.location.y + 30,
                           self._accident_waypoint.transform.location.z),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))

        actor_vehicle_6_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x,
                           self._accident_waypoint.transform.location.y + 37.5,
                           self._accident_waypoint.transform.location.z - 500),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))
        self._actor_vehicle_6_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x,
                           self._accident_waypoint.transform.location.y + 37.5,
                           self._accident_waypoint.transform.location.z),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))

        actor_vehicle_7_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 4,
                           self._accident_waypoint.transform.location.y + 37.5,
                           self._accident_waypoint.transform.location.z - 500),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))
        self._actor_vehicle_7_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 4,
                           self._accident_waypoint.transform.location.y + 37.5,
                           self._accident_waypoint.transform.location.z),
            carla.Rotation(self._accident_waypoint.transform.rotation.pitch, yaw_1,
                           self._accident_waypoint.transform.rotation.roll))

        actor_police_8_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 20.25,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_police_8_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 20.25,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_police_9_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 23,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_police_9_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 23,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_police_10_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 25.75,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_police_10_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 25.75,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_ambulance_11_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 28.5,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_ambulance_11_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 28.5,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_ambulance_12_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 31.25,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_ambulance_12_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 31.25,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_firetruck_13_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 39.75,
                           self._accident_waypoint.transform.location.z - 500),
            self._accident_waypoint.transform.rotation)
        self._actor_firetruck_13_transform = carla.Transform(
            carla.Location(self._accident_waypoint.transform.location.x + 96,
                           self._accident_waypoint.transform.location.y - 39.75,
                           self._accident_waypoint.transform.location.z),
            self._accident_waypoint.transform.rotation)

        actor_vehicle_0 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_0_transform,
                                                              color='255,  0,  0')
        actor_vehicle_1 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_1_transform,
                                                              color='128,128,128')
        actor_vehicle_2 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_2_transform,
                                                              color='128,128,128')
        actor_vehicle_3 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_3_transform,
                                                              color='128,128,128')
        actor_vehicle_4 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_4_transform,
                                                              color='255,  0,  0')
        actor_vehicle_5 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_5_transform,
                                                              color='128,128,128')
        actor_vehicle_6 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_6_transform,
                                                              color='128,128,128')
        actor_vehicle_7 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                              actor_vehicle_7_transform,
                                                              color='128,128,128')
        actor_police_8 = CarlaDataProvider.request_new_actor('vehicle.dodge_charger.police',
                                                             actor_police_8_transform)
        actor_police_9 = CarlaDataProvider.request_new_actor('vehicle.dodge_charger.police',
                                                             actor_police_9_transform)
        actor_police_10 = CarlaDataProvider.request_new_actor('vehicle.dodge_charger.police',
                                                              actor_police_10_transform)
        actor_ambulance_11 = CarlaDataProvider.request_new_actor('vehicle.volkswagen.t2',
                                                                 actor_ambulance_11_transform,
                                                                 color='255,0,0')
        actor_ambulance_12 = CarlaDataProvider.request_new_actor('vehicle.volkswagen.t2',
                                                                 actor_ambulance_12_transform,
                                                                 color='255,0,0')
        actor_firetruck_13 = CarlaDataProvider.request_new_actor('vehicle.carlamotors.carlacola',
                                                                 actor_firetruck_13_transform,
                                                                 color='255,0,0')

        self.other_actors.append(actor_vehicle_0)
        self.other_actors.append(actor_vehicle_1)
        self.other_actors.append(actor_vehicle_2)
        self.other_actors.append(actor_vehicle_3)
        self.other_actors.append(actor_vehicle_4)
        self.other_actors.append(actor_vehicle_5)
        self.other_actors.append(actor_vehicle_6)
        self.other_actors.append(actor_vehicle_7)
        self.other_actors.append(actor_police_8)
        self.other_actors.append(actor_police_9)
        self.other_actors.append(actor_police_10)
        self.other_actors.append(actor_ambulance_11)
        self.other_actors.append(actor_ambulance_12)
        self.other_actors.append(actor_firetruck_13)

        self.debug.draw_point(self._accident_waypoint.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_0_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_1_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_2_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_3_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_4_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_5_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_6_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_vehicle_7_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_police_8_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_police_9_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_police_10_transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(self._actor_ambulance_11_transform.location + carla.Location(z=0.5), size=0.5,
        #                       life_time=0)
        # self.debug.draw_point(self._actor_ambulance_12_transform.location + carla.Location(z=0.5), size=0.5,
        #                       life_time=0)
        # self.debug.draw_point(self._actor_firetruck_13_transform.location + carla.Location(z=0.5), size=0.5,
        #                       life_time=0)

    def _create_behavior(self):
        # Sequence 1
        sequence_1_actor_vehicle_0 = py_trees.composites.Sequence("Sequence 1 – other_actors[0]")
        sequence_1_actor_vehicle_0.add_child(
            ActorTransformSetter(self.other_actors[0], self._actor_vehicle_0_transform))
        sequence_1_actor_vehicle_0.add_child(SetInitSpeed(self.other_actors[0], 10))
        sequence_1_actor_vehicle_0.add_child(
            InTriggerDistanceToLocation(self.other_actors[0], self._accident_waypoint.transform.location, 5.0))
        sequence_1_actor_vehicle_0.add_child(StopVehicle(self.other_actors[0], 1.0))
        sequence_1_actor_vehicle_0.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_1 = py_trees.composites.Sequence("Sequence 1 – other_actors[1]")
        sequence_1_actor_vehicle_1.add_child(
            ActorTransformSetter(self.other_actors[1], self._actor_vehicle_1_transform))
        sequence_1_actor_vehicle_1.add_child(TimeOut(0.5))
        sequence_1_actor_vehicle_1.add_child(SetInitSpeed(self.other_actors[1], 10))
        sequence_1_actor_vehicle_1.add_child(
            InTriggerDistanceToLocation(self.other_actors[1], self._accident_waypoint.transform.location, 10.0))
        sequence_1_actor_vehicle_1.add_child(StopVehicle(self.other_actors[1], 1.0))
        sequence_1_actor_vehicle_1.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_2 = py_trees.composites.Sequence("Sequence 1 – other_actors[2]")
        sequence_1_actor_vehicle_2.add_child(
            ActorTransformSetter(self.other_actors[2], self._actor_vehicle_2_transform))
        sequence_1_actor_vehicle_2.add_child(TimeOut(1.5))
        sequence_1_actor_vehicle_2.add_child(SetInitSpeed(self.other_actors[2], 10))
        sequence_1_actor_vehicle_2.add_child(
            InTriggerDistanceToLocation(self.other_actors[2], self._accident_waypoint.transform.location, 10.0))
        sequence_1_actor_vehicle_2.add_child(StopVehicle(self.other_actors[2], 1.0))
        sequence_1_actor_vehicle_2.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_3 = py_trees.composites.Sequence("Sequence 1 – other_actors[3]")
        sequence_1_actor_vehicle_3.add_child(
            ActorTransformSetter(self.other_actors[3], self._actor_vehicle_3_transform))
        sequence_1_actor_vehicle_3.add_child(TimeOut(2))
        sequence_1_actor_vehicle_3.add_child(SetInitSpeed(self.other_actors[3], 10))
        sequence_1_actor_vehicle_3.add_child(
            InTriggerDistanceToLocation(self.other_actors[3], self._accident_waypoint.transform.location, 10.0))
        sequence_1_actor_vehicle_3.add_child(StopVehicle(self.other_actors[3], 1.0))
        sequence_1_actor_vehicle_3.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_4 = py_trees.composites.Sequence("Sequence 1 – other_actors[4]")
        sequence_1_actor_vehicle_4.add_child(
            ActorTransformSetter(self.other_actors[4], self._actor_vehicle_4_transform))
        sequence_1_actor_vehicle_4.add_child(SetInitSpeed(self.other_actors[4], 10))
        sequence_1_actor_vehicle_4.add_child(
            InTriggerDistanceToLocation(self.other_actors[4], self._accident_waypoint.transform.location, 5.0))
        sequence_1_actor_vehicle_4.add_child(StopVehicle(self.other_actors[4], 1.0))
        sequence_1_actor_vehicle_4.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_5 = py_trees.composites.Sequence("Sequence 1 – other_actors[5]")
        sequence_1_actor_vehicle_5.add_child(
            ActorTransformSetter(self.other_actors[5], self._actor_vehicle_5_transform))
        sequence_1_actor_vehicle_5.add_child(TimeOut(2))
        sequence_1_actor_vehicle_5.add_child(SetInitSpeed(self.other_actors[5], 10))
        sequence_1_actor_vehicle_5.add_child(
            InTriggerDistanceToLocation(self.other_actors[5], self._accident_waypoint.transform.location, 10.0))
        sequence_1_actor_vehicle_5.add_child(StopVehicle(self.other_actors[5], 1.0))
        sequence_1_actor_vehicle_5.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_6 = py_trees.composites.Sequence("Sequence 1 – other_actors[6]")
        sequence_1_actor_vehicle_6.add_child(
            ActorTransformSetter(self.other_actors[6], self._actor_vehicle_6_transform))
        sequence_1_actor_vehicle_6.add_child(TimeOut(2))
        sequence_1_actor_vehicle_6.add_child(SetInitSpeed(self.other_actors[6], 10))
        sequence_1_actor_vehicle_6.add_child(
            InTriggerDistanceToLocation(self.other_actors[6], self._accident_waypoint.transform.location, 13.0))
        sequence_1_actor_vehicle_6.add_child(StopVehicle(self.other_actors[6], 1.0))
        sequence_1_actor_vehicle_6.add_child(TimeOut(15))  # For observing

        sequence_1_actor_vehicle_7 = py_trees.composites.Sequence("Sequence 1 – other_actors[7]")
        sequence_1_actor_vehicle_7.add_child(
            ActorTransformSetter(self.other_actors[7], self._actor_vehicle_7_transform))
        sequence_1_actor_vehicle_7.add_child(TimeOut(3))
        sequence_1_actor_vehicle_7.add_child(SetInitSpeed(self.other_actors[7], 10))
        sequence_1_actor_vehicle_7.add_child(
            InTriggerDistanceToLocation(self.other_actors[7], self._accident_waypoint.transform.location, 15.0))
        sequence_1_actor_vehicle_7.add_child(StopVehicle(self.other_actors[7], 1.0))
        sequence_1_actor_vehicle_7.add_child(TimeOut(15))  # For observing

        actors_emergency_route_wp_1 = CarlaDataProvider.get_map().get_waypoint(self.other_actors[9].get_location(), 15)
        actors_emergency_route_wp_1_2 = actors_emergency_route_wp_1.get_left_lane()
        actors_emergency_route_wp_2 = generate_target_waypoint(actors_emergency_route_wp_1_2, -1)
        actors_emergency_route_wp_3 = generate_target_waypoint(actors_emergency_route_wp_2, -1)
        actors_emergency_route_wp_4_v1 = actors_emergency_route_wp_3.next(50)[0]
        actors_emergency_route_wp_4_v3 = actors_emergency_route_wp_3.next(70)[0]
        actors_emergency_route_wp_4_v2 = actors_emergency_route_wp_3.next(60)[0]

        flag = actors_emergency_route_wp_4_v2.lane_id
        # move to the right lane of the right intersection
        while True:
            if flag * actors_emergency_route_wp_4_v2.lane_id > 0:
                wp_next = actors_emergency_route_wp_4_v2.get_left_lane()
            else:
                break

            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder or wp_next.lane_type == carla.LaneType.Parking:
                break
            else:
                actors_emergency_route_wp_4_v2 = wp_next
        actors_emergency_route_wp_4_v2_f = actors_emergency_route_wp_4_v2.get_right_lane()

        self.debug.draw_point(actors_emergency_route_wp_1.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        self.debug.draw_point(actors_emergency_route_wp_2.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        self.debug.draw_point(actors_emergency_route_wp_3.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(actors_emergency_route_wp_4_v1.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        self.debug.draw_point(actors_emergency_route_wp_4_v2_f.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)
        # self.debug.draw_point(actors_emergency_route_wp_4_v3.transform.location + carla.Location(z=0.5), size=0.5, life_time=0)

        sequence_2_actor_police_8 = py_trees.composites.Sequence("Sequence 1 – other_actors[8]")
        sequence_2_actor_police_8.add_child(
            ActorTransformSetter(self.other_actors[8], self._actor_police_8_transform))

        actor_police_9_plan_1 = []
        for wp in generate_target_waypoint_list(actors_emergency_route_wp_1_2, turn=1)[0]:
            actor_police_9_plan_1.append(wp)
        for wp in generate_target_waypoint_list(actors_emergency_route_wp_2, turn=-1)[0]:
            actor_police_9_plan_1.append(wp)
        for wp in generate_target_waypoint_list(actors_emergency_route_wp_3, turn=1)[0]:
            actor_police_9_plan_1.append(wp)
        actor_police_9_plan_2 = []
        for wp in generate_target_waypoint_list(actors_emergency_route_wp_4_v2_f, turn=-1)[0]:
            actor_police_9_plan_2.append(wp)

        sequence_2_actor_police_9 = py_trees.composites.Sequence("Sequence 1 – other_actors[9]")
        sequence_2_actor_police_9.add_child(
            ActorTransformSetter(self.other_actors[9], self._actor_police_9_transform))
        sequence_2_actor_police_9.add_child(TimeOut(4))
        sequence_2_actor_police_9.add_child(
            WaypointFollower(self.other_actors[9], target_speed=10.0, plan=actor_police_9_plan_1))
        sequence_2_actor_police_9.add_child(
            WaypointFollower(self.other_actors[9], target_speed=0.0, plan=actor_police_9_plan_2))
        # sequence_2_actor_police_9.add_child(
        #     InTriggerDistanceToLocation(self.other_actors[9], self._accident_waypoint.transform.location, 15.0))
        # sequence_2_actor_police_9.add_child(StopVehicle(self.other_actors[9], 1.0))
        sequence_2_actor_police_9.add_child(TimeOut(20))  # For observing

        sequence_2_actor_police_10 = py_trees.composites.Sequence("Sequence 1 – other_actors[10]")
        sequence_2_actor_police_10.add_child(
            ActorTransformSetter(self.other_actors[10], self._actor_police_10_transform))

        sequence_2_actor_ambulance_11 = py_trees.composites.Sequence("Sequence 1 – other_actors[11]")
        sequence_2_actor_ambulance_11.add_child(
            ActorTransformSetter(self.other_actors[11], self._actor_ambulance_11_transform))

        sequence_2_actor_ambulance_12 = py_trees.composites.Sequence("Sequence 1 – other_actors[12]")
        sequence_2_actor_ambulance_12.add_child(
            ActorTransformSetter(self.other_actors[12], self._actor_ambulance_12_transform))

        sequence_2_actor_firetruck_13 = py_trees.composites.Sequence("Sequence 1 – other_actors[13]")
        sequence_2_actor_firetruck_13.add_child(
            ActorTransformSetter(self.other_actors[13], self._actor_firetruck_13_transform))

        sequence_1 = py_trees.composites.Parallel("Sequence 1", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence_1.add_child(sequence_1_actor_vehicle_0)
        sequence_1.add_child(sequence_1_actor_vehicle_1)
        sequence_1.add_child(sequence_1_actor_vehicle_2)
        sequence_1.add_child(sequence_1_actor_vehicle_3)
        sequence_1.add_child(sequence_1_actor_vehicle_4)
        sequence_1.add_child(sequence_1_actor_vehicle_5)
        sequence_1.add_child(sequence_1_actor_vehicle_6)
        sequence_1.add_child(sequence_1_actor_vehicle_7)

        sequence_2 = py_trees.composites.Parallel("Sequence 2", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        sequence_2.add_child(sequence_2_actor_police_8)
        sequence_2.add_child(sequence_2_actor_police_9)
        sequence_2.add_child(sequence_2_actor_police_10)
        sequence_2.add_child(sequence_2_actor_ambulance_11)
        sequence_2.add_child(sequence_2_actor_ambulance_12)
        sequence_2.add_child(sequence_2_actor_firetruck_13)

        main_sequence = py_trees.composites.Sequence("Main Sequence")
        main_sequence.add_child(sequence_1)
        # main_sequence.add_child(StandStill(self.other_actors[7], name="Vehicle_7_StandStill", duration=3.0))
        main_sequence.add_child(sequence_2)

        return main_sequence

    def _create_test_criteria(self):
        pass

    def __del__(self):
        pass
