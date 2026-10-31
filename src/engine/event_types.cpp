#include "event_types.h"

#include "audio/audio_engine.h"
#include "control/city_requests.h"
#include "systems/camera_system.h"
#include "systems/day_night_system.h"
#include "systems/player_system.h"
#include "systems/teleport_pose.h"
#include "systems/vehicle_system.h"
#include "systems/interact_broker_system.h"
#include "systems/elevator_system.h"
#include "systems/render_system.h"

namespace engine {

void describeProperties(PlaySound& e, PropertyVisitor& v) {
    v.field(FieldMeta("clip"), e.clip);
    v.field(FieldMeta("position").units("m"), e.position);
    v.field(FieldMeta("spatial"), e.spatial);
    v.field(FieldMeta("volume").range(0, 4), e.volume);
    v.field(FieldMeta("pitch").range(0.1, 4), e.pitch);
    v.field(FieldMeta("range").units("m"), e.range);
}

void describeEngineEvents(EventBus& bus) {
    describeEvent<PlaySound>(bus, "PlaySound",
                             "Play a sound cue: a clip path or registered name, flat or at a position.");
    describeEvent<TeleportRequest>(bus, "TeleportRequest",
                                   "Move the player (the fly camera when there is none); ground = stand at x z.");
    describeEvent<CameraPoseRequest>(bus, "CameraPoseRequest",
                                     "Put the free camera at a pose and detach it from the player.");
    describeEvent<CameraDetachRequest>(bus, "CameraDetachRequest",
                                       "The F key: detach (1), re-attach (0, the player comes to the camera), toggle (2).");
    describeEvent<DayNightRequest>(bus, "DayNightRequest",
                                   "Drive the day/night cycle: hour, hold, loop minutes, calendar day, moon, terrain shadows, sky HUD, chart.");
    describeEvent<WeatherRequest>(bus, "WeatherRequest", "Set the weather: clear, fair, overcast, storm, auto or off; now snaps it.");
    describeEvent<PossessRequest>(bus, "PossessRequest",
                                  "A possession command line: car|walker [x z], driveto|walkto x z, direct on|off, board, enter, exit, alight, ride x z, stop, release.");
    describeEvent<LimpRequest>(bus, "LimpRequest", "The player goes limp (G), or is thrown limp at a velocity.");
    describeEvent<VehicleSpawnRequest>(bus, "VehicleSpawnRequest", "Drop a vehicle in front of the player (N); recipe picks one.");
    describeEvent<InteractRequest>(bus, "InteractRequest", "The interact key: 1 tap, 2 hold, 3 stand up, 4 next focus (Tab).");
    describeEvent<ElevatorRequest>(bus, "ElevatorRequest", "The lift: call the cab to this door, and/or move the floor selection by pick.");
    describeEvent<RideRequest>(bus, "RideRequest", "The player boards the nearest bus (1), cab (2) or either (3), or gets off (0).");
    describeEvent<TrainBoardRequest>(bus, "TrainBoardRequest", "Put the player aboard a train, standing mid-carriage.");
    describeEvent<KnockRequest>(bus, "KnockRequest", "Throw the walker nearest the player down, as a car would.");
    describeEvent<PinRequest>(bus, "PinRequest", "Follow an agent in the full sim however far it goes (on), or stop.");
    describeEvent<FindPlaceRequest>(bus, "FindPlaceRequest", "Find the places whose name contains this; findplace? answers.");
    describeEvent<CityLayerRequest>(bus, "CityLayerRequest", "Show or hide a city sim debug layer: master, agents, cones, nav or plan.");
    describeEvent<JitterRequest>(bus, "JitterRequest", "Record how evenly a vehicle is drawn for some frames; jitter? answers.");
    describeEvent<CloudsRequest>(bus, "CloudsRequest", "Set the cloud deck live: enabled, coverage, density, scale, wind (below zero: unchanged).");
    describeEvent<LampLightsRequest>(bus, "LampLightsRequest", "The street lamps' point lights on or off (their heads stay lit).");
    describeEvent<TeleportPanelRequest>(bus, "TeleportPanelRequest", "Unfold the Debug window's Teleport section once.");
    describeEvent<KeyTapRequest>(bus, "KeyTapRequest", "Press and release a key through the real input path (e.g. F).");
    describeEvent<KeyHoldRequest>(bus, "KeyHoldRequest", "Hold a key (or MouseLeft) down for seconds of sim time.");
    describeEvent<PersonRequest>(bus, "PersonRequest", "The V key: first (0) or third (1) person on foot, or toggle (2).");
}

}  // namespace engine
