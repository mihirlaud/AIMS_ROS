
"use strict";

let ActuatorControl = require('./ActuatorControl.js');
let ExtendedState = require('./ExtendedState.js');
let FileEntry = require('./FileEntry.js');
let Param = require('./Param.js');
let MountControl = require('./MountControl.js');
let ManualControl = require('./ManualControl.js');
let Altitude = require('./Altitude.js');
let HilControls = require('./HilControls.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let WaypointList = require('./WaypointList.js');
let HomePosition = require('./HomePosition.js');
let RTKBaseline = require('./RTKBaseline.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let Thrust = require('./Thrust.js');
let ParamValue = require('./ParamValue.js');
let RCOut = require('./RCOut.js');
let GPSRAW = require('./GPSRAW.js');
let CommandCode = require('./CommandCode.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let PositionTarget = require('./PositionTarget.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let HilSensor = require('./HilSensor.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let HilGPS = require('./HilGPS.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let ESCStatus = require('./ESCStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let GPSINPUT = require('./GPSINPUT.js');
let Mavlink = require('./Mavlink.js');
let GPSRTK = require('./GPSRTK.js');
let BatteryStatus = require('./BatteryStatus.js');
let DebugValue = require('./DebugValue.js');
let State = require('./State.js');
let RCIn = require('./RCIn.js');
let Vibration = require('./Vibration.js');
let Waypoint = require('./Waypoint.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let RadioStatus = require('./RadioStatus.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let StatusText = require('./StatusText.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let LandingTarget = require('./LandingTarget.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let WaypointReached = require('./WaypointReached.js');
let LogData = require('./LogData.js');
let ESCInfo = require('./ESCInfo.js');
let LogEntry = require('./LogEntry.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let Trajectory = require('./Trajectory.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let VFR_HUD = require('./VFR_HUD.js');
let RTCM = require('./RTCM.js');
let VehicleInfo = require('./VehicleInfo.js');

module.exports = {
  ActuatorControl: ActuatorControl,
  ExtendedState: ExtendedState,
  FileEntry: FileEntry,
  Param: Param,
  MountControl: MountControl,
  ManualControl: ManualControl,
  Altitude: Altitude,
  HilControls: HilControls,
  WheelOdomStamped: WheelOdomStamped,
  TimesyncStatus: TimesyncStatus,
  ESCStatusItem: ESCStatusItem,
  WaypointList: WaypointList,
  HomePosition: HomePosition,
  RTKBaseline: RTKBaseline,
  PlayTuneV2: PlayTuneV2,
  Thrust: Thrust,
  ParamValue: ParamValue,
  RCOut: RCOut,
  GPSRAW: GPSRAW,
  CommandCode: CommandCode,
  HilActuatorControls: HilActuatorControls,
  PositionTarget: PositionTarget,
  OpticalFlowRad: OpticalFlowRad,
  HilSensor: HilSensor,
  OnboardComputerStatus: OnboardComputerStatus,
  EstimatorStatus: EstimatorStatus,
  HilGPS: HilGPS,
  AttitudeTarget: AttitudeTarget,
  ESCStatus: ESCStatus,
  ESCTelemetry: ESCTelemetry,
  GPSINPUT: GPSINPUT,
  Mavlink: Mavlink,
  GPSRTK: GPSRTK,
  BatteryStatus: BatteryStatus,
  DebugValue: DebugValue,
  State: State,
  RCIn: RCIn,
  Vibration: Vibration,
  Waypoint: Waypoint,
  MagnetometerReporter: MagnetometerReporter,
  RadioStatus: RadioStatus,
  ADSBVehicle: ADSBVehicle,
  ESCTelemetryItem: ESCTelemetryItem,
  StatusText: StatusText,
  CompanionProcessStatus: CompanionProcessStatus,
  LandingTarget: LandingTarget,
  HilStateQuaternion: HilStateQuaternion,
  CamIMUStamp: CamIMUStamp,
  OverrideRCIn: OverrideRCIn,
  WaypointReached: WaypointReached,
  LogData: LogData,
  ESCInfo: ESCInfo,
  LogEntry: LogEntry,
  NavControllerOutput: NavControllerOutput,
  ESCInfoItem: ESCInfoItem,
  Trajectory: Trajectory,
  GlobalPositionTarget: GlobalPositionTarget,
  VFR_HUD: VFR_HUD,
  RTCM: RTCM,
  VehicleInfo: VehicleInfo,
};
