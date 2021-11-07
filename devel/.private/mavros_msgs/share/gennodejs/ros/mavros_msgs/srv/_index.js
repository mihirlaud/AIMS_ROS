
"use strict";

let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandHome = require('./CommandHome.js')
let FileList = require('./FileList.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let MountConfigure = require('./MountConfigure.js')
let CommandBool = require('./CommandBool.js')
let FileMakeDir = require('./FileMakeDir.js')
let MessageInterval = require('./MessageInterval.js')
let FileRemove = require('./FileRemove.js')
let LogRequestData = require('./LogRequestData.js')
let ParamSet = require('./ParamSet.js')
let ParamGet = require('./ParamGet.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let ParamPull = require('./ParamPull.js')
let FileRename = require('./FileRename.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let ParamPush = require('./ParamPush.js')
let CommandInt = require('./CommandInt.js')
let LogRequestList = require('./LogRequestList.js')
let FileChecksum = require('./FileChecksum.js')
let WaypointClear = require('./WaypointClear.js')
let StreamRate = require('./StreamRate.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandTOL = require('./CommandTOL.js')
let WaypointPull = require('./WaypointPull.js')
let CommandAck = require('./CommandAck.js')
let FileWrite = require('./FileWrite.js')
let FileClose = require('./FileClose.js')
let FileTruncate = require('./FileTruncate.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let CommandLong = require('./CommandLong.js')
let WaypointPush = require('./WaypointPush.js')
let FileRead = require('./FileRead.js')
let FileOpen = require('./FileOpen.js')
let SetMode = require('./SetMode.js')
let LogRequestEnd = require('./LogRequestEnd.js')

module.exports = {
  CommandTriggerInterval: CommandTriggerInterval,
  SetMavFrame: SetMavFrame,
  CommandHome: CommandHome,
  FileList: FileList,
  VehicleInfoGet: VehicleInfoGet,
  MountConfigure: MountConfigure,
  CommandBool: CommandBool,
  FileMakeDir: FileMakeDir,
  MessageInterval: MessageInterval,
  FileRemove: FileRemove,
  LogRequestData: LogRequestData,
  ParamSet: ParamSet,
  ParamGet: ParamGet,
  CommandTriggerControl: CommandTriggerControl,
  ParamPull: ParamPull,
  FileRename: FileRename,
  WaypointSetCurrent: WaypointSetCurrent,
  ParamPush: ParamPush,
  CommandInt: CommandInt,
  LogRequestList: LogRequestList,
  FileChecksum: FileChecksum,
  WaypointClear: WaypointClear,
  StreamRate: StreamRate,
  FileRemoveDir: FileRemoveDir,
  CommandTOL: CommandTOL,
  WaypointPull: WaypointPull,
  CommandAck: CommandAck,
  FileWrite: FileWrite,
  FileClose: FileClose,
  FileTruncate: FileTruncate,
  CommandVtolTransition: CommandVtolTransition,
  CommandLong: CommandLong,
  WaypointPush: WaypointPush,
  FileRead: FileRead,
  FileOpen: FileOpen,
  SetMode: SetMode,
  LogRequestEnd: LogRequestEnd,
};
