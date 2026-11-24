# 子车道与连续换道（移植计划与进度）

SUMO 中 `MSGlobals::gSublane = (lanechange.duration > 0 || lateral-resolution > 0)`：连续换道（LC2013 + `--lanechange.duration`）
与子车道模型（SL2015 + `--lateral-resolution`）共用同一套横向基础设施。`--lanechange.duration` 时
`gLateralResolution == 0`，`MSLeaderInfo` 只有 1 个子车道槽（`ceil(宽度/0)` 溢出后取 1），即仍是“最近前车”语义。

## 第一阶段：连续换道（gSublane，分辨率 0）

| SUMO | psim2 | 状态 |
|---|---|---|
| `MSAbstractLaneChangeModel::startLaneChangeManeuver / updateCompletion / primaryLaneChanged / endLaneChangeManeuver`，`MSLaneChanger::startChange / continueChange` | `lc.cpp`：`startChange`、`continueChange`、`primaryLaneChanged`；`Sim::endLaneChangeManeuver` | 完成 |
| `MSLCM_LC2013::computeSpeedLat`（maxSpeedLat、lcMaxSpeedLatStanding/Factor）、`estimateLCDuration`、`remainingTime` | `Sim::lcComputeSpeedLat`、`estimateLCDuration`、`lcRemainingTime` | 完成 |
| `MSLaneChanger::checkChange`：转弯前能否完成、目标车道前方危险前车 | `LaneChanger::continuousChangeBlocked` | 完成 |
| 规划：转弯前放慢以完成换道；越过转弯连接时中止换道 | `move.cpp` | 完成 |
| 每条边都有换道器（单车道、内部边） | `Network::allEdgesChangeLanes` | 完成 |
| 横向位置 `myState.myPosLat`、`myFurtherLanesPosLat` | `Kin::posLat`、`LatState::furtherPosLat` | 完成 |
| 影子车道 `updateShadowLane`（横向重叠 > ε，或换道未过中线时预留目标车道）、影子后续车道 | `lateral.cpp`，`Kin::shadowLane / shadowFurther` | 完成 |
| `myPartialVehicles` 含影子占用者（`setPartialOccupation`） | `rebuildPartials`、换道阶段的 `updatePartials`（`lcShadowPerEdge`） | 完成 |
| `getBackPositionOnLane` 对影子（后续）车道 | `backPosOf` | 完成 |
| planMove：影子车道上的前车（`getLastVehicleInformation` + `adaptToLeaders`） | `planMoveInternal` | 完成 |
| 碰撞检测的子车道分支（横向重叠、同步换道豁免） | `detectCollisions` | 完成 |
| `brakeForOverlap` | `Sim::brakeForOverlap` | 完成 |
| `myAlreadyChanged` 只在对向换道时置位（普通换道过中线后仍每步继续） | `LCState::changed` | 完成 |
| 附加文件里 programID="off" 的信号灯程序（MSOffTrafficLightLogic） | `tls.cpp` | 完成 |
| `processLinkApproaches` 的平行连接检查、`getFractionalVehicleLength` 忽略影子、插入时的车长折算、`MSLink` 的 gSublane 分支 | | 待做 |

结果：lc_model/continuous_lanechange、lc_model/continuous、action_step_length/LC_dynamics/continuous_lanechange（不含 TWLT）中 40 个逐步一致；
其余为 DK2008 换道模型、随机车道的概率 flow、knownbugs 用例，以及 ramp_end_maxSpeedLatStanding_0（SUMO 在过中线后停在车道末端等待换道完成，原因未查明）。

## 第二阶段：子车道模型（SL2015、MSLaneChangerSublane、多槽 MSLeaderInfo）

1. 选项与参数：`--lateral-resolution`；车型 latAlignment、minGapLat、lcSublane、lcPushy、lcPushyGap、lcImpatience、lcTimeToImpatience、
   lcAccelLat、lcTurnAlignmentDistance、lcLaneDiscipline、lcSigma、lcMaxDistLatStanding；车辆 departPosLat。
2. `MSLeaderInfo` / `MSLeaderDistanceInfo` / `MSCriticalFollowerDistanceInfo`（按子车道的前车/后车数组，车辆编号）。
3. 边的子车道几何：`MSEdge::mySublaneSides`、车道在边上的右侧位置。
4. 车辆横向函数：`getLatOffset`、`getRightSideOnLane / LeftSide`、`getRightSideOnEdge / LeftSideOnEdge`。
5. 车道查询的子车道版本：`getLastVehicleInformation`、`getFirstVehicleInformation`、`getLeadersOnConsecutive`、
   `getFollowersOnConsecutive`；`MSLane::planMovements / updateLeaderInfo` 的多槽前车。
6. `planMoveInternal`：`adaptToLeaders`（多前车）、后续车道的前车、连接的横向偏移（`MSLink::myLateralShift`）；`MSLink` 的横向冲突判断。
7. 插入：departPosLat、子车道前后车检查。
8. `MSLaneChangerSublane` 与 `MSLCM_SL2015`（约 5700 行）。
9. 输出：fcd 的 posLat。

## 第三阶段：双向左转车道（TWLT）、对向超车（共用影子车道）

## 进度（2026-10-25，已暂停）

sublane_model 302 个用例中 207 个逐步一致（`work/sl4.txt`）。本轮修正：插入前的边占用预检查（MSInsertionControl::checkCandidates）、
换道阶段的安全间距按 gComputeLC 计算（正在制动的车辆）、换道过程中其他路段车道的部分占用者即时更新（setPartialOccupation）、
lcSigma 在逐步对照时置 0。未完成：机动预留（maneuver reservations）、无横向分辨率的 SL2015、子车道换道输出，以及其余约 95 个用例。
