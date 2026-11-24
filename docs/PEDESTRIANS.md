# psim2 行人与人行横道

上游模型 MSPModel_Striping 来自 Eclipse SUMO 1.28 (`src/microsim/transportables/`)。本项目的行人代码在 `peds.cpp`、`pedrouter.cpp`、`ped.hpp`，车辆一侧在 `link.cpp`、`move.cpp`、`insert.cpp`、`lc.cpp`。

## 已实现

- **路网**：人行道、walkingarea、crossing（`EDGE` 的 function），行人 link 与车辆 link 的冲突（crossing 作为 foe lane），共享空间（walkingAreaFoe / walkingAreaFoeExit），`havePedestrianCrossingFoe`、`distToFoePedCrossing`。additional 文件里的 tlLogic 会替换路网中的信号配时（最后加载的程序生效）。
- **需求**（`psim2scen.py`）：`person`、`personFlow`（周期、个数；按概率和泊松的由 psim2 用自己的随机数展开）、`walk`（edges / route，或只给 from / to / busStop：出发时用行人路由器现算，即只步行的 personTrip）、人员 `stop`（等待）。
- **行人路由器**：SUMO 只含行人的 IntermodalNetwork（双向人行边、walkingarea 单节点、出发/到达连接器），Dijkstra 的平局按数字 ID 处理，与 SUMO 相同；人行横道按信号状态加罚时。
- **striping 模型**：条带、障碍物、效用选条带、dawdling、堵塞后挤过（jamTime）、为对向保留条带、walkingarea 路径（贝塞尔形状）与坐标变换、闯红灯（jmDriveAfterRedTime）、黄灯、优先人行横道的接近登记。
- **车辆交互**：车辆为人行横道上的行人停车（blockedAtDist、jmCrossingGap、jmStoplineCrossingGap）、为走向优先人行横道的行人让行、共享 walkingarea 上避让（checkWalkingAreaFoe）、同车道上的行人（nextBlocking：跟车、插入、换道、LC2013 的速度判断）；行人把接近人行横道的车辆、同车道的车辆、walkingarea 上的车辆当作障碍。jmIgnoreFoeProb / jmIgnoreFoeSpeed / jmIgnoreJunctionFoeProb / jmTimegapMinor / jmStoplineGap(Minor)。
- **输出**：`personfcd.csv`（x、y、角度、速度、边、位置）、`personinfo.csv`（每段步行的出发、到达、时长、路程、时间损失、等待时间）。

## 并行方式

- `--sumo-order`：按 SUMO 的顺序串行（车道按数字 ID，本步换过车道的行人只当障碍），用于逐步对照。
- 默认并行：每个方向分三波——人行横道、walkingarea、其余车道（这正是 SUMO 路网里它们数字 ID 的先后）。一波内各车道并行，一个车道任务只改本车道的行人；读同一波其他车道时用这一波开始时的快照（同一波的车道之间本来就不互相看），读其他波的车道时用实时状态，所以行人看到前方车道的情况与 SUMO 一致（编号靠前的已经走过）。跨车道的变化（换车道、走完一段、登记/注销人行横道接近）先记成事件，波末按车道顺序应用。结果与线程数无关（1、4、6、8 线程输出逐字节相同），ThreadSanitizer 无报告。
- 车辆读行人状态只发生在行人移动之后，只读，不需要同步。

## 验证

- **逐步对照**（`tools/sumotest.py pedestrian_model/striping --derandomize`，关掉 dawdling 和速度偏差，比较每步每人的 x、y、角度、速度、边、位置，车辆的车道、位置、速度）：172 个测试中 133 个完全一致（含行人与铁路道口、共享空间里的车辆）。
- **开随机的统计对照**（`--stats 20`，SUMO 和 psim2 各 20 个种子，psim2 用并行模式）：plaza、jam_trafficlight、walkingarea_jam、walkingarea_parallel（车辆与行人共享空间）、3_arm_pathing、双向流等，步行时长、时间损失、等待时间的 |z| ≤ 1.2。双向两条带的流里两边都会偶尔出现大堵（各 200 个种子：SUMO 1/220，psim2 1/220），非堵塞种子的平均步行时间 115.48 s 对 115.5 s。

## 乘车、换乘与限制

公交乘车、换乘、站点 access 和等待位置分别由 `transport.cpp`、`trips.cpp` 和 `peds.cpp` 处理。铁路道口与信号见 `rail.cpp`。子车道实现及其验证范围见 [SUBLANE.md](SUBLANE.md)。

v0.1 尚未实现出租车服务。断开的步行路段和不连续的行程阶段仍可能触发错误，具体处理取决于阶段类型。
