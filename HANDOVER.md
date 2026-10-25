# 机载交接 HANDOVER

给下一个 Cursor agent + 把工程搬到机载工控机。事实来自同目录计划画板 `biped-motor-control-plan.canvas.tsx`（无实验图）、本仓库代码、`--help`、以及 2026-09/10 台架对话。**不要把未跑过的标成完成。** 写本文时未使能电机。

**目标：** 线驱 12 电机双足底座 → MPC + Residual RL（P7 未开工）。

**本机硬件：** 双 USB2CAN · 达妙 MIT（多数）+ Motorevo HA60（踝 roll）· MBS 关节编码器 · Wheeltec N100 IMU。算法与驱动同机 onboard mini-PC。

**工作区：** `/home/ye/00_jfl_26.10/MotorCtrl`（colcon）。新代码 `src/biped_hw`。旧 `src/motor_ctrl` 只读冻结。旁路包：`src/serial`、`src/wheeltec_n100_imu`。规则：`.cursor/rules/karpathy-guidelines.mdc`、`.cursor/rules/motorctrl.mdc`。

---

## 1. Overlay（永远）

```bash
source /opt/ros/humble/setup.bash
source /home/ye/00_jfl_26.10/MotorCtrl/install/setup.bash
```

- **Never** `~/WS/install` 或其它 workspace overlay。
- 拷到机载后改第二行路径，例如 `source /home/<user>/motorctrl/install/setup.bash`。
- 确认：`echo $AMENT_PREFIX_PATH` 里应出现 `.../motorctrl/install/biped_hw` 且在 `/opt/ros/humble` 之前。

---



## 2. Hard rules


| 规则                       | 细节                                                                       |
| ------------------------ | ------------------------------------------------------------------------ |
| `--help` 是 CLI 真相        | 不要发明 flag；改代码后先 `--help` 再跑                                              |
| `--can-id` 十进制           | `12` 不是 `0x0C`。达妙上位机 UI 是十六进制（12 → `0x0C`）                               |
| 串口 by-id                 | N100 / MBS **禁止** `ttyUSB0/1`（CP210x 与 pl2303 会换号）                       |
| USB2CAN                  | 目前仍 `ttyACM0/ACM1`；udev 别名草稿未填 serial。**不要默认 ACM0=左腿**                   |
| MIT 量程用表               | `types.hpp` `default_limit`。不读 RID 21/22/23，没有 `--force-limits`             |
| `--abort-tau` 默认 TMAX    | 按电机峰值：DM10010/L **200**、DM10422 **400**、HA60 **50**。`--abort-tau N` 仍可覆盖 |
| apps 独立                  | 禁止 `#include` 另一个 app。共享逻辑在 `include/biped_hw` + `src/`                  |
| **不要恢复** `p3_left6_sync` | 用户故意删掉。CMake 无此 target。`p3_runtime` 里 `P3App::Left6` 名字残留可忽略             |


使能（`0xFC`）仅当用户本轮明确要求。故障/退出走 P5 `safety_disable_all` → `0xFD`（map 3–14）。

---



## 3. Motor map（权威：`motor_map.hpp`；yaml 只改端口）

Master **每总线都是 2**。左 ACM0 CAN **3–8**；右 ACM1 CAN **9–14**。ID **8 / 14 = HA60**，其余达妙。


| 腿   | 口（本机默认）        | Master | CAN    | 关节                 | 型号       |
| --- | -------------- | ------ | ------ | ------------------ | -------- |
| L   | `/dev/ttyACM0` | 2      | 3      | hip yaw            | DM10010L |
| L   | ACM0           | 2      | 4      | hip parallel OUTER | DM10010  |
| L   | ACM0           | 2      | 5      | hip parallel INNER | DM10010  |
| L   | ACM0           | 2      | 6      | knee pitch         | DM10422  |
| L   | ACM0           | 2      | 7      | ankle pitch        | DM10010L |
| L   | ACM0           | 2      | **8**  | ankle roll         | **HA60** |
| R   | `/dev/ttyACM1` | 2      | 9      | hip yaw            | DM10010L |
| R   | ACM1           | 2      | 10     | hip parallel OUTER | DM10010  |
| R   | ACM1           | 2      | 11     | hip parallel INNER | DM10010  |
| R   | ACM1           | 2      | 12     | knee pitch         | DM10422  |
| R   | ACM1           | 2      | 13     | ankle pitch        | DM10010L |
| R   | ACM1           | 2      | **14** | ankle roll         | **HA60** |


**MBS 8 路地址（十进制，关节空间）：** 4, 5, 6, 7, 10, 11, 12, 13。髋 yaw / 踝 roll **无编码器**。yaml `zero_rad=0` `sign=+1`，未标定。不要往传感器写新地址。

身份核对：ID6 在左口 REG `D[0]=0x06`；ID12 在右口 `D[0]=0x0C`（二进制 12，不是 BCD `0x12`）。插反了就换口，不要改 map。

Damiao 回包 CAN ID 全是 Master `0x02` → demux 用 payload `D[0]` 低 4 位 ESC_ID。

---



## 4. Phase P0–P7

状态：`DONE` / `PARTIAL` / `NOT`。画板灯 + 对话实测；未复测的不升级。


| 阶段                     | 状态          | 已核实                                                                                                                                                                                  | 未做 / 缺口                                                        |
| ---------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------- |
| **P0** 骨架 + map        | **DONE**    | 12 路表、Master=2、`hw_smoke_dual_bus` 开 ACM0/ACM1（无使能）                                                                                                                                  | udev 别名未填 serial                                               |
| **P1** 通信              | **DONE**    | 右 9–14 **comms 6/6**（`--comms-only`，drop=0）。膝 ID6/12 dump-rids：PMAX/VMAX/TMAX=**12.566 / 20 / 500**，CTRL_MODE=MIT                                                                    | 12 路 soak 20–30 min **未跑(已过/26.10.20)**                        |
| **P2** 单电机闭环           | **PARTIAL** | 双膝 pos/vel；`--stage dq` 已对齐上位机（kp=0 kd=2 dq≈5）。ID12 静摩擦 ~5–6 Nm 已知                                                                                                                   | `--stage torque` 未跑；髋 4/5、10/11 与 HA60 正式单轴运动未当 P2 gate        |
| **P3** 多电机同步           | **PARTIAL** | 见下表                                                                                                                                                                                  | `p3_dual12` 12 路运动/扫频未完成；（已过，最快500Hz/贴地）`p3_fault_inject` 未台架  |
| **P4** RobotState      | **PARTIAL** | 软件：`RobotState` 12+8+IMU。live：`/imu` latch；MBS **历史上 2/8**（膝 addr 6/12）。髋三次 IK/FK 软件自测                                                                                               | 左踝MBS异常发热                                                      |
| **P5** 安全              | **PARTIAL** | `safety.hpp`：watchdog、q_soft=**10** / q_hard=**12** rad、tau=**15** Nm、disable-all **3–14**。`p5_safety_self_test --self-test`。p3/p4 abort/SIGINT → disable-all                        | `safety_check` **未接到所有 p3 循环**（p6 有 hard-q）。无 GPIO 急停。未使能 soak |
| **P6** 算法 API          | **PARTIAL** | 软件绿：`getState`/`sendCommand`；`pid_track` 只 pack MIT PD / HA60 servo；`quintic_traj` 抄 `motor_ctrl` rest-to-rest（T=1 时 **a3=10 a4=−15 a5=6**）。`p6_traj_track` 默认右 **9–14**，`--leg left | right`。`--self-test` 无 0xFC                                    |
| **P7** MPC+Residual RL | **NOT**     | 占位。入口条件：P1 延迟、P3 周期、P5 disable-all                                                                                                                                                   | 不写 `mpc/` `rl/`                                                |




### P3 已跑过的子集


| 项                             | 结果                                                                               |
| ----------------------------- | -------------------------------------------------------------------------------- |
| `p3_dual_knee_spin` ID6+ID12  | sine ±0.05 / ±0.10；`--stage dq` wire kd=2 dq≈5，两膝 Δq≈8 后 stroke STOP + 0xFD  |
| `p3_right6_sync --comms-only` | **6/6**（曾 4/6：ID10/11 无回包，后恢复）                                                   |
| right6 enable-hold            | **6/6** `enabled=Y`，800/800 drop=0。裸跑默认只开 **ID12**，六台必须 `--ids 9,10,11,12,13,14` |
| right6 ±20° sine              | 绕 **探测 q0**（`--stroke 0.349`），**不**回固件 0。ID14 当时 ~5.13 rad。髋 10/11 跟同一电机空间行程     |


**回零（home）≠ 设零（set-zero）：**

- Home：`dm_home_to_zero` = **限速 dq**（kp=0、kd、`dq_des=-sign(q)*vel`），**不是** kp=80 阶跃到 0。失败 → disable-all，不上剖面。
- `p3_right6` / `p3_dual12` 的 `--stage sine` **不调用** home，绕 probe q0。
- `p3_dual_knee_spin` 在 sine/dq **之前** 会对两膝 home。实现仍是 **先左后右顺序** `home_one`（曾是 UX 问题：一台在走、另一台还没开始）。**代码仍顺序，未改成双轴同时回零。**
- `--set-zero` 才是固件 0xFE，见 §7。

---



## 5. Key files


| 路径                                                             | 职责                                                      |
| -------------------------------------------------------------- | ------------------------------------------------------- |
| `include/biped_hw/motor_map.hpp` + `config/motor_map.yaml`     | 权威 ID 表；yaml 只覆盖串口                                      |
| `dm_codec.hpp` / `motorevo_codec.hpp` / `frame_demux.hpp`      | MIT / HA60 servo；Damiao 按 ESC_ID demux                  |
| `usb_can_adapter.hpp` `serial_port.hpp` `bus_cycle.hpp`        | 双 USB2CAN 0x55/0xAA；write-all / drain-all               |
| `dm_mit_host.hpp`                                              | 0xFC/0xFD、RID、q0 probe、限速 home、MIT pack                 |
| `safety.hpp` `src/safety.cpp`                                  | watchdog / 10–12 rad / 15 Nm / disable-all 3–14         |
| `robot_state.hpp` `mbs_map.hpp` `mbs_host.hpp` `mbs_codec.hpp` | 12 电机 + 8 MBS + IMU latch                               |
| `hip_parallel_kin.hpp`                                         | 文档三次 IK + Newton FK；q_outer=θ2=ID4/10，q_inner=θ1=ID5/11 |
| `algo_api.hpp` `src/algo_api.cpp`                              | `getState` / `sendCommand`（髋命令走 hip_ik）                 |
| `pid_track.hpp`                                                | **只 pack**，不生成曲线，无外环主机 PID                              |
| `quintic_traj.hpp` `src/quintic_traj.cpp`                      | 五次摆：q0→q0+A→q0→q0−A→q0                                  |
| `apps/*.cpp`                                                   | 相位测试；薄 main                                             |
| `src/motor_ctrl/*`                                             | 冻结参考；不要链接进热路径                                           |
| `src/wheeltec_n100_imu/`                                       | 独立 `imu_node`；p4/p6 只 latch `/imu`                      |
| `udev/99-usb2can-biped.rules`                                  | 草稿，`ATTRS{serial}` 未填，**不要原样启用**                        |


---



## 6. 实机命令（flag 来自 `--help`）

先 overlay。**身份用 dump-rids（无** `--stage` **= 不使能）。**

```bash
# P0 smoke：打 map + 开双口，不使能
ros2 run biped_hw hw_smoke_dual_bus --require-ports \
  --left-port /dev/ttyACM0 --right-port /dev/ttyACM1

# 身份：P1/P2 CAN ID 在源码 kCanId，改完重新编译。口仍用 --port（ACM 可能对调）
ros2 run biped_hw p2_dm_test --dump-rids --port /dev/ttyACM0
ros2 run biped_hw p2_dm_test --dump-rids --port /dev/ttyACM1

# 右6 只通信（无 0xFC）。列出 9–14 才会问 HA60
ros2 run biped_hw p3_right6_sync --comms-only --port /dev/ttyACM1 \
  --ids 9,10,11,12,13,14 --duration 3 --period-ms 10 --verbose

# 右6 ±20° 正弦（使能；绕 q0；勿在未确认 comms 时跑）
ros2 run biped_hw p3_right6_sync --port /dev/ttyACM1 \
  --ids 9,10,11,12,13,14 --stage sine \
  --stroke 0.349 --vel 0.20 --duration 18 --kp 20 --kd 0.5 --abort-tau 15

# P6 离线（无硬件、无 0xFC）
ros2 run biped_hw p6_traj_track --self-test

# P6 live 例（尚未在本通跑过；机载建议先 --amp 0.10）
ros2 run biped_hw p6_traj_track --leg right --amp 0.10 --duration 8 \
  --abort-tau 15 --verbose

# MBS 8 地址顺序轮询（不使能电机）
ros2 run biped_hw mbs_encoder_recv --map --duration 2 --verbose

# N100：launch 默认 CP2102 by-id，不是 ttyUSB
ros2 launch wheeltec_n100_imu imu.launch.py
# p4 锁存 /imu（p4 自己不开 N100 串口）
ros2 run biped_hw p4_state_dump --mbs --imu-topic /imu --duration 4 --period-ms 100 --verbose

# P5 / 髋 IK 离线
ros2 run biped_hw p5_safety_self_test --self-test
ros2 run biped_hw p4_hip_kin_self_test --self-test
```

`p3_right6_sync` 默认 `--ids 12`、口 ACM1，不是整条右腿。`p3_dual12_sync` 默认 `--ids 6`，缺 ACM1 要 `--allow-single-bus` 才只跑左。

---



## 7. Set-zero（写固件零点 — 危险）


| 设备     | 命令                            | 含义                                                                    |
| ------ | ----------------------------- | --------------------------------------------------------------------- |
| Damiao | MIT **0xFE**                  | 当前轴角 → 电机固件原点。不发 0xFC，然后 0xFD                                         |
| HA60   | Motorevo **V1 0xFE**          | 同上；须列入 `--ids`                                                        |
| MBS    | `mbs_encoder_recv --set-zero` | BUS-D SetZero；要 `--addr N` 或 `--map`；不能跟 `--duration` / `--self-test` |


```bash
# 这些默认不是全 ID！
ros2 run biped_hw p3_right6_sync --set-zero          # 只写 ID12
ros2 run biped_hw p3_dual12_sync --set-zero          # 只写 ID6
# 六台 / 十二台必须显式列出，HA60 还要在 --ids 里
# ros2 run biped_hw p3_right6_sync --set-zero --ids 9,10,11,12,13,14
```

与 `--stage` 互斥。右腿已装配、ID14 曾 ~5 rad：**不要在当前姿态上设零。** Home（限速回 0）≠ 0xFE。

---



## 8. Pitfalls

- **ACM 对调：** 插拔后用 dump-rids。认 ID12 `D[0]=0x0C` 的口为右，不要假设 ACM0=左。
- **dq 的 5% VMAX 夹紧已删：** `--stage dq` 的 `--vel 5` 就是 ~5 rad/s（上限 VMAX），不要再当 1。
- **MIT kd 打包范围 0–5。** 上位机 kd=2 合法；想 kd=8 会被夹/无效。
- **PMAX 读寄存器 12.566**（不是猜 400 Nm）。pack 量纲跟 RID 走。
- **摆动绕 probe q0，不是固件 0。** ID14 可在 ~5 rad；硬拉回 0 行程极大。
- **髋 10/11（及 4/5）并联：** 电机空间同一条 traj 会让两曲柄同向走同一 Δq，机构可能别劲。关节空间应走 `hip_ik`，不要把两条电机当独立正弦。
- 只操作 `--ids` 里的电机。HA60 8/14 列了才发。`p6_traj_track` 默认 9–14 **含 ID14**。
- `p3_left6_sync` **故意删除，不要加回。**
- ID12 小行程 stick-slip：kp·0.05≈4 Nm 打不开；~6 Nm 才破开。不要用抬 abort-tau 硬扛。
- P1/P2 电机 ID 是源码 `kCanId`（`p1_dm_communicate` / `p1_evo_communicate` / `p2_dm_test` / `p2_evo_test`），无 `--can-id`。口仍显式 `--port`。

---



## 9. 机载迁移 checklist

拷树（含 `install/`）或在机载 Humble 上重建。路径变了就改 overlay。

- [x] 机载 ROS 2 **Humble**；`libeigen3-dev`（N100）；用户在 `dialout`
- [x] 拷整个 `motorctrl/` **或** `colcon build --packages-select serial wheeltec_n100_imu biped_hw`（N100 依赖工作区里的 `serial`，不是 apt `ros-humble-serial-driver`）
- [x] `source /opt/ros/humble/setup.bash` 再 `source <机载>/motorctrl/install/setup.bash` — **不是** `~/WS/install`
- [ ] `/dev/serial/by-id/` **在新 PC 上会变。** 重新 `ls /dev/serial/by-id/`，改：
  - `config/mbs_encoder.yaml`（Prolific pl2303）
  - `imu.launch.py` / `serial_port`（Silicon Labs CP2102）
- [ ] USB2CAN：新机 ACM 编号会变。用 dump-rids 认左/右；需要时填 `udev/99-usb2can-biped.rules` 的真实 `idVendor/idProduct/serial`，再改 `motor_map.yaml`
- [ ] **不要假设** `ttyACM0` **是左腿**
- [ ] 电源 **24 V**；实机电机+编码器+IMU **共电**（台架曾分电，P4 motors=0/12 是那个原因）
- [ ] **第一次只做 comms-only / dump-rids / smoke**，确认 6/6 或 12 路回包后再使能
- [ ] P6 live **尚未做过**；P7 未开始
- [ ] 关到达妙上位机再开本栈（口会被占）

本机 by-id 参考（**机载不可照抄**）：

- N100：`usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0`
- MBS：`usb-Prolific_Technology_Inc._USB-Serial_Controller_BGA_b114J19-if00-port0`

---



## 10. 机载建议下一步

1. Overlay 正确后：`hw_smoke_dual_bus --require-ports`。
2. **dump-rids 双总线**（ID6 与 ID12，必要时再扫 3–14），确认哪口是左/右。
3. `p3_right6_sync --comms-only --ids 9,10,11,12,13,14` → 要 6/6。
4. 再保守 live：`p6_traj_track --amp 0.10`（默认右 9–14，绕 q0；盯髋并联 10/11）。**本通没跑过。**
5. 共电后再 `p4_state_dump --mbs --imu-topic /imu` 看 motors/mbs valid。
6. **P7 不要开始**（P1 左腿矩阵、P3 12 路周期、P5 循环接入未全绿）。

急停：Ctrl-C → 软件 latch + disable-all `0xFD`。总线堵死时 disable 可能发不出 — 准备切断 24 V。