import {
  Button,
  Callout,
  Card,
  CardBody,
  CardHeader,
  Code,
  CollapsibleSection,
  Divider,
  Grid,
  H1,
  H2,
  H3,
  Pill,
  Row,
  Stack,
  Stat,
  Table,
  Text,
  TodoList,
  computeDAGLayout,
  useCanvasAction,
  useCanvasState,
  useHostTheme,
} from "cursor/canvas";

type PhaseId = "0" | "1" | "2" | "3" | "4" | "5" | "6" | "7";

const HANDOVER_PATH = "/home/lanluo/WS/motorctrl/HANDOVER.md";

const PHASES: Array<{
  id: PhaseId;
  title: string;
  status: "DONE" | "PARTIAL" | "NOT";
  gate: string;
  goal: string;
  verified: string;
  gap: string;
  files: string;
  sequence: string[];
  verify: string[];
  exit: string;
  not: string;
}> = [
  {
    id: "0",
    title: "P0 仓库骨架 + motor_map",
    status: "DONE",
    gate: "hw_smoke_dual_bus",
    goal: "在旧包旁新建 biped_hw，冻结旧代码。权威 motor_map + 双 UsbCanAdapter 可编译、可打开端口。不使能。",
    verified: "12 路表、Master=2、smoke 打开 ACM0/ACM1（无使能）",
    gap: "udev 别名未填 serial",
    files: "biped_hw 包；SerialPort wrap；UsbCanAdapter；motor_map.hpp + yaml。不拆 all_motor.h。",
    sequence: [
      "sibling 包 motorctrl/src/biped_hw；旧 motor_ctrl 只读。",
      "types.hpp：BusId / CanId / MotorType / JointRole。禁止 Num_Of_Motor 哨兵 OOB。",
      "motor_map.hpp 写死 12 路（Master=2 per-bus，电机 3–14）；yaml 只覆盖 port。",
      "SerialPort 打开失败 return error，禁止 exit(-1)。默认 B921600。",
      "UsbCanAdapter：一实例一串口；0x55/0xAA；left/right 各 Master 2 合法。",
      "hw_smoke_dual_bus：打印 12 行 map，打开双口。",
    ],
    verify: [
      "colcon build --packages-select biped_hw 通过。",
      "map dump：左 3–8 / 右 9–14，Master 2；4/10=OUTER，5/11=INNER。",
      "缺端口明确报错，不 abort 进程。",
    ],
    exit: "包可构建；map 已核对；双 adapter 构造成功。未 enable。",
    not: "不使能、不抽 codec、不删旧 god-file、不加 ROS 控制循环。",
  },
  {
    id: "1",
    title: "P1 电机通信",
    status: "PARTIAL",
    gate: "p1_* → p1_soak",
    goal: "按左 Damiao → 同总线 HA60 → 左 6 → 右总线 → 双 12 打通。先测延迟。1 kHz 是目标不是前提。",
    verified: "右 9–14 comms-only 6/6 drop=0。膝 ID6/12 dump-rids：PMAX/VMAX/TMAX=12.566/20/500，CTRL_MODE=MIT",
    gap: "左 3–8 全矩阵未近确认；12 路 soak 20–30 min 未跑",
    files: "dm_codec / motorevo_codec / frame_demux / cycle_logger。Damiao 以官方协议为准；Motorevo 从 all_motor.h 抽出。",
    sequence: [
      "dm_codec：MIT pack/unpack、enable 0xFC / disable 0xFD、RID 21/22/23。DM10422 读寄存器，禁止猜 400 Nm。",
      "motorevo_codec：默认 servo pack。禁止 servo 内 Damiao receive()。",
      "frame_demux：Damiao 回包 CAN ID 全是 0x02，从 D[0] 低 4 位取 ESC_ID。",
      "阶梯：单 ID → 混协议 → 左 6 write-all/read-all → 右 → 双 12 → soak。",
    ],
    verify: [
      "enable/disable 可靠；drop 与 RTT 入库。",
      "混总线不双读；12 路同一日志格式。",
      "soak 无静默卡死（本通未跑）。",
    ],
    exit: "12 路开环通信稳定；延迟分布已知。闭环未开始。",
    not: "无位置/速度/力矩闭环、无 IK、无 MPC、不加 Gitee 父仓 submodule。",
  },
  {
    id: "2",
    title: "P2 单电机闭环",
    status: "PARTIAL",
    gate: "p2_dm_mit_ladder + p2_evo_servo_track",
    goal: "一台 Damiao MIT：pos→vel→torque 阶梯；一台 Motorevo 默认 servo。只收集 tracking 指标。",
    verified: "双膝 pos/vel；--stage dq 已对齐上位机（kp=0 kd=2 dq≈5）。ID12 静摩擦量级已知",
    gap: "--stage torque 未跑；髋 4/5、10/11 与 HA60 正式单轴未当 P2 gate",
    files: "apps/p2_dm_mit_ladder.cpp、p2_evo_servo_track.cpp；metrics.hpp。不新增 mpc/。",
    sequence: [
      "台架固定或小行程止挡。先 disable 确认机械。",
      "Damiao：小 kp 位置阶跃 → 速度 → 小力矩。每级独立日志。",
      "Motorevo：本 gate 不切 mode。",
    ],
    verify: ["误差与温升可复现；急停或超时能 disable。"],
    exit: "单电机闭环指标可引用。未上多电机同步。",
    not: "不上 MPC/RL；不做 12 路同时励磁。",
  },
  {
    id: "3",
    title: "P3 多电机同步",
    status: "PARTIAL",
    gate: "p3_right6_sync / p3_dual12_sync / p3_dual_knee_spin",
    goal: "6 然后 12 同一控制周期。量化负载、最坏延迟、dropout。正式安全策略在 P5。",
    verified:
      "双膝 triangle / --stage dq；p3_right6_sync --comms-only 6/6；right6 enable-hold 6/6（须显式 --ids 9–14）",
    gap: "p3_left6_sync 已删不要加回；p3_dual12 12 路运动/扫频未完成；p3_fault_inject 未台架",
    files: "bus_cycle.hpp；p3_right6_sync / p3_dual12_sync / p3_dual_knee_spin / p3_runtime。无 p3_left6_sync。",
    sequence: [
      "右 6 先 comms-only，再考虑使能。裸跑默认只开 ID12，六台必须 --ids 9,10,11,12,13,14。",
      "扩到 12：两 USB2CAN 并行，周期结束对齐。",
      "扫频与 fault inject 仍待做。",
      "triangle 绕探测 q0，不回固件零。home ≠ set-zero（0xFE）。",
    ],
    verify: ["周期 overrun、最坏 RTT、每 ID drop 有表。", "单电机丢失时行为有日志（策略在 P5）。"],
    exit: "选定生产周期（未测通则禁止宣称 1 kHz）。",
    not: "不要恢复 p3_left6_sync；不上关节 IK；不把故障当正常忽略。",
  },
  {
    id: "4",
    title: "P4 状态 + 时间戳",
    status: "PARTIAL",
    gate: "p4_state_dump",
    goal: "RobotState：12 电机 + 8 关节 MBS + N100 /imu，同一 steady 周期戳。缺传感器 valid=false。",
    verified: "软件 12+8+IMU。髋三次 IK/FK 自测。实机共电（用户确认；台架曾分电导致 motors=0/12）",
    gap: "髋/踝 6 路 MBS 历史上静默；听包与 bus_cycle 写周期对齐、校准零点未完",
    files: "robot_state.hpp、mbs_map.hpp、p4_state_dump、mbs_encoder_recv --map、hip_parallel_kin。",
    sequence: [
      "MotorFeedback[12] 对齐 kMotorMap；缺总线 valid=false，不填假数。",
      "MbsFeedback[8]：地址 4,5,6,7,10,11,12,13。髋 yaw / 踝 roll 无槽。",
      "ImuFeedback 锁存 /imu（p4 不打开 N100 串口）。",
      "默认听 USB-CAN（无 0xFC）。--self-test 无硬件。",
    ],
    verify: [
      "p4_state_dump --self-test；8 槽存在。",
      "另开 imu_node 后 --imu-topic /imu 能 latch。",
    ],
    exit: "上层能读同一 RobotState。髋 stub 与校准零点未完。",
    not: "P5 watchdog 本阶段不冒充完成；不使能；不抬 abort-tau。",
  },
  {
    id: "5",
    title: "P5 安全层",
    status: "PARTIAL",
    gate: "p5_safety_self_test",
    goal: "限位、超时、watchdog、dropout→disable-all、e-stop hook。与控制器分文件。",
    verified:
      "safety.hpp：watchdog、q_soft=10 / q_hard=12 rad、tau=15 Nm、disable-all 3–14。--self-test。p3/p4 abort/SIGINT → disable-all",
    gap: "safety_check 未接到所有 p3 循环（p6 有 hard-q）。无 GPIO 急停。未使能 soak",
    files: "safety.hpp + src/safety.cpp；apps/p5_safety_self_test.cpp。无 estop.hpp / AppBase。",
    sequence: [
      "watchdog pet/timeout、软限位 hold、硬限位 disable、软件 e-stop latch（SIGINT/SIGTERM，无 GPIO）。",
      "safety_disable_all：dm_send_disable 0xFD 到 motor_map 12（左 3–8 / 右 9–14）。",
      "p5_safety_self_test --self-test 注入 timeout/estop/hard_q/drop；不断言 0xFC。",
    ],
    verify: ["--self-test 请求 12 路 disable-all；无 0xFC。", "软限位 hold 不 disable。安全代码无 PID/MPC。"],
    exit: "无 safety 绿不能上算法 API 的 live 路径。",
    not: "不在本阶段做步态；不把安全逻辑写进 codec。",
  },
  {
    id: "6",
    title: "P6 算法部署 API",
    status: "PARTIAL",
    gate: "p6_traj_track",
    goal: "同机 getState / sendCommand。pid_track 只 pack；轨迹在 quintic_traj。绕探测 q0。",
    verified:
      "软件绿：getState/sendCommand；pid_track 只 pack；quintic rest-to-rest（T=1 时 a3=10 a4=−15 a5=6）。默认右 9–14。--self-test 无 0xFC",
    gap: "无 live P6（本通明确未使能）",
    files: "algo_api.hpp、pid_track.hpp、quintic_traj.hpp；apps/p6_traj_track.cpp。无 plugins/、无 SHM。",
    sequence: [
      "getState：MIT fb + 可选 MBS + 可选 IMU latch。缺则 valid=false。",
      "sendCommand：电机 q_des；髋 roll/pitch 走 hip_ik（CAN 4/5、10/11）。退出 P5 disable-all。",
      "quintic：q0→q0+A→q0→q0−A→q0。不回固件零。",
      "p6_traj_track：薄 main。默认右 9–14（含 HA60 14）。--leg left|right。--self-test 无硬件。",
    ],
    verify: ["--help / --self-test rc=0（pack 有限、无 0xFC）。", "默认右 9–14，不是 ID12 only。"],
    exit: "上层只调 getState/sendCommand + RobotState。MPC/RL 在 P7。",
    not: "不实现 MPC/RL；无 plugin；不经 ROS topic 当热路径。",
  },
  {
    id: "7",
    title: "P7 MPC + Residual RL",
    status: "NOT",
    gate: "仅当 P1–P5 全绿",
    goal: "占位。通信、同步、安全未绿之前不写 MPC/RL 控制。",
    verified: "无。入口条件：P1 延迟、P3 周期、P5 disable-all",
    gap: "不写 mpc/ 或 rl/ 源文件",
    files: "本执行计划不创建 mpc/ 或 rl/。",
    sequence: [
      "入口：P1 延迟已知、P3 周期选定、P5 disable-all 验证。",
      "届时再定 plugin vs SHM；残差 RL 只输出 residual。",
      "髋 FK/IK 已在下层；禁止把旧 stub 当真值。",
    ],
    verify: ["本阶段无代码验收；过早合入 MPC 视为计划失败。"],
    exit: "N/A — 后续单独计划。",
    not: "现在不写 MPC、不训 RL、不把旧 ZMP god-file 当架构。",
  },
];

const PHASE_TODOS = PHASES.map((p) => ({
  id: p.id,
  content: `${p.title} — ${p.status}. 已核实：${p.verified}。缺口：${p.gap}`,
  status:
    p.status === "DONE"
      ? ("completed" as const)
      : p.status === "PARTIAL"
        ? ("in_progress" as const)
        : ("pending" as const),
}));

const MOTOR_ROWS = [
  ["L", "ttyACM0", "2", "3", "hip yaw", "DM10010L"],
  ["L", "ttyACM0", "2", "4", "hip parallel OUTER", "DM10010"],
  ["L", "ttyACM0", "2", "5", "hip parallel INNER", "DM10010"],
  ["L", "ttyACM0", "2", "6", "knee pitch", "DM10422"],
  ["L", "ttyACM0", "2", "7", "ankle pitch", "DM10010L"],
  ["L", "ttyACM0", "2", "8", "ankle roll", "HA60"],
  ["R", "ttyACM1", "2", "9", "hip yaw", "DM10010L"],
  ["R", "ttyACM1", "2", "10", "hip parallel OUTER", "DM10010"],
  ["R", "ttyACM1", "2", "11", "hip parallel INNER", "DM10010"],
  ["R", "ttyACM1", "2", "12", "knee pitch", "DM10422"],
  ["R", "ttyACM1", "2", "13", "ankle pitch", "DM10010L"],
  ["R", "ttyACM1", "2", "14", "ankle roll", "HA60"],
];

const MBS_ROWS = [
  ["1", "left hip roll", "4"],
  ["2", "left hip pitch", "5"],
  ["3", "left knee pitch", "6"],
  ["4", "left ankle pitch", "7"],
  ["5", "right hip roll", "10"],
  ["6", "right hip pitch", "11"],
  ["7", "right knee pitch", "12"],
  ["8", "right ankle pitch", "13"],
];

function LayerDag() {
  const theme = useHostTheme();
  const layout = computeDAGLayout({
    nodes: [
      { id: "serial" },
      { id: "usbL" },
      { id: "usbR" },
      { id: "dm" },
      { id: "evo" },
      { id: "cycle" },
      { id: "safety" },
      { id: "state" },
      { id: "api" },
      { id: "pid" },
      { id: "traj" },
    ],
    edges: [
      { from: "serial", to: "usbL" },
      { from: "serial", to: "usbR" },
      { from: "usbL", to: "dm" },
      { from: "usbL", to: "evo" },
      { from: "usbR", to: "dm" },
      { from: "usbR", to: "evo" },
      { from: "dm", to: "cycle" },
      { from: "evo", to: "cycle" },
      { from: "cycle", to: "safety" },
      { from: "safety", to: "state" },
      { from: "state", to: "api" },
      { from: "traj", to: "api" },
      { from: "api", to: "pid" },
    ],
    direction: "horizontal",
    nodeWidth: 108,
    nodeHeight: 34,
    rankGap: 36,
    nodeGap: 18,
    padding: 12,
  });
  const labels: Record<string, string> = {
    serial: "SerialPort",
    usbL: "UsbCan L",
    usbR: "UsbCan R",
    dm: "dm_codec",
    evo: "evo_codec",
    cycle: "bus_cycle",
    safety: "safety",
    state: "RobotState",
    api: "algo_api",
    pid: "pid_track",
    traj: "quintic_traj",
  };
  return (
    <svg
      width={layout.width}
      height={layout.height}
      viewBox={`0 0 ${layout.width} ${layout.height}`}
      role="img"
      aria-label="biped_hw layer dependency from SerialPort to algo_api"
    >
      {layout.edges.map((e) => (
        <line
          key={`${e.from}-${e.to}`}
          x1={e.sourceX}
          y1={e.sourceY}
          x2={e.targetX}
          y2={e.targetY}
          stroke={theme.stroke.secondary}
          strokeWidth={1}
        />
      ))}
      {layout.nodes.map((n) => (
        <g key={n.id} transform={`translate(${n.x},${n.y})`}>
          <rect
            width={108}
            height={34}
            rx={4}
            fill={n.id === "safety" ? theme.fill.tertiary : theme.bg.elevated}
            stroke={n.id === "api" ? theme.accent.primary : theme.stroke.primary}
          />
          <text
            x={54}
            y={22}
            textAnchor="middle"
            fill={theme.text.primary}
            fontSize={11}
          >
            {labels[n.id]}
          </text>
        </g>
      ))}
    </svg>
  );
}

export default function BipedMotorControlPlan() {
  const dispatch = useCanvasAction();
  const [phaseId, setPhaseId] = useCanvasState<PhaseId>("phase", "6");
  const phase = PHASES.find((p) => p.id === phaseId) ?? PHASES[0];

  return (
    <Stack gap={24}>
      <Stack gap={8}>
        <H1>索驱双足电机控制 — 计划 / 状态</H1>
        <Text tone="secondary">
          分阶段计划板，不是实验笔记。算法与驱动同机 onboard mini-PC。工作区
          `motorctrl/` · 新包 `src/biped_hw` · 旧 `src/motor_ctrl` 只读。未使能电机不标完成。
        </Text>
        <Row gap={8} align="center" wrap>
          <Button
            variant="primary"
            onClick={() => dispatch({ type: "openFile", path: HANDOVER_PATH })}
          >
            Open HANDOVER.md
          </Button>
          <Text size="small" tone="tertiary">
            交接原文：`/home/lanluo/WS/motorctrl/HANDOVER.md`
          </Text>
        </Row>
      </Stack>

      <Grid columns={4} gap={16}>
        <Stat value="P0 DONE" label="骨架 + motor_map" tone="success" />
        <Stat value="P1–P6 PARTIAL" label="软件或子集，未全绿" tone="warning" />
        <Stat value="P7 NOT" label="MPC + Residual RL 未开工" />
        <Stat value="abort-tau 15" label="禁止 --force-limits" tone="info" />
      </Grid>

      <Callout tone="warning" title="本通明确不做">
        12 路 soak、整条左腿运动矩阵、P6 live 使能、P7 MPC/RL、抬 abort-tau、--force-limits、恢复 p3_left6_sync。画板已去掉历史实验曲线与台架 dump。
      </Callout>

      <H2>P0–P7 状态</H2>
      <Text size="small" tone="tertiary">
        DONE / PARTIAL / NOT 来自 HANDOVER。点选一行打开该阶段计划。未复测的不升级。
      </Text>
      <TodoList
        todos={PHASE_TODOS}
        onTodoClick={(t) => setPhaseId(t.id as PhaseId)}
      />
      <Table
        headers={["阶段", "状态", "已核实", "缺口"]}
        rows={PHASES.map((p) => [p.title, p.status, p.verified, p.gap])}
        striped
        stickyHeader
        rowTone={PHASES.map((p) =>
          p.status === "DONE"
            ? "success"
            : p.status === "PARTIAL"
              ? "warning"
              : "neutral",
        )}
      />

      <H2>硬件图（权威 map，不是实验数据）</H2>
      <Text size="small" tone="tertiary">
        Master 每总线都是 2。左默认 ACM0 CAN 3–8；右 ACM1 CAN 9–14。ID 8/14 = HA60，其余达妙。yaml 只改端口。插反了换口，不改 map。Damiao 回包 CAN ID 全是 0x02 → demux 用 D[0] 低 4 位 ESC_ID。
      </Text>
      <Table
        headers={["腿", "口（本机默认）", "Master", "CAN", "关节", "型号"]}
        rows={MOTOR_ROWS}
        striped
        stickyHeader
      />
      <Grid columns={2} gap={16}>
        <Stack gap={8}>
          <H3>MBS 8 路（十进制，关节空间）</H3>
          <Table
            headers={["#", "关节", "addr"]}
            rows={MBS_ROWS}
            striped
          />
          <Text size="small" tone="tertiary">
            髋 yaw / 踝 roll 无编码器。yaml zero_rad=0 sign=+1，未标定。不要往传感器写新地址。
          </Text>
        </Stack>
        <Stack gap={8}>
          <H3>N100 IMU</H3>
          <Text>
            Wheeltec N100 独立 `imu_node`。p4/p6 只 latch `/imu`，自己不开 N100 串口。口用 CP2102 by-id，禁止 ttyUSB0/1。
          </Text>
          <Callout tone="info" title="身份核对">
            ID6 左口 REG D[0]=0x06；ID12 右口 D[0]=0x0C（二进制 12，不是 BCD 0x12）。不要默认 ACM0=左腿。
          </Callout>
        </Stack>
      </Grid>

      <H2>架构</H2>
      <Text size="small" tone="tertiary">
        顶层只有 apps/ 相位测试：每个 .cpp 独立可执行，禁止 include 另一个 app。共享逻辑在 include/biped_hw 与 src/。Colcon 工作区 motorctrl/；旧包冻结。
      </Text>
      <Callout tone="danger" title="不要恢复 p3_left6_sync">
        用户故意删除。CMake 无此 target。p3_runtime 里 P3App::Left6 名字残留可忽略。用 p3_right6_sync / p3_dual12_sync / p3_dual_knee_spin。
      </Callout>
      <Table
        headers={["层", "路径", "职责"]}
        rows={[
          [
            "apps/",
            "apps/*.cpp 各一个 executable",
            "相位测试；禁止 include 另一个 app；main：parse → open → loop → cleanup",
          ],
          [
            "安全",
            "safety.hpp / src/safety.cpp",
            "watchdog、限位、e-stop latch、双总线 0xFD",
          ],
          [
            "算法 API",
            "algo_api / quintic_traj / pid_track",
            "getState、sendCommand、五次 q_des；pid_track 只 pack。不是 plugin",
          ],
          [
            "协议 + 总线",
            "dm_codec、usb_can_adapter、bus_cycle、robot_state、mbs_*",
            "编解码与设备 I/O",
          ],
        ]}
      />
      <Text size="small" tone="tertiary">
        safety 夹在 bus_cycle 与 RobotState 之间；算法只碰 getState/sendCommand。
      </Text>
      <LayerDag />

      <H2>安全 / P6 / 机载</H2>
      <Grid columns={3} gap={16}>
        <Callout tone="danger" title="安全（一句话）">
          故障/退出 → safety_disable_all → 0xFD 打 map 3–14。台架 abort-tau 一直 15 Nm（p3 --help 默认 30 仍传 15）。禁止 --force-limits（RID 失败就停）。无 GPIO 急停；Ctrl-C 软件 latch。总线堵死时 disable 可能发不出 — 准备切断 24 V。
        </Callout>
        <Callout tone="info" title="P6">
          quintic 绕探测 q0（q0→q0+A→q0→q0−A→q0），不回固件零。p6_traj_track 默认右 9–14（含 HA60 14）。本通只 --self-test，无 live 使能。pid_track 只 pack MIT PD / HA60 servo，无外环主机 PID。髋 10/11 命令走 hip_ik。
        </Callout>
        <Callout tone="warning" title="机载">
          新 PC 上 by-id 与 ACM 编号会变。不要假设 ttyACM0 是左腿。第一次：dump-rids 认左/右，再 comms-only，确认回包后再使能。overlay 永远是 motorctrl/install，不是 ~/WS/install。实机电机+编码器+IMU 共电。
        </Callout>
      </Grid>

      <H2>阶段详情</H2>
      <Text tone="secondary">点选阶段查看工作序列、验收与禁止项。</Text>
      <Row gap={8} wrap>
        {PHASES.map((p) => (
          <span key={p.id}>
            <Pill
              active={phaseId === p.id}
              onClick={() => setPhaseId(p.id)}
            >
              {p.title} · {p.status}
            </Pill>
          </span>
        ))}
      </Row>
      <Card>
        <CardHeader trailing={<Pill size="sm" active>{phase.status}</Pill>}>
          {phase.title} · {phase.gate}
        </CardHeader>
        <CardBody>
          <Stack gap={12}>
            <Text weight="semibold">{phase.goal}</Text>
            <Text size="small" tone="secondary">
              文件：{phase.files}
            </Text>
            <H3>工作序列</H3>
            <Table
              headers={["#", "步骤"]}
              rows={phase.sequence.map((s, i) => [String(i + 1), s])}
            />
            <H3>验证</H3>
            <Table
              headers={["检查项"]}
              rows={phase.verify.map((v) => [v])}
            />
            <Text>
              <Text weight="semibold">退出：</Text> {phase.exit}
            </Text>
            <Callout tone="neutral" title="本阶段不要做">
              {phase.not}
            </Callout>
          </Stack>
        </CardBody>
      </Card>

      <CollapsibleSection title="硬约束与关键文件" defaultOpen={false} count={2}>
        <Stack gap={12}>
          <Callout tone="warning" title="硬约束">
            髋是空间双链 2-DoF 近平行四边形，不是 ID 专用 pitch/roll 电机。q_outer=θ2（CAN 4/10），q_inner=θ1（5/11）；右腿 roll 镜像。弃用旧踝多项式。Master=2；禁止把 MasterId 写入 Damiao 回包 map。DM10422 PMAX/VMAX/TMAX 读寄存器。Motorevo 默认 servo。摆动绕 probe q0，不是固件 0。Home（限速 dq）≠ set-zero（0xFE）。
          </Callout>
          <Table
            headers={["路径", "职责"]}
            rows={[
              ["motor_map.hpp + config/motor_map.yaml", "权威 ID 表；yaml 只覆盖串口"],
              ["dm_codec / motorevo_codec / frame_demux", "MIT / HA60 servo；Damiao 按 ESC_ID demux"],
              ["usb_can_adapter / serial_port / bus_cycle", "双 USB2CAN 0x55/0xAA；write-all / drain-all"],
              ["dm_mit_host.hpp", "0xFC/0xFD、RID、q0 probe、限速 home"],
              ["safety.hpp", "watchdog / 10–12 rad / 15 Nm / disable-all 3–14"],
              ["robot_state / mbs_map / mbs_host", "12 电机 + 8 MBS + IMU latch"],
              ["hip_parallel_kin.hpp", "文档三次 IK + Newton FK"],
              ["algo_api / pid_track / quintic_traj", "getState/sendCommand；只 pack；五次绕 q0"],
              ["apps/*.cpp", "相位测试；薄 main"],
              ["udev/99-usb2can-biped.rules", "草稿，ATTRS{serial} 未填，不要原样启用"],
            ]}
            striped
          />
        </Stack>
      </CollapsibleSection>

      <CollapsibleSection title="机载下一步（HANDOVER §10）" defaultOpen={false}>
        <Table
          headers={["#", "动作"]}
          rows={[
            ["1", "overlay 正确后 hw_smoke_dual_bus --require-ports"],
            ["2", "dump-rids 双总线（ID6 与 ID12），确认哪口是左/右"],
            ["3", "p3_right6_sync --comms-only --ids 9,10,11,12,13,14 → 要 6/6"],
            ["4", "再保守 live：p6_traj_track --amp 0.10（本通没跑过）"],
            ["5", "共电后再 p4_state_dump --mbs --imu-topic /imu"],
            ["6", "P7 不要开始"],
          ]}
        />
      </CollapsibleSection>

      <Divider />
      <Text size="small" tone="tertiary">
        Source: `/home/lanluo/WS/motorctrl/HANDOVER.md` · 计划板，无实验曲线。Cursor 打开副本在 canvases/；同内容也放在 HANDOVER 同目录。
      </Text>
    </Stack>
  );
}
