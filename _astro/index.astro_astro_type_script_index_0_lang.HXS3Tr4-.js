var e={astar:`// astar.mbt - A* 路径规划
// astar.mbt - A* 路径规划

///|
pub struct Point {
  x : Int
  y : Int
}

///|
pub fn Point::new(x : Int, y : Int) -> Point {
  { x, y }
}

///|
pub fn point_eq(a : Point, b : Point) -> Bool {
  a.x == b.x && a.y == b.y
}

///|
fn heuristic(a : Point, b : Point) -> Int {
  (a.x - b.x).abs() + (a.y - b.y).abs()
}

///|
fn get_neighbors(p : Point, width : Int, height : Int) -> Array[Point] {
  let neighbors = []
  let dxs = [-1, 1, 0, 0]
  let dys = [0, 0, -1, 1]
  let mut i = 0
  while i < 4 {
    let nx = p.x + dxs[i]
    let ny = p.y + dys[i]
    if nx >= 0 && nx < width && ny >= 0 && ny < height {
      neighbors.push(Point::new(nx, ny))
    }
    i = i + 1
  }
  neighbors
}

///|
pub fn contains_point(arr : Array[Point], p : Point) -> Bool {
  let mut i = 0
  while i < arr.length() {
    if point_eq(arr[i], p) {
      return true
    }
    i = i + 1
  }
  false
}

///|
fn find_point(arr : Array[Point], p : Point) -> Int {
  let mut i = 0
  while i < arr.length() {
    if point_eq(arr[i], p) {
      return i
    }
    i = i + 1
  }
  -1
}

///|
pub fn a_star(
  start : Point,
  goal : Point,
  width : Int,
  height : Int,
  obstacles : Array[Point],
) -> Array[Point]? {
  if contains_point(obstacles, start) || contains_point(obstacles, goal) {
    return None
  }

  let mut open_set = [start]
  let closed_set = []
  let came_from = []
  let g_score = [0]
  let f_score = [heuristic(start, goal)]

  while open_set.length() > 0 {
    // 找最小 f 的节点
    let mut current_idx = 0
    let mut current_f = f_score[0]
    let mut i = 1
    while i < open_set.length() {
      if f_score[i] < current_f {
        current_f = f_score[i]
        current_idx = i
      }
      i = i + 1
    }

    let current = open_set[current_idx]
    if point_eq(current, goal) {
      // 重建路径
      let path = [goal]
      let mut cur = current
      let mut found = true
      while found {
        let idx = find_point(came_from, cur)
        if idx == -1 {
          found = false
        } else {
          cur = came_from[idx]
          path.push(cur)
        }
      }
      // 反转路径
      let reversed = []
      let mut j = path.length() - 1
      while j >= 0 {
        reversed.push(path[j])
        j = j - 1
      }
      return Some(reversed)
    }

    // 从 open_set 移除 current
    let new_open = []
    let mut idx = 0
    while idx < open_set.length() {
      if !point_eq(open_set[idx], current) {
        new_open.push(open_set[idx])
      }
      idx = idx + 1
    }
    open_set = new_open
    closed_set.push(current)

    let neighbors = get_neighbors(current, width, height)
    let mut ni = 0
    while ni < neighbors.length() {
      let neighbor = neighbors[ni]
      if contains_point(closed_set, neighbor) {
        ni = ni + 1
        continue
      }
      if contains_point(obstacles, neighbor) {
        ni = ni + 1
        continue
      }

      let tentative_g = g_score[current_idx] + 1
      let neighbor_idx = find_point(open_set, neighbor)
      if neighbor_idx == -1 {
        open_set.push(neighbor)
        came_from.push(current)
        g_score.push(tentative_g)
        f_score.push(tentative_g + heuristic(neighbor, goal))
      } else if tentative_g < g_score[neighbor_idx] {
        came_from[neighbor_idx] = current
        g_score[neighbor_idx] = tentative_g
        f_score[neighbor_idx] = tentative_g + heuristic(neighbor, goal)
      }
      ni = ni + 1
    }
  }

  None
}

pub struct Point { x: Int, y: Int }
// ...`,pid:`// pid.mbt - PID 控制器
// pid.mbt - 完整 PID 控制器

///|
pub struct PID {
  kp : Double
  ki : Double
  kd : Double
  setpoint : Double
  integral : Double
  prev_error : Double
}

///|
pub fn PID::new(kp : Double, ki : Double, kd : Double) -> PID {
  { kp, ki, kd, setpoint: 0.0, integral: 0.0, prev_error: 0.0 }
}

///|
pub fn PID::set_setpoint(self : PID, sp : Double) -> PID {
  {
    kp: self.kp,
    ki: self.ki,
    kd: self.kd,
    setpoint: sp,
    integral: 0.0, // 重置积分
    prev_error: 0.0, // 重置误差
  }
}

///|
pub fn PID::update(self : PID, measurement : Double, dt : Double) -> Double {
  let error = self.setpoint - measurement
  let p_term = self.kp * error

  let integral = self.integral + error * dt
  // 积分钳位，防止饱和
  let integral_clamped = if integral > 100.0 {
    100.0
  } else if integral < -100.0 {
    -100.0
  } else {
    integral
  }
  let i_term = self.ki * integral_clamped

  let derivative = if dt > 0.0 { (error - self.prev_error) / dt } else { 0.0 }
  let d_term = self.kd * derivative

  p_term + i_term + d_term
}

pub struct PID { kp: Double, ki: Double, kd: Double }
// ...`,swarm:`// swarm.mbt - 集群协同评估
///|
/// Swarm coordination module for AgriDroneSwarm.

///|
pub fn swarm_clamp_non_negative(value : Int) -> Int {
  if value < 0 {
    0
  } else {
    value
  }
}

///|
pub fn swarm_clamp_percent(value : Int) -> Int {
  if value < 0 {
    0
  } else if value > 100 {
    100
  } else {
    value
  }
}

///|
pub fn swarm_active_ratio_percent(
  active_drones : Int,
  total_drones : Int,
) -> Int {
  let active = swarm_clamp_non_negative(active_drones)
  let total = swarm_clamp_non_negative(total_drones)

  if total <= 0 {
    0
  } else {
    swarm_clamp_percent(active * 100 / total)
  }
}

///|
pub fn swarm_average_load_percent(
  total_load_percent : Int,
  drone_count : Int,
) -> Int {
  let load = swarm_clamp_non_negative(total_load_percent)
  let count = swarm_clamp_non_negative(drone_count)

  if count <= 0 {
    0
  } else {
    swarm_clamp_percent(load / count)
  }
}

///|
pub fn swarm_communication_quality_score(
  connected_drones : Int,
  total_drones : Int,
  packet_loss_percent : Int,
) -> Int {
  let connected_ratio = swarm_active_ratio_percent(
    connected_drones, total_drones,
  )
  let packet_loss = swarm_clamp_percent(packet_loss_percent)

  swarm_clamp_percent(connected_ratio - packet_loss)
}

///|
pub fn swarm_coordination_score(
  active_ratio_percent : Int,
  communication_quality_score : Int,
  average_load_percent : Int,
) -> Int {
  let active = swarm_clamp_percent(active_ratio_percent)
  let communication = swarm_clamp_percent(communication_quality_score)
  let load = swarm_clamp_percent(average_load_percent)

  let load_balance_score = 100 - load / 2
  swarm_clamp_percent(active / 3 + communication / 3 + load_balance_score / 3)
}

///|
pub fn swarm_mission_capacity_hectares(
  active_drones : Int,
  hectares_per_drone : Int,
) -> Int {
  swarm_clamp_non_negative(active_drones) *
  swarm_clamp_non_negative(hectares_per_drone)
}

///|
pub fn swarm_required_drones(
  mission_hectares : Int,
  hectares_per_drone : Int,
) -> Int {
  let mission = swarm_clamp_non_negative(mission_hectares)
  let capacity = swarm_clamp_non_negative(hectares_per_drone)

  if capacity <= 0 {
    0
  } else {
    (mission + capacity - 1) / capacity
  }
}

///|
pub fn swarm_has_enough_drones(
  active_drones : Int,
  required_drones : Int,
) -> Bool {
  swarm_clamp_non_negative(active_drones) >=
  swarm_clamp_non_negative(required_drones)
}

///|
pub fn swarm_status(
  coordination_score : Int,
  has_enough_drones : Bool,
) -> String {
  let score = swarm_clamp_percent(coordination_score)

  if !has_enough_drones {
    "insufficient_drones"
  } else if score >= 80 {
    "excellent"
  } else if score >= 60 {
    "stable"
  } else if score >= 40 {
    "weak"
  } else {
    "unstable"
  }
}

///|
pub fn swarm_should_rebalance(
  average_load_percent : Int,
  communication_quality_score : Int,
  inactive_drones : Int,
) -> Bool {
  let load = swarm_clamp_percent(average_load_percent)
  let communication = swarm_clamp_percent(communication_quality_score)
  let inactive = swarm_clamp_non_negative(inactive_drones)

  load >= 75 || communication < 55 || inactive > 0
}

///|
pub fn swarm_summary() -> Unit {
  let total_drones = 12
  let active_drones = 10
  let connected_drones = 9
  let inactive_drones = total_drones - active_drones
  let total_load = 680
  let packet_loss = 6
  let mission_hectares = 180
  let hectares_per_drone = 18

  let active_ratio = swarm_active_ratio_percent(active_drones, total_drones)
  let average_load = swarm_average_load_percent(total_load, active_drones)
  let communication = swarm_communication_quality_score(
    connected_drones, total_drones, packet_loss,
  )
  let coordination = swarm_coordination_score(
    active_ratio, communication, average_load,
  )
  let capacity = swarm_mission_capacity_hectares(
    active_drones, hectares_per_drone,
  )
  let required = swarm_required_drones(mission_hectares, hectares_per_drone)
  let enough = swarm_has_enough_drones(active_drones, required)
  let status = swarm_status(coordination, enough)
  let rebalance = swarm_should_rebalance(
    average_load, communication, inactive_drones,
  )

  println("Swarm Summary")
  println("active ratio: {active_ratio}%")
  println("average load: {average_load}%")
  println("communication quality: {communication}")
  println("coordination score: {coordination}")
  println("mission capacity hectares: {capacity}")
  println("required drones: {required}")
  println("has enough drones: {enough}")
  println("status: {status}")
  println("should rebalance: {rebalance}")
}

///|
/// A drone in the swarm.
pub struct Drone {
  position : Vector2D
  velocity : Vector2D
  max_speed : Double
}

///|
/// Create a new drone.
pub fn Drone::new(
  position : Vector2D,
  velocity : Vector2D,
  max_speed : Double,
) -> Drone {
  Drone::{ position, velocity, max_speed }
}

///|
/// Limit drone velocity by max_speed while keeping direction unchanged.
pub fn Drone::limit_speed(self : Drone) -> Drone {
  let speed = self.velocity.length()

  if speed > self.max_speed {
    let limited_velocity = self.velocity.normalize().scale(self.max_speed)
    Drone::{
      position: self.position,
      velocity: limited_velocity,
      max_speed: self.max_speed,
    }
  } else {
    self
  }
}

///|
/// Move drone one step using speed-limited velocity.
pub fn Drone::step(self : Drone) -> Drone {
  let drone = self.limit_speed()

  Drone::{
    position: drone.position.add(drone.velocity),
    velocity: drone.velocity,
    max_speed: drone.max_speed,
  }
}

///|
/// A swarm containing multiple drones.
pub struct Swarm {
  drones : Array[Drone]
}

///|
/// Create a new swarm from drones.
pub fn Swarm::new(drones : Array[Drone]) -> Swarm {
  Swarm::{ drones, }
}

///|
/// Move every drone in the swarm one step.
pub fn Swarm::step(self : Swarm) -> Swarm {
  let stepped_drones = self.drones.map(fn(drone) { drone.step() })

  Swarm::{ drones: stepped_drones }
}

///|
/// Get number of drones in the swarm.
pub fn Swarm::size(self : Swarm) -> Int {
  self.drones.length()
}

///|
/// Check whether the swarm has no drones.
pub fn Swarm::is_empty(self : Swarm) -> Bool {
  self.drones.length() == 0
}

///|
/// Add a drone to the swarm.
pub fn Swarm::add_drone(self : Swarm, drone : Drone) -> Swarm {
  let drones = self.drones
  drones.push(drone)

  Swarm::{ drones, }
}

///|
/// Calculate average speed of all drones in the swarm.
pub fn Swarm::average_speed(self : Swarm) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let mut total = 0.0
    let mut count = 0.0

    for drone in self.drones {
      total = total + drone.velocity.length()
      count = count + 1.0
    }

    total / count
  }
}

///|
/// Calculate center position of all drones in the swarm.
pub fn Swarm::center(self : Swarm) -> Vector2D {
  if self.drones.length() == 0 {
    Vector2D::new(0.0, 0.0)
  } else {
    let mut total = Vector2D::new(0.0, 0.0)
    let mut count = 0.0

    for drone in self.drones {
      total = total.add(drone.position)
      count = count + 1.0
    }

    total.scale(1.0 / count)
  }
}

///|
/// Calculate average velocity vector of all drones in the swarm.
pub fn Swarm::average_velocity(self : Swarm) -> Vector2D {
  if self.drones.length() == 0 {
    Vector2D::new(0.0, 0.0)
  } else {
    let mut total = Vector2D::new(0.0, 0.0)
    let mut count = 0.0

    for drone in self.drones {
      total = total.add(drone.velocity)
      count = count + 1.0
    }

    total.scale(1.0 / count)
  }
}

///|
/// Calculate the maximum distance from any drone to the swarm center.
pub fn Swarm::max_distance_from_center(self : Swarm) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let center = self.center()
    let mut max_distance = 0.0

    for drone in self.drones {
      let offset = drone.position.add(center.scale(-1.0))
      let distance = offset.length()

      if distance > max_distance {
        max_distance = distance
      }
    }

    max_distance
  }
}

///|
/// Calculate the average distance from drones to the swarm center.
pub fn Swarm::average_distance_from_center(self : Swarm) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let center = self.center()
    let mut total_distance = 0.0
    let mut count = 0.0

    for drone in self.drones {
      let offset = drone.position.add(center.scale(-1.0))
      total_distance = total_distance + offset.length()
      count = count + 1.0
    }

    total_distance / count
  }
}

///|
/// Check whether the swarm is too spread out.
pub fn Swarm::is_too_spread(self : Swarm, max_radius : Double) -> Bool {
  self.max_distance_from_center() > max_radius
}

///|
/// Calculate cohesion score of the swarm.
///
/// The score is in range approximately from 0.0 to 1.0.
/// A higher score means the swarm is more compact.
pub fn Swarm::cohesion_score(self : Swarm) -> Double {
  let spread = self.average_distance_from_center()
  1.0 / (1.0 + spread)
}

///|
/// Calculate alignment score of the swarm.
///
/// The score is in range approximately from 0.0 to 1.0.
/// A higher score means drones are moving in a more consistent direction.
pub fn Swarm::alignment_score(self : Swarm) -> Double {
  if self.drones.length() == 0 {
    1.0
  } else {
    let avg_speed = self.average_speed()

    if avg_speed == 0.0 {
      1.0
    } else {
      let avg_velocity = self.average_velocity()
      let score = avg_velocity.length() / avg_speed

      if score > 1.0 {
        1.0
      } else {
        score
      }
    }
  }
}

///|
/// Check whether the swarm is sufficiently aligned.
pub fn Swarm::is_aligned(self : Swarm, min_score : Double) -> Bool {
  self.alignment_score() >= min_score
}

///|
/// Count drones within a radius from the target position.
pub fn Swarm::count_within_radius(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Int {
  let mut count = 0

  for drone in self.drones {
    let offset = drone.position.add(target.scale(-1.0))
    let distance = offset.length()

    if distance <= radius {
      count = count + 1
    }
  }

  count
}

///|
/// Check whether any drone is within a radius from the target position.
pub fn Swarm::any_within_radius(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Bool {
  self.count_within_radius(target, radius) > 0
}

///|
/// Check whether all drones are within a radius from the target position.
///
/// Empty swarm returns false because there is no drone to complete the task.
pub fn Swarm::all_within_radius(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Bool {
  if self.drones.length() == 0 {
    false
  } else {
    self.count_within_radius(target, radius) == self.drones.length()
  }
}

///|
/// Calculate the nearest distance from any drone to the target position.
///
/// Empty swarm returns 0.0.
pub fn Swarm::nearest_distance_to(self : Swarm, target : Vector2D) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let mut nearest = 0.0
    let mut first = true

    for drone in self.drones {
      let offset = drone.position.add(target.scale(-1.0))
      let distance = offset.length()

      if first {
        nearest = distance
        first = false
      } else if distance < nearest {
        nearest = distance
      }
    }

    nearest
  }
}

///|
/// Calculate the farthest distance from any drone to the target position.
///
/// Empty swarm returns 0.0.
pub fn Swarm::farthest_distance_to(self : Swarm, target : Vector2D) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let mut farthest = 0.0

    for drone in self.drones {
      let offset = drone.position.add(target.scale(-1.0))
      let distance = offset.length()

      if distance > farthest {
        farthest = distance
      }
    }

    farthest
  }
}

///|
/// Calculate the ratio of drones within a radius from the target position.
///
/// Empty swarm returns 0.0.
pub fn Swarm::arrival_ratio(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    let mut arrived = 0.0
    let mut total = 0.0

    for drone in self.drones {
      let offset = drone.position.add(target.scale(-1.0))
      let distance = offset.length()

      if distance <= radius {
        arrived = arrived + 1.0
      }

      total = total + 1.0
    }

    arrived / total
  }
}

///|
/// Calculate mission progress based on how many drones have arrived near target.
///
/// The result is in range from 0.0 to 1.0.
pub fn Swarm::mission_progress(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Double {
  self.arrival_ratio(target, radius)
}

///|
/// Check whether the mission is complete.
///
/// Mission is complete only when all drones are within the target radius.
pub fn Swarm::is_mission_complete(
  self : Swarm,
  target : Vector2D,
  radius : Double,
) -> Bool {
  self.all_within_radius(target, radius)
}

///|
/// Calculate coverage radius around the target position.
///
/// This is the farthest distance from any drone to the target.
pub fn Swarm::coverage_radius(self : Swarm, target : Vector2D) -> Double {
  self.farthest_distance_to(target)
}

///|
/// Calculate overall formation quality.
///
/// This combines cohesion and alignment.
/// The result is approximately in range from 0.0 to 1.0.
pub fn Swarm::formation_quality(self : Swarm) -> Double {
  let cohesion = self.cohesion_score()
  let alignment = self.alignment_score()

  (cohesion + alignment) * 0.5
}

///|
/// Calculate task readiness score.
///
/// This evaluates whether the swarm is compact and aligned enough for task execution.
pub fn Swarm::task_readiness_score(self : Swarm) -> Double {
  self.formation_quality()
}

///|
/// Check whether the swarm needs regrouping.
///
/// The swarm needs regrouping if it is too spread out or not aligned enough.
pub fn Swarm::needs_regroup(
  self : Swarm,
  max_radius : Double,
  min_alignment : Double,
) -> Bool {
  if self.is_too_spread(max_radius) {
    true
  } else {
    self.alignment_score() < min_alignment
  }
}

///|
/// Check whether the swarm is ready for task execution.
///
/// Empty swarm is not ready.
pub fn Swarm::is_ready_for_task(
  self : Swarm,
  max_radius : Double,
  min_alignment : Double,
) -> Bool {
  if self.drones.length() == 0 {
    false
  } else if self.is_too_spread(max_radius) {
    false
  } else {
    self.alignment_score() >= min_alignment
  }
}

///|
/// Check whether the swarm is operational.
///
/// A swarm is operational when it has at least one drone.
pub fn Swarm::is_operational(self : Swarm) -> Bool {
  self.drones.length() > 0
}

///|
/// Calculate a general swarm status score.
///
/// This score combines:
/// - whether the swarm has drones
/// - formation cohesion
/// - velocity alignment
///
/// Empty swarm returns 0.0.
pub fn Swarm::status_score(self : Swarm) -> Double {
  if self.drones.length() == 0 {
    0.0
  } else {
    self.formation_quality()
  }
}

pub fn swarm_center(drones: Array[Drone]) -> Vector2D { ... }
// ...`,simulation:`// simulation.mbt - 确定性仿真框架
// simulation.mbt - 确定性仿真框架

///|
pub struct DroneState {
  id : Int
  pos_x : Double
  pos_y : Double
  pos_z : Double
  velocity_x : Double
  velocity_y : Double
  velocity_z : Double
  battery : Double
}

///|
pub struct SimEvent {
  time : Double
  drone_id : Int
  event_type : String
  message : String
}

///|
pub struct Simulation {
  time : Double
  drones : Array[DroneState]
  events : Array[SimEvent]
}

///|
pub fn Simulation::new() -> Simulation {
  { time: 0.0, drones: [], events: [] }
}

///|
pub fn Simulation::add_drone(
  self : Simulation,
  drone : DroneState,
) -> Simulation {
  let drones = self.drones
  drones.push(drone)
  { time: self.time, drones, events: self.events }
}

///|
pub fn Simulation::step(self : Simulation, dt : Double) -> Simulation {
  let new_time = self.time + dt
  let new_drones = []
  let mut i = 0
  while i < self.drones.length() {
    let d = self.drones[i]
    let updated = {
      id: d.id,
      pos_x: d.pos_x + d.velocity_x * dt,
      pos_y: d.pos_y + d.velocity_y * dt,
      pos_z: d.pos_z + d.velocity_z * dt,
      velocity_x: d.velocity_x,
      velocity_y: d.velocity_y,
      velocity_z: d.velocity_z,
      battery: d.battery - 0.1 * dt,
    }
    new_drones.push(updated)
    i = i + 1
  }
  { time: new_time, drones: new_drones, events: self.events }
}

///|
pub fn Simulation::log_event(
  self : Simulation,
  drone_id : Int,
  event_type : String,
  message : String,
) -> Simulation {
  let events = self.events
  events.push({ time: self.time, drone_id, event_type, message })
  { time: self.time, drones: self.drones, events }
}

///|
pub fn Simulation::get_drone(self : Simulation, id : Int) -> DroneState? {
  let mut i = 0
  while i < self.drones.length() {
    if self.drones[i].id == id {
      return Some(self.drones[i])
    }
    i = i + 1
  }
  None
}

///|
pub fn Simulation::get_events(self : Simulation) -> Array[SimEvent] {
  self.events
}

///|
pub fn Simulation::get_time(self : Simulation) -> Double {
  self.time
}

pub struct Simulation { time: Double, drones: Array[DroneState] }
// ...`,spray:`// spray.mbt - 喷洒任务辅助
///|
/// Spray module for AgriDroneSwarm.
/// Estimate spray mode, liquid usage, nozzle flow, weather safety,
/// coverage, refill requirement, and spray quality.

///|
/// Spray mode: off.
pub fn spray_mode_off() -> Int {
  0
}

///|
/// Spray mode: low.
pub fn spray_mode_low() -> Int {
  1
}

///|
/// Spray mode: normal.
pub fn spray_mode_normal() -> Int {
  2
}

///|
/// Spray mode: high.
pub fn spray_mode_high() -> Int {
  3
}

///|
/// Spray mode name.
pub fn spray_mode_name(mode : Int) -> String {
  match mode {
    0 => "off"
    1 => "low"
    2 => "normal"
    3 => "high"
    _ => "unknown"
  }
}

///|
/// Nozzle type: mist.
pub fn spray_nozzle_mist() -> Int {
  1
}

///|
/// Nozzle type: fan.
pub fn spray_nozzle_fan() -> Int {
  2
}

///|
/// Nozzle type: cone.
pub fn spray_nozzle_cone() -> Int {
  3
}

///|
/// Nozzle type name.
pub fn spray_nozzle_name(nozzle : Int) -> String {
  match nozzle {
    1 => "mist"
    2 => "fan"
    3 => "cone"
    _ => "unknown"
  }
}

///|
/// Clamp non-negative integer.
pub fn spray_clamp_non_negative(value : Int) -> Int {
  if value < 0 {
    0
  } else {
    value
  }
}

///|
/// Clamp percent to 0..100.
pub fn spray_clamp_percent(value : Int) -> Int {
  if value < 0 {
    0
  } else if value > 100 {
    100
  } else {
    value
  }
}

///|
/// Estimate base liquid flow per minute.
pub fn spray_base_flow_liter_per_minute(mode : Int) -> Int {
  if mode == spray_mode_off() {
    0
  } else if mode == spray_mode_low() {
    1
  } else if mode == spray_mode_normal() {
    2
  } else if mode == spray_mode_high() {
    3
  } else {
    1
  }
}

///|
/// Estimate nozzle flow bonus.
pub fn spray_nozzle_flow_bonus(nozzle : Int) -> Int {
  if nozzle == spray_nozzle_mist() {
    0
  } else if nozzle == spray_nozzle_fan() {
    1
  } else if nozzle == spray_nozzle_cone() {
    2
  } else {
    0
  }
}

///|
/// Estimate total liquid flow per minute.
pub fn spray_flow_liter_per_minute(mode : Int, nozzle : Int) -> Int {
  spray_base_flow_liter_per_minute(mode) + spray_nozzle_flow_bonus(nozzle)
}

///|
/// Estimate spray width in meters.
pub fn spray_width_meters(nozzle : Int, altitude_meters : Int) -> Int {
  let altitude = spray_clamp_non_negative(altitude_meters)
  let base = if nozzle == spray_nozzle_mist() {
    3
  } else if nozzle == spray_nozzle_fan() {
    5
  } else if nozzle == spray_nozzle_cone() {
    4
  } else {
    3
  }
  base + altitude / 3
}

///|
/// Estimate recommended spray speed.
pub fn spray_recommended_speed_mps(mode : Int, wind_speed : Int) -> Int {
  let wind = spray_clamp_non_negative(wind_speed)
  let base = if mode == spray_mode_low() {
    7
  } else if mode == spray_mode_normal() {
    5
  } else if mode == spray_mode_high() {
    4
  } else {
    6
  }

  if wind >= 10 {
    if base > 2 {
      base - 2
    } else {
      1
    }
  } else if wind >= 6 {
    if base > 1 {
      base - 1
    } else {
      1
    }
  } else {
    base
  }
}

///|
/// Check whether wind is safe for spraying.
pub fn spray_wind_safe(wind_speed : Int) -> Bool {
  spray_clamp_non_negative(wind_speed) <= 10
}

///|
/// Check whether temperature is safe for spraying.
pub fn spray_temperature_safe(temperature_c : Int) -> Bool {
  temperature_c >= 5 && temperature_c <= 35
}

///|
/// Check whether humidity is safe for spraying.
pub fn spray_humidity_safe(humidity_percent : Int) -> Bool {
  let humidity = spray_clamp_percent(humidity_percent)
  humidity >= 30 && humidity <= 90
}

///|
/// Check whether weather is safe for spraying.
pub fn spray_weather_safe(
  wind_speed : Int,
  temperature_c : Int,
  humidity_percent : Int,
) -> Bool {
  spray_wind_safe(wind_speed) &&
  spray_temperature_safe(temperature_c) &&
  spray_humidity_safe(humidity_percent)
}

///|
/// Estimate liquid required for area.
pub fn spray_liquid_required_liter(
  area_square_meter : Int,
  dose_liter_per_1000_square_meter : Int,
) -> Int {
  let area = spray_clamp_non_negative(area_square_meter)
  let dose = spray_clamp_non_negative(dose_liter_per_1000_square_meter)
  area * dose / 1000
}

///|
/// Estimate spray duration in minutes.
pub fn spray_duration_minutes(
  liquid_liter : Int,
  mode : Int,
  nozzle : Int,
) -> Int {
  let liquid = spray_clamp_non_negative(liquid_liter)
  let flow = spray_flow_liter_per_minute(mode, nozzle)
  if flow <= 0 {
    0
  } else {
    (liquid + flow - 1) / flow
  }
}

///|
/// Estimate area covered per minute.
pub fn spray_area_per_minute(speed_mps : Int, width_meters : Int) -> Int {
  let speed = spray_clamp_non_negative(speed_mps)
  let width = spray_clamp_non_negative(width_meters)
  speed * width * 60
}

///|
/// Estimate coverage percent.
pub fn spray_coverage_percent(sprayed_area : Int, field_area : Int) -> Int {
  let sprayed = spray_clamp_non_negative(sprayed_area)
  let field = spray_clamp_non_negative(field_area)
  if field <= 0 {
    0
  } else {
    spray_clamp_percent(sprayed * 100 / field)
  }
}

///|
/// Estimate remaining liquid after spraying.
pub fn spray_remaining_liter(tank_liter : Int, used_liter : Int) -> Int {
  let tank = spray_clamp_non_negative(tank_liter)
  let used = spray_clamp_non_negative(used_liter)
  if used >= tank {
    0
  } else {
    tank - used
  }
}

///|
/// Check whether tank has enough liquid.
pub fn spray_tank_enough(tank_liter : Int, required_liter : Int) -> Bool {
  spray_clamp_non_negative(tank_liter) >=
  spray_clamp_non_negative(required_liter)
}

///|
/// Estimate refill requirement.
pub fn spray_refill_required(tank_liter : Int, required_liter : Int) -> Bool {
  !spray_tank_enough(tank_liter, required_liter)
}

///|
/// Estimate refill amount.
pub fn spray_refill_amount_liter(tank_liter : Int, required_liter : Int) -> Int {
  let tank = spray_clamp_non_negative(tank_liter)
  let required = spray_clamp_non_negative(required_liter)
  if required > tank {
    required - tank
  } else {
    0
  }
}

///|
/// Estimate spray quality score.
pub fn spray_quality_score(
  wind_speed : Int,
  temperature_c : Int,
  humidity_percent : Int,
  mode : Int,
) -> Int {
  let mut score = 100

  if !spray_wind_safe(wind_speed) {
    score = score - 30
  } else if wind_speed >= 7 {
    score = score - 10
  }

  if !spray_temperature_safe(temperature_c) {
    score = score - 20
  }

  if !spray_humidity_safe(humidity_percent) {
    score = score - 15
  }

  if mode == spray_mode_high() {
    score = score - 5
  }

  spray_clamp_percent(score)
}

///|
/// Convert spray quality score to level.
pub fn spray_quality_level(score : Int) -> String {
  let safe_score = spray_clamp_percent(score)
  if safe_score >= 85 {
    "excellent"
  } else if safe_score >= 70 {
    "good"
  } else if safe_score >= 50 {
    "limited"
  } else {
    "unsafe"
  }
}

///|
/// Check whether spray mission can start.
pub fn spray_can_start(
  tank_liter : Int,
  required_liter : Int,
  wind_speed : Int,
  temperature_c : Int,
  humidity_percent : Int,
) -> Bool {
  spray_tank_enough(tank_liter, required_liter) &&
  spray_weather_safe(wind_speed, temperature_c, humidity_percent)
}

///|
/// Print spray module summary.
pub fn spray_summary() -> Unit {
  let field_area = 12000
  let dose = 6
  let tank = 90
  let mode = spray_mode_normal()
  let nozzle = spray_nozzle_fan()
  let altitude = 6
  let wind = 7
  let temperature = 29
  let humidity = 62

  let required = spray_liquid_required_liter(field_area, dose)
  let flow = spray_flow_liter_per_minute(mode, nozzle)
  let duration = spray_duration_minutes(required, mode, nozzle)
  let width = spray_width_meters(nozzle, altitude)
  let speed = spray_recommended_speed_mps(mode, wind)
  let area_per_minute = spray_area_per_minute(speed, width)
  let sprayed_area = area_per_minute * duration
  let coverage = spray_coverage_percent(sprayed_area, field_area)
  let remaining = spray_remaining_liter(tank, required)
  let refill = spray_refill_required(tank, required)
  let refill_amount = spray_refill_amount_liter(tank, required)
  let weather_ok = spray_weather_safe(wind, temperature, humidity)
  let quality = spray_quality_score(wind, temperature, humidity, mode)
  let can_start = spray_can_start(tank, required, wind, temperature, humidity)

  println("Spray mode: " + spray_mode_name(mode))
  println("Spray nozzle: " + spray_nozzle_name(nozzle))
  println("Spray field area: " + field_area.to_string() + " m2")
  println("Spray required liquid: " + required.to_string() + " L")
  println("Spray tank liquid: " + tank.to_string() + " L")
  println("Spray flow: " + flow.to_string() + " L/min")
  println("Spray duration: " + duration.to_string() + " min")
  println("Spray width: " + width.to_string() + " m")
  println("Spray speed: " + speed.to_string() + " m/s")
  println("Spray area per minute: " + area_per_minute.to_string() + " m2")
  println("Spray estimated coverage: " + coverage.to_string() + "%")
  println("Spray remaining liquid: " + remaining.to_string() + " L")
  println("Spray refill required: " + refill.to_string())
  println("Spray refill amount: " + refill_amount.to_string() + " L")
  println("Spray weather safe: " + weather_ok.to_string())
  println("Spray quality score: " + quality.to_string())
  println("Spray quality level: " + spray_quality_level(quality))
  println("Spray can start: " + can_start.to_string())
}

pub fn spray_weather_safe(wind: Int, temp: Int, humidity: Int) -> Bool { ... }
// ...`,pid_test:`// pid_test.mbt - PID 控制器测试
///|
test "pid_basic" {
  let pid = PID::new(1.0, 0.1, 0.05)
  let pid = PID::set_setpoint(pid, 10.0)
  let out = PID::update(pid, 0.0, 0.1)
  inspect(out > 0.0, content="true")
}

test "pid_basic" { ... }
// ...`},t=document.getElementById(`codePopup`),n=document.getElementById(`overlay`),r=document.getElementById(`popupTitle`),i=document.getElementById(`codeContent`),a=document.getElementById(`popupClose`);document.querySelectorAll(`.card-wrapper`).forEach(a=>{a.addEventListener(`click`,o=>{let s=a.dataset.code,c=e[s]||`// 代码尚未提供，请编辑 codeMap 中的对应条目`;r.innerText=a.querySelector(`.card h3`)?.innerText||s,i.innerText=c,t.classList.add(`active`),n.classList.add(`active`)})});function o(){t.classList.remove(`active`),n.classList.remove(`active`)}a.addEventListener(`click`,o),n.addEventListener(`click`,o),document.addEventListener(`keydown`,e=>{e.key===`Escape`&&o()});