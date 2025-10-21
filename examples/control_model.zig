const std = @import("std");
const zmj = @import("zmujoco");
const c = zmj.c;
const mjtNum = zmj.mjtNum;

// MuJoCo data structures
var m: ?*c.mjModel = null;
var d: ?*c.mjData = null;
var cam: c.mjvCamera = .{};
var vopt: c.mjvOption = .{};
var pert: c.mjvPerturb = .{};
var scn: c.mjvScene = .{};
var con: c.mjrContext = .{};

// Mouse state
var button_left = false;
var button_middle = false;
var button_right = false;
var lastx: f64 = 0;
var lasty: f64 = 0;
var paused = false;

// Control state
var control_mode: enum { JointControl, Preset, Reaching, Wave } = .JointControl;
var target_pos = [2]f64{ 0.5, 0.3 }; // Target position in 2D space
var time: f64 = 0.0;

// Joint control targets
var target_shoulder_angle: f64 = 0.0;
var target_elbow_angle: f64 = 0.0;
const joint_control_speed: f64 = 1.0; // radians per second

// Manual muscle activations (0-1 for each of 6 muscles)
var manual_activations = [6]f64{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

// Preset poses
const Pose = struct {
    name: []const u8,
    shoulder_angle: f64,
    elbow_angle: f64,
};

const presets = [_]Pose{
    .{ .name = "Rest", .shoulder_angle = 0.0, .elbow_angle = 0.0 },
    .{ .name = "Reach Forward", .shoulder_angle = std.math.pi / 4.0, .elbow_angle = 0.0 },
    .{ .name = "Reach Up", .shoulder_angle = std.math.pi / 2.0, .elbow_angle = 0.0 },
    .{ .name = "Bent Elbow", .shoulder_angle = std.math.pi / 4.0, .elbow_angle = std.math.pi / 3.0 },
    .{ .name = "Wave Position", .shoulder_angle = std.math.pi / 2.0, .elbow_angle = std.math.pi / 2.0 },
};

var current_preset: usize = 0;

// Saved custom poses (up to 5)
var saved_poses: [5]?Pose = [_]?Pose{null} ** 5;

// GLFW callbacks
fn keyboard(window: *zmj.glfw.Window, key: zmj.glfw.Key, scancode: c_int, act: zmj.glfw.Action, mods: zmj.glfw.Mods) callconv(.c) void {
    _ = window;
    _ = scancode;

    if (act == zmj.glfw.Action.press) {
        switch (key) {
            zmj.glfw.Key.space => paused = !paused,
            zmj.glfw.Key.backspace => {
                c.mj_resetData(m.?, d.?);
                c.mj_forward(m.?, d.?);
                time = 0.0;
                target_shoulder_angle = 0.0;
                target_elbow_angle = 0.0;
            },
            // Control mode switching
            zmj.glfw.Key.one => control_mode = .JointControl,
            zmj.glfw.Key.two => control_mode = .Preset,
            zmj.glfw.Key.three => control_mode = .Reaching,
            zmj.glfw.Key.four => control_mode = .Wave,
            
            // Joint control mode (active in mode 1)
            zmj.glfw.Key.w => {
                if (control_mode == .JointControl) {
                    target_shoulder_angle += 0.1;
                }
            },
            zmj.glfw.Key.s => {
                if (control_mode == .JointControl) {
                    target_shoulder_angle -= 0.1;
                }
            },
            zmj.glfw.Key.up => {
                if (control_mode == .JointControl) {
                    target_elbow_angle += 0.1;
                }
            },
            zmj.glfw.Key.down => {
                if (control_mode == .JointControl) {
                    target_elbow_angle -= 0.1;
                }
            },
            
            // Preset mode (active in mode 2)
            zmj.glfw.Key.left => {
                if (control_mode == .Preset) {
                    if (current_preset > 0) {
                        current_preset -= 1;
                    } else {
                        current_preset = presets.len - 1;
                    }
                    applyPreset(presets[current_preset]);
                }
            },
            zmj.glfw.Key.right => {
                if (control_mode == .Preset) {
                    current_preset = (current_preset + 1) % presets.len;
                    applyPreset(presets[current_preset]);
                }
            },
            
            // Save/Load custom poses (Ctrl+1-5 to save, 5-9 to load)
            zmj.glfw.Key.five => {
                if (mods.shift) {
                    saveCustomPose(0);
                } else if (control_mode == .Preset) {
                    loadCustomPose(0);
                }
            },
            zmj.glfw.Key.six => {
                if (mods.shift) {
                    saveCustomPose(1);
                } else if (control_mode == .Preset) {
                    loadCustomPose(1);
                }
            },
            zmj.glfw.Key.seven => {
                if (mods.shift) {
                    saveCustomPose(2);
                } else if (control_mode == .Preset) {
                    loadCustomPose(2);
                }
            },
            zmj.glfw.Key.eight => {
                if (mods.shift) {
                    saveCustomPose(3);
                } else if (control_mode == .Preset) {
                    loadCustomPose(3);
                }
            },
            zmj.glfw.Key.nine => {
                if (mods.shift) {
                    saveCustomPose(4);
                } else if (control_mode == .Preset) {
                    loadCustomPose(4);
                }
            },
            
            else => {},
        }
    }
}

fn mouse_button(window: *zmj.glfw.Window, button: zmj.glfw.MouseButton, act: zmj.glfw.Action, mods: zmj.glfw.Mods) callconv(.c) void {
    _ = mods;
    _ = button;
    _ = act;

    button_left = (zmj.glfw.getMouseButton(window, zmj.glfw.MouseButton.left) == zmj.glfw.Action.press);
    button_middle = (zmj.glfw.getMouseButton(window, zmj.glfw.MouseButton.middle) == zmj.glfw.Action.press);
    button_right = (zmj.glfw.getMouseButton(window, zmj.glfw.MouseButton.right) == zmj.glfw.Action.press);

    zmj.glfw.getCursorPos(window, &lastx, &lasty);
}

fn mouse_move(window: *zmj.glfw.Window, xpos: f64, ypos: f64) callconv(.c) void {
    if (!button_left and !button_middle and !button_right) {
        return;
    }

    const dx = xpos - lastx;
    const dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    var width: c_int = 0;
    var height: c_int = 0;
    zmj.glfw.getWindowSize(window, &width, &height);

    if (button_right) {
        c.mjv_moveCamera(m.?, c.mjMOUSE_MOVE_H, dx / @as(f64, @floatFromInt(width)), dy / @as(f64, @floatFromInt(height)), &scn, &cam);
    } else if (button_left) {
        c.mjv_moveCamera(m.?, c.mjMOUSE_ROTATE_H, dx / @as(f64, @floatFromInt(width)), dy / @as(f64, @floatFromInt(height)), &scn, &cam);
    }
}

fn scroll(window: *zmj.glfw.Window, xoffset: f64, yoffset: f64) callconv(.c) void {
    _ = window;
    _ = xoffset;
    c.mjv_moveCamera(m.?, c.mjMOUSE_ZOOM, 0, yoffset, &scn, &cam);
}

// Apply a preset pose
fn applyPreset(pose: Pose) void {
    target_shoulder_angle = pose.shoulder_angle;
    target_elbow_angle = pose.elbow_angle;
    std.log.info("Applied preset: {s}", .{pose.name});
}

// Save current joint angles as custom pose
fn saveCustomPose(slot: usize) void {
    const current_shoulder = d.?.qpos[0];
    const current_elbow = d.?.qpos[1];
    
    var name_buf: [32]u8 = undefined;
    const name = std.fmt.bufPrint(&name_buf, "Custom {d}", .{slot + 1}) catch "Custom";
    
    saved_poses[slot] = Pose{
        .name = name,
        .shoulder_angle = current_shoulder,
        .elbow_angle = current_elbow,
    };
    std.log.info("Saved pose to slot {d}: shoulder={d:.2}, elbow={d:.2}", .{ slot + 1, current_shoulder, current_elbow });
}

// Load a custom pose
fn loadCustomPose(slot: usize) void {
    if (saved_poses[slot]) |pose| {
        applyPreset(pose);
        std.log.info("Loaded custom pose {d}", .{slot + 1});
    } else {
        std.log.info("No pose saved in slot {d}", .{slot + 1});
    }
}

// Get end-effector position (tip of forearm)
fn getEndEffectorPos() [2]f64 {
    const shoulder_angle = d.?.qpos[0];
    const elbow_angle = d.?.qpos[1];
    
    const upper_len = 0.5; // Length of upper arm
    const fore_len = 0.5; // Length of forearm
    
    const x = upper_len * @cos(shoulder_angle) + fore_len * @cos(shoulder_angle + elbow_angle);
    const y = upper_len * @sin(shoulder_angle) + fore_len * @sin(shoulder_angle + elbow_angle);
    
    return .{ x, y };
}

// PD controller to move joints to target angles using muscles
fn jointPDController() void {
    const current_shoulder = d.?.qpos[0];
    const current_elbow = d.?.qpos[1];
    
    const shoulder_error = target_shoulder_angle - current_shoulder;
    const elbow_error = target_elbow_angle - current_elbow;
    
    const kp = 5.0;
    const kd = 0.5;
    
    const shoulder_vel = d.?.qvel[0];
    const elbow_vel = d.?.qvel[1];
    
    // Clear all activations
    for (&manual_activations) |*activation| {
        activation.* = 0.0;
    }
    
    // Shoulder control
    const shoulder_control = kp * shoulder_error - kd * shoulder_vel;
    if (shoulder_control > 0) {
        manual_activations[0] = @min(1.0, shoulder_control); // SF
    } else {
        manual_activations[1] = @min(1.0, -shoulder_control); // SE
    }
    
    // Elbow control
    const elbow_control = kp * elbow_error - kd * elbow_vel;
    if (elbow_control > 0) {
        manual_activations[2] = @min(1.0, elbow_control); // EF
    } else {
        manual_activations[3] = @min(1.0, -elbow_control); // EE
    }
}

// Simple reaching controller
fn reachingController() void {
    const end_pos = getEndEffectorPos();
    const error_x = target_pos[0] - end_pos[0];
    const error_y = target_pos[1] - end_pos[1];
    
    const kp = 2.0;
    
    for (&manual_activations) |*activation| {
        activation.* = 0.0;
    }
    
    if (error_x > 0.05) {
        manual_activations[0] = @min(1.0, kp * error_x);
    } else if (error_x < -0.05) {
        manual_activations[1] = @min(1.0, -kp * error_x);
    }
    
    if (error_y > 0.05) {
        manual_activations[2] = @min(1.0, kp * error_y);
    } else if (error_y < -0.05) {
        manual_activations[3] = @min(1.0, -kp * error_y);
    }
}

// Cyclic wave motion
fn waveController() void {
    const freq = 1.0;
    const phase = 2.0 * std.math.pi * freq * time;
    
    manual_activations[0] = 0.3 + 0.3 * @sin(phase);
    manual_activations[1] = 0.3 - 0.3 * @sin(phase);
    manual_activations[2] = 0.3 + 0.3 * @sin(phase + std.math.pi / 2.0);
    manual_activations[3] = 0.3 - 0.3 * @sin(phase + std.math.pi / 2.0);
    manual_activations[4] = 0.2;
    manual_activations[5] = 0.2;
}

// Apply control based on current mode
fn applyControl() void {
    switch (control_mode) {
        .JointControl, .Preset => {
            jointPDController();
        },
        .Reaching => {
            reachingController();
        },
        .Wave => {
            waveController();
        },
    }
    
    // Set muscle activations in MuJoCo
    for (manual_activations, 0..) |activation, i| {
        d.?.ctrl[i] = activation;
    }
}

pub fn main() !void {
    // Load model
    const model_path = "libs/mujoco/model/tendon_arm/arm26.xml";
    var error_buf: [1024]u8 = undefined;
    m = c.mj_loadXML(model_path, null, &error_buf, error_buf.len);
    if (m == null) {
        std.log.err("Failed to load model: {s}", .{error_buf});
        return;
    }
    defer c.mj_deleteModel(m.?);

    d = c.mj_makeData(m.?);
    if (d == null) {
        std.log.err("Failed to make data", .{});
        return;
    }
    defer c.mj_deleteData(d.?);

    try zmj.glfw.init();
    const window = try zmj.glfw.createWindow(1200, 800, "Arm Controller", null);
    defer zmj.glfw.terminate();
    zmj.glfw.makeContextCurrent(window);
    zmj.glfw.swapInterval(1);

    c.mjv_defaultCamera(&cam);
    c.mjv_defaultOption(&vopt);
    c.mjv_defaultScene(&scn);
    c.mjr_defaultContext(&con);

    c.mjv_makeScene(m.?, &scn, 2000);
    c.mjr_makeContext(m.?, &con, c.mjFONTSCALE_150);
    defer c.mjv_freeScene(&scn);
    defer c.mjr_freeContext(&con);

    cam.azimuth = 90.0;
    cam.elevation = -20.0;
    cam.distance = 2.5;
    cam.lookat[0] = 0.5;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.0;

    vopt.flags[c.mjVIS_TENDON] = 1;

    _ = zmj.glfw.setKeyCallback(window, keyboard);
    _ = zmj.glfw.setMouseButtonCallback(window, mouse_button);
    _ = zmj.glfw.setCursorPosCallback(window, mouse_move);
    _ = zmj.glfw.setScrollCallback(window, scroll);

    std.log.info("=== Arm Controller ===", .{});
    std.log.info("Control Modes:", .{});
    std.log.info("  1 - Joint Control (direct)", .{});
    std.log.info("  2 - Preset Poses", .{});
    std.log.info("  3 - Reaching controller", .{});
    std.log.info("  4 - Wave motion", .{});
    std.log.info("\nJoint Control (Mode 1):", .{});
    std.log.info("  W/S - Shoulder up/down", .{});
    std.log.info("  Up/Down - Elbow flex/extend", .{});
    std.log.info("\nPreset Mode (Mode 2):", .{});
    std.log.info("  Left/Right - Cycle through presets", .{});
    std.log.info("  Shift+5-9 - Save current pose", .{});
    std.log.info("  5-9 - Load saved pose", .{});
    std.log.info("\nOther:", .{});
    std.log.info("  Space - Pause/Resume", .{});
    std.log.info("  Backspace - Reset", .{});

    while (!zmj.glfw.windowShouldClose(window)) {
        const simstart = zmj.glfw.getTime();

        if (!paused) {
            applyControl();
            c.mj_step(m.?, d.?);
            time += m.?.opt.timestep;
        }

        var viewport = c.mjrRect{ .left = 0, .bottom = 0, .width = 0, .height = 0 };
        zmj.glfw.getFramebufferSize(window, &viewport.width, &viewport.height);

        c.mjv_updateScene(m.?, d.?, &vopt, &pert, &cam, c.mjCAT_ALL, &scn);
        c.mjr_setBuffer(c.mjFB_WINDOW, &con);
        c.mjr_render(viewport, &scn, &con);

        // Render HUD
        var overlay_buf: [512]u8 = undefined;
        const mode_str = switch (control_mode) {
            .JointControl => "JOINT CONTROL",
            .Preset => "PRESET",
            .Reaching => "REACHING",
            .Wave => "WAVE",
        };
        
        const overlay = if (control_mode == .Preset) blk: {
            break :blk std.fmt.bufPrintZ(&overlay_buf, 
                "Mode: {s}\nPreset: {s}\nShoulder: {d:.2} rad\nElbow: {d:.2} rad\nMuscles: SF={d:.2} SE={d:.2} EF={d:.2} EE={d:.2}", 
                .{ mode_str, presets[current_preset].name, d.?.qpos[0], d.?.qpos[1], 
                   manual_activations[0], manual_activations[1], manual_activations[2], manual_activations[3] }
            ) catch "Error";
        } else blk: {
            break :blk std.fmt.bufPrintZ(&overlay_buf, 
                "Mode: {s}\nTarget Shoulder: {d:.2} rad\nTarget Elbow: {d:.2} rad\nActual: S={d:.2} E={d:.2}\nMuscles: SF={d:.2} SE={d:.2} EF={d:.2} EE={d:.2}", 
                .{ mode_str, target_shoulder_angle, target_elbow_angle, d.?.qpos[0], d.?.qpos[1],
                   manual_activations[0], manual_activations[1], manual_activations[2], manual_activations[3] }
            ) catch "Error";
        };
        
        c.mjr_overlay(c.mjFONT_NORMAL, c.mjGRID_TOPLEFT, viewport, overlay.ptr, null, &con);

        zmj.glfw.swapBuffers(window);
        zmj.glfw.pollEvents();

        const simend = zmj.glfw.getTime();
        const looptime = simend - simstart;
        const sleeptime = m.?.opt.timestep - looptime;
        if (sleeptime > 0) {
            std.Thread.sleep(@as(u64, @intFromFloat(sleeptime * 1e9)));
        }
    }
}
