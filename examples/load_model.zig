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

// GLFW callbacks
fn keyboard(window: *zmj.glfw.Window, key: zmj.glfw.Key, scancode: c_int, act: zmj.glfw.Action, mods: zmj.glfw.Mods) callconv(.c) void {
    _ = window;
    _ = scancode;
    _ = mods;

    if (act == zmj.glfw.Action.press) {
        switch (key) {
            zmj.glfw.Key.space => paused = !paused,
            zmj.glfw.Key.backspace => {
                c.mj_resetData(m.?, d.?);
                c.mj_forward(m.?, d.?);
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

    // Make data
    d = c.mj_makeData(m.?);
    if (d == null) {
        std.log.err("Failed to make data", .{});
        return;
    }
    defer c.mj_deleteData(d.?);

    // Init GLFW
    try zmj.glfw.init();
    const window = try zmj.glfw.createWindow(1200, 800, "flybody_viewer", null);
    defer zmj.glfw.terminate();
    zmj.glfw.makeContextCurrent(window);
    zmj.glfw.swapInterval(1);

    // Init MuJoCo visualization
    c.mjv_defaultCamera(&cam);
    c.mjv_defaultOption(&vopt);
    c.mjv_defaultScene(&scn);
    c.mjr_defaultContext(&con);

    c.mjv_makeScene(m.?, &scn, 2000);
    c.mjr_makeContext(m.?, &con, c.mjFONTSCALE_150);
    defer c.mjv_freeScene(&scn);
    defer c.mjr_freeContext(&con);

    // Set callbacks
    _ = zmj.glfw.setKeyCallback(window, keyboard);
    _ = zmj.glfw.setMouseButtonCallback(window, mouse_button);
    _ = zmj.glfw.setCursorPosCallback(window, mouse_move);
    _ = zmj.glfw.setScrollCallback(window, scroll);

    // Main loop
    while (!zmj.glfw.windowShouldClose(window)) {
        const simstart = zmj.glfw.getTime();

        if (!paused) {
            c.mj_step(m.?, d.?);
        }

        // get framebuffer viewport
        var viewport = c.mjrRect{ .left = 0, .bottom = 0, .width = 0, .height = 0 };
        zmj.glfw.getFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        c.mjv_updateScene(m.?, d.?, &vopt, &pert, &cam, c.mjCAT_ALL, &scn);
        c.mjr_setBuffer(c.mjFB_WINDOW, &con);
        c.mjr_render(viewport, &scn, &con);

        // swap buffers
        zmj.glfw.swapBuffers(window);

        // poll events
        zmj.glfw.pollEvents();

        // sync to real time
        const simend = zmj.glfw.getTime();
        const looptime = simend - simstart;
        const sleeptime = m.?.opt.timestep - looptime;
        if (sleeptime > 0) {
            std.Thread.sleep(@as(u64, @intFromFloat(sleeptime * 1e9)));
        }
    }
}
