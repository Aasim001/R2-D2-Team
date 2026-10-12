// ============================================================================
//  Compact pickup robot  -  stacked chassis + side scissor arm + one-way flap
//  4x N20 encoder motors, 2x TB6612 + ESP32 on a 70x90 perfboard, 2S battery,
//  buck converter, MG90S servo, ESP32-CAM, AprilTag top plate.
//
//  Units mm.  z = 0 is the FLOOR.  x = across the robot (pickup side is +x),
//  y = along the robot.  Measure your parts and edit the values below.
//
//  part = "assembly" | "assembly_carry" | "assembly_reach" |
//         "lower" | "deck" | "top" | "cam_cradle" | "link" | "link_servo" |
//         "tip_block" | "flap" | "spacer_10" | "spacer_4" | "wheel"
// ============================================================================
part = "assembly";
fit = 0.3;
$fn = 40;

/* ---------- wheels + N20 ---------- */
wheel_d = 34;  wheel_w = 12;  shaft_d = 3;  shaft_flat = 2.5;
gear_w = 12; gear_h = 10; gear_l = 9; can_l = 15; enc_l = 12; enc_w = 18;
shaft_hole = 5; m16_sp = 9; m16_d = 1.9;
axle_z = wheel_d/2;
axle_y = [17.5, 112.5];

/* ---------- body ---------- */
BX = [14, 124];            // lower plate x-range (outer faces of the motor walls)
L  = 130;
wall = 2.5;  floor_t = 2;
plate_z   = axle_z - gear_h/2 - fit/2 - floor_t;    // underside of the lower plate
plate_top = plate_z + floor_t;
deck_z = 76;  deck_t = 3;                           // middle deck
top_z  = deck_z + deck_t + 60;  top_t = 2.5;        // AprilTag plate
post_xy   = [[18, 30], [118, 28], [18, 100], [118, 100]];   // lower plate -> deck
pillar_xy = [[19, 6], [119, 6], [19, 124], [119, 124]];     // deck -> top plate

/* ---------- side panel the cube rests on ---------- */
panel_x = 138;  panel_y = [39, 91];  panel_top = 30;  panel_t = 3;

/* ---------- battery + buck (lower plate) ---------- */
bat  = [77, 41];    bat_c  = [69, 60];   bat_h = 21;
buck = [43.5, 21.5]; buck_c = [69, 92];

/* ---------- deck items ---------- */
perf = [70, 90];  perf_c = [49.5, 65];  perf_in = 3.5;  perf_stand = 8;
switch_cut = [19.2, 12.9];  switch_c = [40, 12];
mg90 = [12.2, 22.8];  mg90_tab = 27.8;     // body along y
servo_shaft = [98, 46];
spline_drop = 12.5;      // MG90S spline tip below its mounting tabs
spline_d = 4.7;

/* ---------- scissor arm ---------- */
N = 4;  Lk = 38;  link_w = 7;  link_t = 3;  pin_d = 3.3;
xb = servo_shaft[0];  y0 = servo_shaft[1];
A_z = deck_z + deck_t - spline_drop;      // underside of top link layer
B_z = A_z - 0.5 - link_t;                 // underside of bottom link layer
blk_t = 4;  blk_z = B_z - 0.5 - blk_t;    // tip block
hinge_off = 8;  hinge_z = 56;  y_f = 65;  // flap hinge (x from tip pivot), height, centre
stow_len = 32;  theta_min = 23;
function th(len) = acos(len/(N*Lk));
th_stow  = th(stow_len);
w_max = Lk*sin(th_stow);  w_min = Lk*sin(theta_min);

/* ---------- flap ---------- */
flap_len = 24; flap_tilt = 15; flap_t = 2.5; cube_s = 50;
flap_w = cube_s + 6;  finger_l = 12;  finger_t = 2.5;
knuck_r = 2.3; knuck_l = 20; lug_t = 4; lug_r = 3.5;

/* ---------- camera ---------- */
cam_p = [115, 62, 117];  cam_tilt = 35;  cam_board = [27, 40.5];  cam_stand = 10;
cam_n = [cos(cam_tilt), 0, -sin(cam_tilt)];   // lens direction
cam_t = [sin(cam_tilt), 0,  cos(cam_tilt)];   // board long axis

// ============================================================================
module rr(x0, y0, x1, y1, r) { translate([x0 + r, y0 + r]) offset(r = r) square([x1 - x0 - 2*r, y1 - y0 - 2*r]); }
module mirror_x() { children(); translate([BX[0] + BX[1], 0, 0]) mirror([1, 0, 0]) children(); }

// ---------------------------------------------------------------- LOWER PLATE
module cradle_solid(y) {
    translate([BX[0], y - 11, plate_z]) cube([wall, 22, axle_z + 8 - plate_z]);
    for (s = [-1, 1]) translate([BX[0] + wall, y + s*(gear_w/2 + fit/2 + 1) - 1, plate_z])
        cube([gear_l + can_l - 2, 2, plate_top + gear_h + fit - plate_z]);
}
module cradle_cuts(y) {
    translate([BX[0] - 1, y, axle_z]) rotate([0, 90, 0]) cylinder(d = shaft_hole, h = wall + 2);
    for (s = [-1, 1]) translate([BX[0] - 1, y + s*m16_sp/2, axle_z]) rotate([0, 90, 0]) cylinder(d = m16_d, h = wall + 2);
    for (s = [-1, 1]) translate([BX[0] + wall + gear_l + can_l/2 - 2.25, y + s*(gear_w/2 + 3.15) - 1, plate_z - 1]) cube([4.5, 2, floor_t + 2]);
    translate([BX[0] + wall + gear_l + can_l - 1, y - enc_w/2, plate_z - 1]) cube([enc_l + 1, enc_w, floor_t + 2]);
}
module guides(sz, c, h = 4) {
    x0 = c[0] - sz[0]/2 - fit; x1 = c[0] + sz[0]/2 + fit;
    y0_ = c[1] - sz[1]/2 - fit; y1 = c[1] + sz[1]/2 + fit;
    for (x = [x0, x1], y = [y0_, y1]) {
        sx = x == x0 ? -1 : 1; sy = y == y0_ ? -1 : 1;
        translate([sx < 0 ? x - 1.5 : x, sy < 0 ? y - 1.5 : y - 5, plate_top - 0.01]) cube([1.5, 6.5, h]);
        translate([sx < 0 ? x - 1.5 : x - 5, sy < 0 ? y - 1.5 : y, plate_top - 0.01]) cube([6.5, 1.5, h]);
    }
}
module tie_slots(sz, c) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([c[0] + sx*sz[0]/4 - 2.25, c[1] + sy*(sz[1]/2 + 3) - 1, plate_z - 1]) cube([4.5, 2, floor_t + 2]);
}
module lower() {
    difference() {
        union() {
            translate([0, 0, plate_z]) linear_extrude(floor_t) rr(BX[0], 0, BX[1], L, 5);
            translate([BX[1] - 1, panel_y[0], plate_z]) cube([panel_x - BX[1] + 1, panel_y[1] - panel_y[0], floor_t]);
            translate([panel_x - panel_t, panel_y[0], plate_z]) cube([panel_t, panel_y[1] - panel_y[0], panel_top - plate_z]);
            for (y = [panel_y[0], panel_y[1] - 2]) translate([BX[1] - 1, y, plate_z]) cube([panel_x - panel_t - BX[1] + 1.01, 2, panel_top - plate_z - 4]);
            translate([BX[1] - 1, (panel_y[0] + panel_y[1])/2 - 1, plate_z]) cube([panel_x - panel_t - BX[1] + 1.01, 2, panel_top - plate_z - 4]);
            for (y = axle_y) mirror_x() cradle_solid(y);
            for (p = post_xy) translate([p[0], p[1], plate_z]) cylinder(d = 8, h = deck_z - plate_z);
            guides(bat, bat_c);
        }
        for (y = axle_y) mirror_x() cradle_cuts(y);
        for (p = post_xy) translate([p[0], p[1], deck_z - 12]) cylinder(d = 2.5, h = 13);
        tie_slots(bat, bat_c);
        tie_slots(buck, buck_c);
    }
}

// ---------------------------------------------------------------- DECK
function perf_holes() = [for (i = [-1, 1], j = [-1, 1])
    [perf_c[0] + i*(perf[0]/2 - perf_in), perf_c[1] + j*(perf[1]/2 - perf_in)]];
module cam_face_slab(depth, y_lo = -10, y_hi = 14, s_lo = -21, s_hi = 21) {   // region behind the mounting face
    multmatrix([[cam_n[0], 0, cam_t[0], cam_p[0]], [0, 1, 0, cam_p[1]], [cam_n[2], 0, cam_t[2], cam_p[2]], [0, 0, 0, 1]])
        translate([-depth, y_lo, s_lo]) cube([depth, y_hi - y_lo, s_hi - s_lo]);
}
cam_post = [106, 80, 8, 8];      // x, y, size x, size y  (beside the camera, never in front of it)
module cam_mast() {
    translate([cam_post[0], cam_post[1], deck_z + deck_t - 0.01]) cube([cam_post[2], cam_post[3], 50]);
    cam_face_slab(6);
    hull() {                                                           // bridge from post top to the slab back
        translate([cam_post[0], cam_post[1], deck_z + deck_t + 38]) cube([cam_post[2], cam_post[3], 12]);
        cam_face_slab(6, 8, 14, 4, 21);
    }
}
module deck() {
    sc = [servo_shaft[0], servo_shaft[1] - 5.5];      // servo body centre (body along -y)
    difference() {
        union() {
            translate([0, 0, deck_z]) linear_extrude(deck_t) rr(BX[0], 0, BX[1], L, 5);
            for (h = perf_holes()) translate([h[0], h[1], deck_z + deck_t]) cylinder(d = 6, h = perf_stand);
            for (p = pillar_xy) translate([p[0], p[1], deck_z + deck_t]) cylinder(d = 8, h = top_z - deck_z - deck_t);
            cam_mast();
        }
        for (p = post_xy) translate([p[0], p[1], deck_z - 1]) cylinder(d = 3.4, h = deck_t + 2);
        for (p = pillar_xy) translate([p[0], p[1], top_z - 10]) cylinder(d = 2.5, h = 11);
        for (h = perf_holes()) translate([h[0], h[1], deck_z + deck_t + 1]) cylinder(d = 2.5, h = perf_stand);
        // MG90S, shaft down
        translate([sc[0] - mg90[0]/2 - 0.25, sc[1] - mg90[1]/2 - 0.25, deck_z - 1]) cube([mg90[0] + 0.5, mg90[1] + 0.5, deck_t + 2]);
        for (s = [-1, 1]) translate([sc[0], sc[1] + s*mg90_tab/2, deck_z - 1]) cylinder(d = 2, h = deck_t + 2);
        // slot for the sliding base pin
        hull() for (y = [y0 + w_min - 2, y0 + w_max + 2]) translate([xb, y, deck_z - 1]) cylinder(d = pin_d, h = deck_t + 2);
        // power switch
        translate([switch_c[0] - switch_cut[0]/2, switch_c[1] - switch_cut[1]/2, deck_z - 1]) cube([switch_cut[0], switch_cut[1], deck_t + 2]);
        // wire pass-throughs
        translate([55, 6, deck_z - 1]) cube([25, 10, deck_t + 2]);
        translate([30, 114, deck_z - 1]) cube([50, 10, deck_t + 2]);
        translate([100, 96, deck_z - 1]) cube([14, 16, deck_t + 2]);
        // camera cradle screws
        for (s = [-12, 12]) translate(cam_p + s*cam_t) rotate([0, -(90 - cam_tilt), 0]) translate([0, 0, -1]) cylinder(d = 2.5, h = 11);
    }
}

// ---------------------------------------------------------------- TOP (AprilTag)
module top_plate() {
    difference() {
        translate([0, 0, top_z]) linear_extrude(top_t) rr(BX[0], 0, BX[1], L, 5);
        for (p = pillar_xy) translate([p[0], p[1], top_z - 1]) cylinder(d = 3.4, h = top_t + 2);
        translate([(BX[0] + BX[1])/2 - 38, L/2 - 38, top_z + top_t - 0.6]) cube([76, 76, 1]);
        translate([(BX[0] + BX[1])/2, L - 2, top_z + top_t - 0.6]) linear_extrude(1) polygon([[-3, -4], [3, -4], [0, 0]]);
    }
}

// ---------------------------------------------------------------- CAMERA CRADLE (local, flat)
module cam_cradle_local() {   // x = across board (27), y = along board (40.5), z = away from mast
    bw = cam_board[0] + 5; bl = cam_board[1];
    difference() {
        union() {
            translate([-bw/2, -bl/2 - 6, 0]) cube([bw, bl + 12, 2.5]);
            for (s = [-1, 1]) translate([-bw/2, s > 0 ? bl/2 + fit : -bl/2 - fit - 6, 0]) cube([bw, 6, cam_stand + 1.6 + 2.5]);
        }
        for (s = [-1, 1]) translate([-bw/2 - 1, s > 0 ? bl/2 - 1.5 : -bl/2 - fit - 0.2 - 0.0, cam_stand])
            cube([bw + 2, 1.5 + fit + 0.2, 1.6 + 0.4]);                         // grooves, board slides in sideways
        translate([-cam_board[0]/2 + 3, -bl/2 + 6, -1]) cube([cam_board[0] - 6, bl - 12, 5]);   // window for header pins
        for (s = [-12, 12]) translate([0, s, -1]) cylinder(d = 3.4, h = 5);
    }
}
module cam_cradle_world() {
    multmatrix([[0, cam_t[0], cam_n[0], cam_p[0]], [1, 0, 0, cam_p[1]], [0, cam_t[2], cam_n[2], cam_p[2]], [0, 0, 0, 1]])
        cam_cradle_local();
}

// ---------------------------------------------------------------- SCISSOR
module link_local(servo = false) {
    difference() {
        hull() { cylinder(d = link_w, h = link_t); translate([Lk, 0, 0]) cylinder(d = link_w, h = link_t); }
        if (servo) translate([0, 0, -1]) cylinder(d = spline_d, h = link_t + 2);
        else translate([0, 0, -1]) cylinder(d = pin_d, h = link_t + 2);
        for (x = [Lk/2, Lk]) translate([x, 0, -1]) cylinder(d = pin_d, h = link_t + 2);
    }
}
module link_at(p, q, z, servo = false) {
    translate([p[0], p[1], z]) rotate(atan2(q[1] - p[1], q[0] - p[0])) link_local(servo);
}
function xt_of(len) = xb + len;
module scissor(len) {
    t = th(len); dx = Lk*cos(t); w = Lk*sin(t);
    for (k = [0:N-1]) {
        xk = xb + k*dx;
        link_at([xk, y0], [xk + dx, y0 + w], A_z, servo = (k == 0));     // top layer
        link_at([xk, y0 + w], [xk + dx, y0], B_z);                        // bottom layer
    }
}

// ---------------------------------------------------------------- TIP BLOCK
module tip_block(xt) {
    translate([xt, 0, 0]) difference() {
        union() {
            translate([-5, y_f - 24, blk_z]) cube([hinge_off + 5 + 7, 48, blk_t]);
            for (s = [-1, 1]) {
                y1 = s < 0 ? y_f - knuck_l/2 - 0.3 - lug_t : y_f + knuck_l/2 + 0.3;
                hull() {
                    translate([hinge_off - lug_r, y1, blk_z]) cube([2*lug_r, lug_t, 0.1]);
                    translate([hinge_off, y1, hinge_z]) rotate([-90, 0, 0]) cylinder(r = lug_r, h = lug_t);
                }
            }
            // stop pad: its underside matches the flap lever at the rest angle
            intersection() {
                translate([hinge_off, 0, hinge_z]) rotate([0, flap_tilt, 0])
                    translate([2.3, y_f - knuck_l/2, 1.1]) cube([4.2, knuck_l, 6]);
                translate([0, y_f - knuck_l/2, hinge_z - 3]) cube([hinge_off + 8, knuck_l, blk_z + blk_t - hinge_z + 3]);
            }
        }
        translate([0, y0, blk_z - 1]) cylinder(d = 2.5, h = blk_t + 2);                         // fixed pivot
        hull() for (y = [y0 + w_min - 2, y0 + w_max + 2]) translate([0, y, blk_z - 1]) cylinder(d = pin_d, h = blk_t + 2);
        translate([-6, y0 + 6, blk_z + blk_t - 1.5]) cube([5, 22, 2]);                           // relief for the last pivot
        translate([hinge_off, y_f - 30, hinge_z]) rotate([-90, 0, 0]) cylinder(d = 2.0, h = 60); // M2 hinge pin
    }
}

// ---------------------------------------------------------------- FLAP (local: axis on y, hangs down -z, robot side -x)
module flap_local() {
    W = flap_w + 2*finger_t;
    difference() {
        union() {
            translate([0, y_f - knuck_l/2, 0]) rotate([-90, 0, 0]) cylinder(r = knuck_r, h = knuck_l);
            translate([-flap_t/2, y_f - knuck_l/2, -4.5]) cube([flap_t, knuck_l, 4.5]);
            translate([2.0, y_f - knuck_l/2, -1]) cube([2.5, knuck_l, 2]);                  // stop lever
            hull() {
                translate([-flap_t/2, y_f - W/2, -4.5]) cube([flap_t, W, 0.1]);
                translate([0, y_f - W/2, -flap_len + flap_t/2]) rotate([-90, 0, 0]) cylinder(d = flap_t, h = W);
            }
            for (s = [-1, 1]) translate([-flap_t/2 - finger_l, s < 0 ? y_f - W/2 : y_f + W/2 - finger_t, -flap_len + 3])
                cube([finger_l + 0.01, finger_t, flap_len - 3 - 4.5]);
        }
        translate([0, y_f - knuck_l, 0]) rotate([-90, 0, 0]) cylinder(d = 2.4, h = 2*knuck_l);
    }
}
module flap_world(xt, ride = 0) {
    translate([xt + hinge_off, 0, hinge_z]) rotate([0, flap_tilt + ride, 0]) translate([0, 0, 0]) flap_local();
}

// ---------------------------------------------------------------- SPACERS + WHEEL
module spacer(h) { difference() { cylinder(d = 5.5, h = h); translate([0, 0, -1]) cylinder(d = 3.4, h = h + 2); } }
module wheel() {
    difference() {
        cylinder(d = wheel_d, h = wheel_w);
        translate([0, 0, wheel_w/2 - 2]) difference() { cylinder(d = wheel_d + 1, h = 4); cylinder(d = wheel_d - 2.4, h = 4); }
        translate([0, 0, -1]) intersection() {
            cylinder(d = shaft_d + 0.15, h = wheel_w + 5);
            translate([-shaft_d, -shaft_d, 0]) cube([shaft_d + shaft_flat - shaft_d/2 + 0.1, 2*shaft_d, wheel_w + 5]);
        }
        for (a = [0:72:359]) rotate(a) translate([wheel_d/2 - 8, 0, -1]) cylinder(d = 6, h = wheel_w + 2);
    }
}

// ---------------------------------------------------------------- PREVIEW DUMMIES
module dummy_motor() {
    color("gold") translate([BX[0] + wall, -gear_w/2, axle_z - gear_h/2]) cube([gear_l, gear_w, gear_h]);
    color("silver") translate([BX[0] + wall + gear_l, 0, axle_z]) rotate([0, 90, 0])
        intersection() { cylinder(d = 12, h = can_l); cube([gear_h, 20, 2*can_l], center = true); }
    color("white") translate([BX[0] + wall + gear_l + can_l + 4, -7.5, axle_z - 7.5]) cube([1.2, 15, 15]);
}
module dummies() {
    for (y = axle_y) mirror_x() translate([0, y, 0]) {
        dummy_motor();
        color("dimgray") translate([BX[0] - 1 - wheel_w, 0, axle_z]) rotate([0, 90, 0]) wheel();
    }
    color("darkslategray") translate([bat_c[0] - bat[0]/2, bat_c[1] - bat[1]/2, plate_top]) cube([bat[0], bat[1], bat_h]);
    color("mediumblue") translate([buck_c[0] - buck[0]/2, buck_c[1] - buck[1]/2, plate_top]) cube([buck[0], buck[1], 14]);
    color("seagreen") translate([perf_c[0] - perf[0]/2, perf_c[1] - perf[1]/2, deck_z + deck_t + perf_stand]) cube([perf[0], perf[1], 1.6]);
    color("black") translate([perf_c[0] - 14.25, perf_c[1] - 26, deck_z + deck_t + perf_stand + 1.6]) cube([28.5, 52, 12]);
    color("royalblue") translate([servo_shaft[0] - mg90[0]/2, servo_shaft[1] - 5.5 - mg90[1]/2, deck_z + deck_t - spline_drop + 3.5]) cube([mg90[0], mg90[1], 26]);
    color("black") multmatrix([[0, cam_t[0], cam_n[0], cam_p[0]], [1, 0, 0, cam_p[1]], [0, cam_t[2], cam_n[2], cam_p[2]], [0, 0, 0, 1]])
        translate([-cam_board[0]/2, -cam_board[1]/2, 2.5 + cam_stand]) cube([cam_board[0], cam_board[1], 1.6]);
}
module cube_at(x0) { color("khaki") translate([x0, y_f - cube_s/2, 0]) cube(cube_s); }

module arm_at(len, ride = 0) {
    color("mediumpurple") scissor(len);
    color("orange") tip_block(xt_of(len));
    color("tomato") flap_world(xt_of(len), ride);
}
module assembly(state = "stow") {
    color("steelblue") lower();
    color("lightsteelblue") deck();
    color("whitesmoke") top_plate();
    color("orange") cam_cradle_world();
    dummies();
    if (state == "stow")  arm_at(stow_len);
    if (state == "carry") { arm_at(carry_len()); cube_at(panel_x); }
    if (state == "reach") { arm_at(N*Lk*cos(theta_min)); cube_at(panel_x + 100 - cube_s - 2); }
}
// hinge position that pulls a cube flat against the panel
function carry_len() = (panel_x + cube_s + 7.8) - hinge_off - xb;

// ---------------------------------------------------------------- print orientations
module lay(p) {
    if (p == "lower")      translate([0, 0, -plate_z]) lower();
    if (p == "deck")       translate([0, 0, -deck_z]) deck();
    if (p == "top")        translate([0, 0, -top_z]) top_plate();
    if (p == "cam_cradle") cam_cradle_local();
    if (p == "link")       link_local(false);
    if (p == "link_servo") link_local(true);
    if (p == "tip_block")  translate([0, 0, blk_z + blk_t]) mirror([0, 0, 1]) tip_block(0);     // upside down
    if (p == "flap")       rotate([90, 0, 0]) translate([0, -(y_f - (flap_w + 2*finger_t)/2), 0]) flap_local();
    if (p == "spacer_10")  spacer(10);
    if (p == "spacer_4")   spacer(4);
    if (p == "wheel")      wheel();
}

if (part == "assembly")            assembly("stow");
else if (part == "assembly_carry") assembly("carry");
else if (part == "assembly_reach") assembly("reach");
else if (part != "none")           lay(part);
