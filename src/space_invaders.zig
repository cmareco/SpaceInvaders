//
// Space Invaders emulator
//

const std = @import("std");
const rl = @import("raylib");
const c = @import("8080.zig");

const SpaceInvaders = struct {
    // memory map
    // 0000 - 1FFF  - 8K -> rom
    // 2000 - 23FF  - 1K -> ram
    // 2400 - 3FFF  - 7K -> video ram: rotated 90 deg counter-clockwise
    // rotated screen: x_new = y; y_new = (256 - x)

    cpu: c.Intel8080 = undefined,

    ports: [256]u8 = undefined, // only 5 ports are used
    shift_register: u16 = 0,
    shift_register_offset: u8 = 0,
    vram_offset: u16 = 0x2400,
    ram: [32 * 1024]u8 = undefined,
    background_texture: rl.Texture2D = undefined,
    bloom_shader: rl.Shader = undefined,
    targetTexture: rl.RenderTexture2D = undefined,
    x_offset: i32 = 164, // we draw the emuator screen at this offset
    y_offset: i32 = 30,
    sounds: [10]rl.Sound = undefined,

    game_paused: bool = false,
    prev_sound1: u8 = 0, // used to detect if the sound was played

    const sound_names = enum { ufo, shot, player_die, invader_die, fleet1, fleet2, fleet3, fleet4, ufo_hit, extended_play };

    // Input ports (1, 2): 0 is not used by the SI code
    const Port_1_bits = enum {
        coin_slot,
        p2_select,
        p1_select,
        unused1,
        p1_fire,
        p1_left,
        p1_right,
        unused2,
    };

    const Port_2_bits = enum {
        dip_nbr_ships_bit1,
        dip_nbr_ships_bit2,
        dpi_tilt,
        dip_bonus_life,
        p2_fire,
        p2_left,
        p2_right,
        dip_show_coin_info,
    };

    const scale_factor: i32 = 2;
    const screen_width: i16 = (224 * scale_factor);
    const screen_height: i16 = (256 * scale_factor);

    // The output ports will be handled separately (by if statements to convert from int to the flag set)

    // Init machine
    fn init(self: *SpaceInvaders, dip_settings: u8, rom_path: []const u8) u64 {
        self.cpu = c.Intel8080{ .ports_in = self.ports[0..], .ports_out = self.ports[0..] };
        self.ports[0] = 0b01110000;
        self.ports[1] = 0b00001000; // some bits are always 1
        self.ports[2] = dip_settings;
        self.ports[3] = 0b00000000; // some bits are always 1
        self.ports[5] = 0b000000000; // some bits are always 1

        const bytes: u64 = load_rom(rom_path, self.ram[0..]) catch blk: {
            std.debug.print("Error loading ROM\n", .{});
            break :blk 0;
        };
        std.debug.print("Rom loaded - {d} bytes\n", .{bytes});

        // load background image
        const image: rl.Image = rl.loadImage("resources/invaders_horizontal.png") catch blk: {
            std.debug.print("Error loading background image\n", .{});
            break :blk undefined;
        };
        self.background_texture = rl.loadTextureFromImage(image) catch blk: {
            std.debug.print("Error loading background texture\n", .{});
            //rl.unloadImage(image);
            break :blk undefined;
        };
        rl.unloadImage(image); // Unload image from RAM, keep texture in VRAM

        // load shader
        self.bloom_shader = rl.loadShader(null, "resources/shaders/bloom.fs") catch blk: {
            std.debug.print("Error loading shader\n", .{});
            break :blk undefined;
        };

        // Load sounds
        const sound_files = [_][:0]const u8{ "resources/sounds/0.wav", "resources/sounds/1.wav", "resources/sounds/2.wav", "resources/sounds/3.wav", "resources/sounds/4.wav", "resources/sounds/5.wav", "resources/sounds/6.wav", "resources/sounds/7.wav", "resources/sounds/8.wav", "resources/sounds/9.wav" };

        // Load all sounds
        for (sound_files, 0..) |file, i| {
            self.sounds[i] = rl.loadSound(file) catch blk: {
                std.debug.print("Error loading sound: {s}\n", .{file});
                break :blk undefined;
            };
        }

        return bytes;
    }

    // loads rom into memory
    fn load_rom(rom_path: []const u8, ram: []u8) !u64 {
        const file = try std.fs.cwd().openFile(rom_path, .{});
        defer file.close();

        const rom_size = try file.getEndPos();
        const rom_data = try file.readToEndAlloc(std.heap.page_allocator, rom_size);
        defer std.heap.page_allocator.free(rom_data);

        // Load ROM data into Chip8 memory
        std.mem.copyForwards(u8, ram[0..], rom_data);
        return rom_size;
    }

    // sets a specific bit flag on a port
    fn set_port_flag(self: *SpaceInvaders, port_nbr: u8, flag: anytype, value: u1) void {
        const f = @intFromEnum(flag);
        const mask = (@as(u8, 1) << f);
        if (value == 1) // set flag
            self.ports[port_nbr] |= mask
        else // reset flag: ~mask reverses the mask: 00100000 -> 11011111
            self.ports[port_nbr] &= ~mask;

        //std.debug.print("pn={d}, flag={d}, mask={b:08} port bits={b:08}\n", .{ port_nbr, f, mask, self.ports[port_nbr] });
    }

    // hardware for shift register
    fn handle_ports_out(self: *SpaceInvaders, port_nbr: u8) void {
        switch (port_nbr) {
            5 => {
                //std.debug.print("port[5] = {b:08}\n", .{self.ports[5]});

                const sound_to_play = self.ports[5] & 0b00011111;
                if (sound_to_play & 0b00000001 > 0 and !rl.isSoundPlaying(self.sounds[4])) {
                    rl.playSound(self.sounds[4]); // fleet 1
                }
                if (sound_to_play & 0b00000010 > 0 and !rl.isSoundPlaying(self.sounds[5])) {
                    rl.playSound(self.sounds[5]); // fleet 2
                }
                if (sound_to_play & 0b00000100 > 0 and !rl.isSoundPlaying(self.sounds[6])) {
                    rl.playSound(self.sounds[6]); // fleet 3
                }
                if (sound_to_play & 0b00001000 > 0 and !rl.isSoundPlaying(self.sounds[7])) {
                    rl.playSound(self.sounds[7]); // fleet 4
                }
                if (sound_to_play & 0b00010000 > 0 and !rl.isSoundPlaying(self.sounds[8])) {
                    rl.playSound(self.sounds[8]); // ufo hit
                }

                // set the port to 0, so it doesn't play the sound again
                self.ports[5] = 0b00000000;
            },

            4 => {
                self.shift_register = (@as(u16, self.ports[4]) << 8) | (self.shift_register >> 8);
            },
            3 => { // play sounds

                const sound_to_play = self.ports[3] & 0b00011111;
                if (sound_to_play & 0b00000001 > 0 and !rl.isSoundPlaying(self.sounds[0])) {
                    rl.playSound(self.sounds[0]); // ufo sound
                }
                if (self.prev_sound1 & 0b00000010 == 0 and sound_to_play & 0b00000010 > 0 and !rl.isSoundPlaying(self.sounds[1])) {
                    rl.playSound(self.sounds[1]); // shot sound
                    std.debug.print("shot sound played, prev: {b:08}\n", .{self.prev_sound1});
                }
                if (sound_to_play & 0b00000100 > 0 and !rl.isSoundPlaying(self.sounds[2])) {
                    rl.playSound(self.sounds[2]); // player die sound
                }
                if (sound_to_play & 0b00001000 > 0 and !rl.isSoundPlaying(self.sounds[3])) {
                    rl.playSound(self.sounds[3]); // invader die sound
                }
                if (sound_to_play & 0b00010000 > 0 and !rl.isSoundPlaying(self.sounds[9])) {
                    rl.playSound(self.sounds[9]); // extended play sound
                }
                // reset the port
                self.prev_sound1 = sound_to_play;
                //self.ports[3] &= 0b00100001;
                self.ports[3] = 0;
            },
            2 => {
                self.shift_register_offset = self.ports[2] & 0b00000111;
                self.ports[2] = 0b00000001;
                // TO DO: handle sounds here
                //
                //
            },
            else => {
                _ = 1;
            },
        }
    }

    // receive key pressed and pass it to the ports
    fn handle_input(self: *SpaceInvaders) void {
        // if is_key_pressed is true, a key was pressed, so we set the value to 1
        // if not, a key was released, so set to 0

        const c_key: u1 = if (rl.isKeyDown(.c)) 1 else 0;
        self.set_port_flag(1, SpaceInvaders.Port_1_bits.coin_slot, c_key);

        const one_key: u1 = if (rl.isKeyDown(.one)) 1 else 0;
        self.set_port_flag(1, SpaceInvaders.Port_1_bits.p1_select, one_key);

        const left_key: u1 = if (rl.isKeyDown(.left)) 1 else 0;
        self.set_port_flag(1, SpaceInvaders.Port_1_bits.p1_left, left_key);

        const right_key: u1 = if (rl.isKeyDown(.right)) 1 else 0;
        self.set_port_flag(1, SpaceInvaders.Port_1_bits.p1_right, right_key);

        const space_key: u1 = if (rl.isKeyDown(.space)) 1 else 0;
        self.set_port_flag(1, SpaceInvaders.Port_1_bits.p1_fire, space_key);

        if (rl.isKeyPressed(.p)) {
            self.game_paused = if (self.game_paused) false else true;
        }
    }

    // draw screen
    fn draw_screen(self: *SpaceInvaders) void {
        var pixel: u8 = undefined;

        rl.beginTextureMode(self.targetTexture);
        rl.drawTexture(self.background_texture, 0, 0, .white);

        for (0..224) |line| {
            for (0..32) |column| {
                pixel = self.ram[self.vram_offset + (column + (32 * line))];
                for (0..8) |p| {
                    if ((pixel >> (7 - @as(u3, @truncate(p)))) & 0b00000001 == 1) {
                        const column_i32 = @as(i32, @intCast(column));
                        const line_i32 = @as(i32, @intCast(line));
                        const x: i32 = line_i32 * scale_factor;
                        const y: i32 = (255 - (column_i32 * 8) + @as(i32, @intCast(p))) * scale_factor;
                        // determine the color, based on the scanline
                        const pixel_color: rl.Color = switch (y) {
                            78...138 => .magenta,
                            380...492 => .dark_green,
                            else => .white,
                        };

                        rl.drawRectangle(x + self.x_offset, y + self.y_offset, scale_factor, scale_factor, pixel_color);
                    }
                }
            }
        }
        if (self.game_paused)
            rl.drawText("Game paused", (screen_width / 2) - 50, screen_height / 2, 20, .light_gray);
        rl.endTextureMode();

        // draw the target texture with the bloom shader
        rl.beginDrawing();
        rl.drawTexture(self.background_texture, 0, 0, .white);

        rl.beginShaderMode(self.bloom_shader);
        const rect = rl.Rectangle{ .x = 0, .y = 0, .width = 775.0, .height = -552.0 };
        const vec2 = rl.Vector2{ .x = 0, .y = 0 };
        rl.drawTextureRec(self.targetTexture.texture, rect, vec2, .white);
        rl.endShaderMode();

        rl.endDrawing();
    }

    // // draw screen
    // fn draw_screen(self: *SpaceInvaders) void {
    //     var pixel: u8 = undefined;

    //     rl.beginDrawing();

    //     rl.drawTexture(self.background_texture, 0, 0, .white);

    //     //rl.beginShaderMode(self.bloom_shader);
    //     for (0..224) |line| {
    //         for (0..32) |column| {
    //             pixel = self.ram[self.vram_offset + (column + (32 * line))];
    //             for (0..8) |p| {
    //                 if ((pixel >> (7 - @as(u3, @truncate(p)))) & 0b00000001 == 1) {
    //                     const column_i32 = @as(i32, @intCast(column));
    //                     const line_i32 = @as(i32, @intCast(line));
    //                     const x: i32 = line_i32 * scale_factor;
    //                     const y: i32 = (255 - (column_i32 * 8) + @as(i32, @intCast(p))) * scale_factor;
    //                     // determine the color, based on the scanline
    //                     const pixel_color: rl.Color = switch (y) {
    //                         78...138 => .magenta,
    //                         380...492 => .dark_green,
    //                         else => .white,
    //                     };

    //                     rl.drawRectangle(x + self.x_offset, y + self.y_offset, scale_factor, scale_factor, pixel_color);
    //                 }
    //             }
    //         }
    //     }
    //     //rl.endShaderMode();
    //     if (self.game_paused)
    //         rl.drawText("Game paused", (screen_width / 2) - 50, screen_height / 2, 20, .light_gray);

    //     rl.endDrawing();
    // }

    // runs a single cpu cyclw
    fn run_cycle(self: *SpaceInvaders, debug: bool) void {
        const instruction = self.ram[self.cpu.pc];

        var port_nbr: u8 = 0;

        if (self.game_paused)
            return;

        const opcode = @as(c.Intel8080.opcodes, @enumFromInt(instruction));

        if (opcode == .IN_ or opcode == .OUT_)
            port_nbr = self.ram[self.cpu.pc + 1];

        // If it's an IN opcode, calc the values to be returned by the port
        if (opcode == c.Intel8080.opcodes.IN_ and port_nbr == 3) {
            // for IN: do it before running the instruction on the CPU so
            // the value is in port 3 already
            self.ports[3] = @truncate(((self.shift_register >> @as(u4, @truncate((8 - self.shift_register_offset)))) & 0xFF));
        }

        self.cpu.execute_instruction(self.ram[0..], debug);

        if (opcode == c.Intel8080.opcodes.OUT_) {
            // For OUT: run the handler after the CPU has executed the instruction
            // since the port values were already set by the CPU
            // if (port_nbr == 3)
            //     std.debug.print("{X:04}: {s} {X:02} port[3] = {b:08} ]\n", .{ self.cpu.pc - 1, c.Intel8080.opcode_names[instruction], self.ram[self.cpu.pc - 1], self.ports[3] });

            self.handle_ports_out(port_nbr);
        }
    }
};

fn debug_fn() void {
    var si = SpaceInvaders{};
    std.debug.print("port[1] = {b:08}\n", .{si.ports[1]});
    while (true) {
        if (rl.isKeyDown(rl.KeyboardKey.escape) == true) { // escape: exit
            return;
        }
        si.handle_input();
        if (si.ports[1] != 0)
            std.debug.print("port[1] = {b:08}\n", .{si.ports[1]});
    }
}

// execute half a screen worth of cpu cycles
fn run_half_screen(si: *SpaceInvaders, cycles_by_frame: u32, debug: bool) void {
    const cycles = si.cpu.cycles;
    while (si.cpu.cycles - cycles < cycles_by_frame and si.game_paused == false) {
        si.run_cycle(debug);
        si.handle_input();
    }
}

// Global Constants
const cycles_per_frame = 33333;
const frame_time = 0.016667;
const window_width: i32 = 775; // size of the emulator windows, needs to fit the background image
const window_height: i32 = 572;
const x_offset = 164; // we draw the emuator screen at this offset
const y_offset = 30;

pub fn main() !void {
    // Args
    const args = try std.process.argsAlloc(std.heap.page_allocator);
    defer std.process.argsFree(std.heap.page_allocator, args);
    const debug: bool = if (args.len == 2) true else false;

    // Raylib init: must init this before calling SpaceInvaders.init()
    rl.initWindow(window_width, window_height, "Space Invaders");
    defer rl.closeWindow(); // Close window and OpenGL context
    rl.setTargetFPS(60); // Set our game to run at 60 frames-per-second

    rl.initAudioDevice();
    defer rl.closeAudioDevice(); // Close audio device

    // Init Space Invaders
    const rom_path = "resources/invaders.rom";
    var si = SpaceInvaders{};

    if (si.init(0b00000001, rom_path) == 0) {
        std.debug.print(("Not initialized, exiting"), .{});
        return;
    }
    si.targetTexture = rl.loadRenderTexture(window_width, window_height) catch {
        std.debug.print("Error loading render texture\n", .{});
        return;
    };

    var last_frame = rl.getTime();

    //--------------------------------------------------------------------------------------
    // Main game loop
    while (!rl.windowShouldClose()) { // Detect window close button or ESC key

        if (rl.getTime() - last_frame >= frame_time) {
            // execute half a frame worth of cycles
            si.draw_screen();
            run_half_screen(&si, cycles_per_frame, debug);

            si.cpu.interrupt_vector = 0xCF; // RST 1
            si.cpu.interrupt_req = 1;
            //si.draw_screen();

            run_half_screen(&si, cycles_per_frame, debug);
            si.cpu.interrupt_vector = 0xD7; // RST 2
            si.cpu.interrupt_req = 1;
            //si.draw_screen();
            last_frame = rl.getTime();
        }

        //----------------------------------------------------------------------------------
    }
}

//----------------------------------------------------------------------
//
//  T E S T S
//
//

test "handle input" {
    std.testing.refAllDecls(@This());
    var si = SpaceInvaders{};

    std.debug.print("before: port[1]={b:08}\n", .{si.ports[1]});
    si.handle_input(true, rl.KeyboardKey.c);
    std.debug.print("after: port[1]={b:08}\n", .{si.ports[1]});
}

test "shift register" {
    var si = SpaceInvaders{};

    si.shift_register = 0x0000;
    si.ports[3] = 0xAA;

    si.handle_ports_out(4); // shift data
    try std.testing.expect(si.shift_register == 0xAA00);

    si.ports[3] = 0xBB;
    si.handle_ports_out(4); // shift data
    try std.testing.expect(si.shift_register == 0xBBAA);

    si.ports[2] = 2; // 4 bits
    si.handle_ports_out(2); // set offset

    // return shifted value
    std.debug.print("prev: {b:016}\n", .{si.shift_register});
    const shift: u4 = @truncate(8 - si.shift_register_offset);
    si.ports[3] = @truncate((si.shift_register >> shift) & 0xFF);
    try std.testing.expect(si.ports[3] == 0b11101110);
    std.debug.print("after: {b:08}\n", .{si.ports[3]});
}

test "init" {
    const rom_path = "resources/invaders.rom";
    var si = SpaceInvaders{};
    if (si.init(0b00000011, rom_path) == 0) {
        std.debug.print(("Not initialized, exiting"), .{});
    }

    try std.testing.expect(si.ram[3] == 0xC3 and si.ram[4] == 0xD4);

    si.shift_register = 2;
    std.debug.print(("shift register: {d}\n"), .{si.shift_register});
}
