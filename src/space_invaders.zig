//
// Space Invaders emulator
//

const std = @import("std");
const c = @import("8080.zig");

const SpaceInvaders = struct {
    // memory map
    // 0000 - 1FFF  - 8K -> rom
    // 2000 - 23FF  - 1K -> ram
    // 2400 - 3FFF  - 7K -> video ram: rotated 90 deg counter-clockwise
    // rotated screen: x_new = y; y_new = (256 - x)

    cpu: c.Intel8080 = undefined,

    ports: [16]u8 = undefined, // only 5 ports are used
    shift_register: u16 = 0,
    shift_register_offset: u8 = 0,
    vram_offset: u16 = 0x2400,

    ram: [16 * 1024]u8 = undefined,

    // Input ports (1, 2): 0 is not used by the SI code
    const Port_1_bits = enum {
        coin_slot,
        p1_select,
        p2_select,
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

    // The output ports will be handled separately (by if statements to convert from int to the flag set)

    // Init machine
    fn init(self: *SpaceInvaders, dip_settings: u8, rom_path: []const u8) u64 {
        self.cpu = c.Intel8080{ .ports_in = self.ports[0..], .ports_out = self.ports[0..] };

        self.ports[1] = 0b10001000; // some bits are always 1
        self.ports[2] = dip_settings;
        self.ports[3] = 0b11100000; // some bits are always 1
        self.ports[5] = 0b11000000; // some bits are always 1

        const bytes: u64 = load_rom(rom_path, self.ram[0..]) catch blk: {
            std.debug.print("Error loading ROM\n", .{});
            break :blk 0;
        };
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
    fn set_port_flag(self: *SpaceInvaders, port_nbr: u8, flag: anytype) void {
        const f = @intFromEnum(flag);
        const mask = (@as(u8, 1) << f);
        self.ports[port_nbr] |= mask;
        std.debug.print("pn={d}, flag={d}, mask={b:08} port bits={b:08}\n", .{ port_nbr, f, mask, ports_in[port_nbr] });
    }
};

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
