const std = @import("std");

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

var ports_in: [4]u8 = undefined;

fn set_port_flag(port_nbr: u8, flag: anytype) void {
    const f = @intFromEnum(flag);
    const mask = (@as(u8, 1) << f);
    ports_in[port_nbr] |= mask;
    std.debug.print("pn={d}, flag={d}, mask={b:08} port bits={b:08}\n", .{ port_nbr, f, mask, ports_in[port_nbr] });
}

pub fn main() void {
    ports_in[1] = 0b10001000;
    ports_in[2] = 0b00000000;
    set_port_flag(1, Port_1_bits.p1_fire);
    set_port_flag(2, Port_2_bits.p2_left);
}

// var cpu = c.Intel8080{ .ports_in = &in, .ports_out = &out };
