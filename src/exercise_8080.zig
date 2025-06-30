// ******************************************************
// Exercise 8080: runs standard 8080 tests on the emu
// ******************************************************

const std = @import("std");
const c = @import("8080.zig");

// loads one of the tests from disk
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

// run a test: simulates a CP/M system
fn run_test(ram: []u8, debug: bool) u8 {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = c.Intel8080{ .ports_in = &in, .ports_out = &out };

    _ = debug;

    var success: u1 = 0;
    cpu.pc = 0x100;
    cpu.sp = 0x100;
    ram[5] = 0xC9; // RET at 0x05 to handle "CALL 5"

    // inject OUT 0,A at 0x0000 (signal to stop test)
    ram[0x0000] = 0xD3;
    ram[0x0001] = 0xC9;

    // execute
    while (true) {
        // const pc_before = cpu.pc;
        if (ram[cpu.pc] == 0x76) {
            std.debug.print("HLT at {X:04}\n", .{cpu.pc});
            return 1;
        }

        // handle calls to INT 5
        if (cpu.pc == 0x0005) {
            switch (cpu.c) {
                9 => {
                    var i = cpu.get_register_pair(c.Intel8080.register_pairs.D);
                    while (ram[i] != '$') : (i += 1) {
                        std.debug.print("{c}", .{ram[i]});
                    }
                    std.debug.print("\n", .{});
                    success = 1;
                },

                2 => {
                    std.debug.print("{c}", .{cpu.e});
                },
                else => unreachable,
            }
        }
        cpu.execute_instruction(ram[0..], false);

        if (cpu.pc == 0) {
            // std.debug.print("\nJump to 0000 from {X:04}\n", .{pc_before});
            // std.debug.print("\nSuccess = {d}\n", .{success});
            return 1;
        }
    }
    return 0;
}

pub fn main() void {
    var ram: [0xFFFF]u8 = undefined;

    const test_files = [_][]const u8{ "resources/8080PRE.COM", "resources/tst8080.com", "resources/TEST.COM", "resources/CPUTEST.COM" };

    for (test_files, 0..) |file, i| {
        std.debug.print(">> Running test: #{d}: {s}\n", .{ i, file });
        const bytes = load_rom(file, ram[0x100..]);
        const result = run_test(ram[0..], true);
        std.debug.print("\n<< Test result: {}, bytes read: {any}\n\n", .{ result, bytes });
    }
}

test "load_rom" {
    var ram: [0xFFFF]u8 = undefined;
    const rom_path = "resources/8080PRE.COM";
    const bytes = load_rom(rom_path, ram[0..]);
    std.debug.print("bytes read: {any}\n", .{bytes});

    try std.testing.expect(ram[0] == 0x3E);
}

test "run_test" {
    var ram: [0xFFFF]u8 = undefined;

    const rom_path = "resources/8080PRE.COM";
    const bytes: u64 = try load_rom(rom_path, ram[0x100..]) orelse 0;

    const result = run_test(ram[0..], true);
    std.testing.expect(result == 1);
    std.debug.print("test result: {}, bytes read: {any}\n", .{ result, bytes });
}

test "array of strings" {
    const test_files = [_][]const u8{ "resources/8080PRE.COM", "resources/tst8080.com", "resources/TEST.COM" };

    for (test_files, 0..) |file, i| {
        std.debug.print("file #{d}: {s}\n", .{ i, file });
    }
}
