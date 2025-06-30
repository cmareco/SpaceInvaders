const std = @import("std");

pub const Intel8080 = struct {
    // custom registers
    pc: u16 = 0x0000,
    sp: u16 = 0x0000,
    //registers
    a: u8 = 0, // accumulator
    b: u8 = 0, // register b
    c: u8 = 0, // register c
    d: u8 = 0, // register d
    e: u8 = 0, // register e
    h: u8 = 0, // register h
    l: u8 = 0, // register l
    flags: flags_struct = .{ .byte = 0x02 }, // initial flags, set carry and zero

    running_state: run_state = .running,

    // interrupts
    inte: u1 = 0, // interrupt enable bit
    interrupt_req: u1 = 0, // interrupt request signal
    interrupt_delay: u4 = 0, // ??
    interrupt_vector: u8 = 0, // opcode provided by interrupt req

    ports_in: []u8,
    ports_out: []u8,

    cycles: u32 = 0,

    pub const register_pairs = enum(u2) {
        B = 0,
        D = 1,
        H = 2,
        PSW = 3,
    };

    pub const flag_bits = packed struct { carry: u1, nu1: u1, parity: u1, nu2: u1, aux_carry: u1, nu3: u1, zero: u1, sign: u1 };

    const flags_struct = packed union {
        byte: u8,
        bits: flag_bits,
    };

    const run_state = enum {
        running,
        halted,
        stopped,
    };

    // zig fmt: off
    pub const opcodes = enum(u8) { 
        NOP, LXI_B, STAX_B, INX_B, INR_B, DCR_B, MVI_B, RLC, NOP0, DAD_B, LDAX_B, DCX_B, INR_C, DCR_C, MVI_C, RRC,
        NOP1, LXI_D, STAX_D, INX_D, INR_D, DCR_D, MVI_D, RAL, NOP2, DAD_D, LDAX_D, DCX_D, INR_E, DCR_E, MVI_E, RAR, // 1 
        NOP3, LXI_H, SHLD, INX_H, INR_H, DCR_H, MVI_H, DAA, NOP4, DAD_H, LHLD, DCX_H, INR_L, DCR_L, MVI_L, CMA, //2
        NOP5, LXI_SP, STA, INX_SP, INR_M, DCR_M, MVI_M, STC, NOP6, DAD_SP, LDA, DCX_SP, INR_A, DCR_A, MVI_A, CMC, //3
        MOV_BB, MOV_BC, MOV_BD, MOV_BE, MOV_BH, MOV_BL, MOV_BM, MOV_BA, MOV_CB, MOV_CC, MOV_CD, MOV_CE, MOV_CH, MOV_CL, MOV_CM, MOV_CA, // 4
        MOV_DB, MOV_DC, MOV_DD, MOV_DE, MOV_DH, MOV_DL, MOV_DM, MOV_DA, MOV_EB, MOV_EC, MOV_ED, MOV_EE, MOV_EH, MOV_EL, MOV_EM, MOV_EA, 
        MOV_HB, MOV_HC, MOV_HD, MOV_HE, MOV_HH, MOV_HL, MOV_HM, MOV_HA, MOV_LB, MOV_LC, MOV_LD, MOV_LE, MOV_LH, MOV_LL, MOV_LM, MOV_LA, 
        MOV_MB, MOV_MC, MOV_MD, MOV_ME, MOV_MH, MOV_ML, HLT, MOV_MA, MOV_AB, MOV_AC, MOV_AD, MOV_AE, MOV_AH, MOV_AL, MOV_AM, MOV_AA, 
        ADD_B, ADD_C, ADD_D, ADD_E, ADD_H, ADD_L, ADD_M, ADD_A, ADC_B, ADC_C, ADC_D, ADC_E, ADC_H, ADC_L, ADC_M, ADC_A, 
        SUB_B, SUB_C, SUB_D, SUB_E, SUB_H, SUB_L, SUB_M, SUB_A, SBB_B, SBB_C, SBB_D, SBB_E, SBB_H, SBB_L, SBB_M, SBB_A, 
        ANA_B, ANA_C, ANA_D, ANA_E, ANA_H, ANA_L, ANA_M, ANA_A, XRA_B, XRA_C, XRA_D, XRA_E, XRA_H, XRA_L, XRA_M, XRA_A, 
        ORA_B, ORA_C, ORA_D, ORA_E, ORA_H, ORA_L, ORA_M, ORA_A, CMP_B, CMP_C, CMP_D, CMP_E, CMP_H, CMP_L, CMP_M, CMP_A, 
        RNZ, POP_B, JNZ, JMP, CNZ, PUSH_B, ADI, RST_0, RZ, RET, JZ, JMP1, CZ, CALL, ACI, RST_1, 
        RNC, POP_D, JNC, OUT_, CNC, PUSH_D, SUI, RST_2, RC, RET1, JC, IN_, CC, CALL1, SBI, RST_3, 
        RPO, POP_H, JPO, XTHL, CPO, PUSH_H, ANI, RST_4, RPE, PCHL, JPE, XCHG, CPE, CALL2, XRI, RST_5,
        RP, POP_PSW, JP, DI, CP, PUSH_PSW, ORI, RST_6, RM, SPHL, JM, EI, CM, CALL3, CPI, RST_7 
    };
    // zig fmt: on

    const opcode_names = [_][]const u8{ "NOP", "LXI_B", "STAX_B", "INX_B", "INR_B", "DCR_B", "MVI_B", "RLC", "NOP0", "DAD_B", "LDAX_B", "DCX_B", "INR_C", "DCR_C", "MVI_C", "RRC", "NOP1", "LXI_D", "STAX_D", "INX_D", "INR_D", "DCR_D", "MVI_D", "RAL", "NOP2", "DAD_D", "LDAX_D", "DCX_D", "INR_E", "DCR_E", "MVI_E", "RAR", "NOP3", "LXI_H", "SHLD", "INX_H", "INR_H", "DCR_H", "MVI_H", "DAA", "NOP4", "DAD_H", "LHLD", "DCX_H", "INR_L", "DCR_L", "MVI_L", "CMA", "NOP5", "LXI_SP", "STA", "INX_SP", "INR_M", "DCR_M", "MVI_M", "STC", "NOP6", "DAD_SP", "LDA", "DCX_SP", "INR_A", "DCR_A", "MVI_A", "CMC", "MOV_BB", "MOV_BC", "MOV_BD", "MOV_BE", "MOV_BH", "MOV_BL", "MOV_BM", "MOV_BA", "MOV_CB", "MOV_CC", "MOV_CD", "MOV_CE", "MOV_CH", "MOV_CL", "MOV_CM", "MOV_CA", "MOV_DB", "MOV_DC", "MOV_DD", "MOV_DE", "MOV_DH", "MOV_DL", "MOV_DM", "MOV_DA", "MOV_EB", "MOV_EC", "MOV_ED", "MOV_EE", "MOV_EH", "MOV_EL", "MOV_EM", "MOV_EA", "MOV_HB", "MOV_HC", "MOV_HD", "MOV_HE", "MOV_HH", "MOV_HL", "MOV_HM", "MOV_HA", "MOV_LB", "MOV_LC", "MOV_LD", "MOV_LE", "MOV_LH", "MOV_LL", "MOV_LM", "MOV_LA", "MOV_MB", "MOV_MC", "MOV_MD", "MOV_ME", "MOV_MH", "MOV_ML", "HLT", "MOV_MA", "MOV_AB", "MOV_AC", "MOV_AD", "MOV_AE", "MOV_AH", "MOV_AL", "MOV_AM", "MOV_AA", "ADD_B", "ADD_C", "ADD_D", "ADD_E", "ADD_H", "ADD_L", "ADD_M", "ADD_A", "ADC_B", "ADC_C", "ADC_D", "ADC_E", "ADC_H", "ADC_L", "ADC_M", "ADC_A", "SUB_B", "SUB_C", "SUB_D", "SUB_E", "SUB_H", "SUB_L", "SUB_M", "SUB_A", "SBB_B", "SBB_C", "SBB_D", "SBB_E", "SBB_H", "SBB_L", "SBB_M", "SBB_A", "ANA_B", "ANA_C", "ANA_D", "ANA_E", "ANA_H", "ANA_L", "ANA_M", "ANA_A", "XRA_B", "XRA_C", "XRA_D", "XRA_E", "XRA_H", "XRA_L", "XRA_M", "XRA_A", "ORA_B", "ORA_C", "ORA_D", "ORA_E", "ORA_H", "ORA_L", "ORA_M", "ORA_A", "CMP_B", "CMP_C", "CMP_D", "CMP_E", "CMP_H", "CMP_L", "CMP_M", "CMP_A", "RNZ", "POP_B", "JNZ", "JMP", "CNZ", "PUSH_B", "ADI", "RST_0", "RZ", "RET", "JZ", "JMP1", "CZ", "CALL", "ACI", "RST_1", "RNC", "POP_D", "JNC", "OUT_", "CNC", "PUSH_D", "SUI", "RST_2", "RC", "RET1", "JC", "IN_", "CC", "CALL1", "SBI", "RST_3", "RPO", "POP_H", "JPO", "XTHL", "CPO", "PUSH_H", "ANI", "RST_4", "RPE", "PCHL", "JPE", "XCHG", "CPE", "CALL2", "XRI", "RST_5", "RP", "POP_PSW", "JP", "DI", "CP", "PUSH_PSW", "ORI", "RST_6", "RM", "SPHL", "JM", "EI", "CM", "CALL3", "CPI", "RST_7" };

    const opcode_cycles = [_]u8{
        //  0  1   2   3   4   5   6   7   8  9   A   B   C   D   E  F
        4, 10, 7, 5, 5, 5, 7, 4, 4, 10, 7, 5, 5, 5, 7, 4, // 0
        4, 10, 7, 5, 5, 5, 7, 4, 4, 10, 7, 5, 5, 5, 7, 4, // 1
        4, 10, 16, 5, 5, 5, 7, 4, 4, 10, 16, 5, 5, 5, 7, 4, // 2
        4, 10, 13, 5, 10, 10, 10, 4, 4, 10, 13, 5, 5, 5, 7, 4, // 3
        5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5, // 4
        5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5, // 5
        5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5, // 6
        7, 7, 7, 7, 7, 7, 7, 7, 5, 5, 5, 5, 5, 5, 7, 5, // 7
        4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, // 8
        4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, // 9
        4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, // A
        4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, // B
        5, 10, 10, 10, 11, 11, 7, 11, 5, 10, 10, 10, 11, 17, 7, 11, // C
        5, 10, 10, 10, 11, 11, 7, 11, 5, 10, 10, 10, 11, 17, 7, 11, // D
        5, 10, 10, 18, 11, 11, 7, 11, 5, 5, 10, 4, 11, 17, 7, 11, // E
        5, 10, 10, 4, 11, 11, 7, 11, 5, 5, 10, 4, 11, 17, 7, 11, // F
    };

    // pring registers for debugging
    fn print_registers(self: *Intel8080) void {
        std.debug.print("A: {X:02} B: {X:02} C: {X:02} D: {X:02} E: {X:02} H: {X:02} L: {X:02} SP: {X:04} PC: {X:04} FLAGS: {X:02}\n", .{ self.a, self.b, self.c, self.d, self.e, self.h, self.l, self.sp, self.pc, self.flags.byte });
    }

    // gets the contents of a single register (0-7)
    fn get_register(self: *Intel8080, reg: u8) u8 {
        switch (reg) {
            7 => return self.a,
            0 => return self.b,
            1 => return self.c,
            2 => return self.d,
            3 => return self.e,
            4 => return self.h,
            5 => return self.l,
            else => return 0, // Invalid register
        }
    }

    // sets the contents of a single register (0-7)
    fn set_register(self: *Intel8080, reg: u8, value: u8) void {
        switch (reg) {
            7 => self.a = value,
            0 => self.b = value,
            1 => self.c = value,
            2 => self.d = value,
            3 => self.e = value,
            4 => self.h = value,
            5 => self.l = value,
            else => {}, // Invalid register, do nothing
        }
    }

    // gets a word from 2 bytes (hi, lo)
    fn get_word_from_bytes(hi: u8, lo: u8) u16 {
        return @as(u16, hi) << 8 | @as(u16, lo);
    }

    // gets the contents of a register pair (0-3)
    pub fn get_register_pair(self: *Intel8080, reg_pair: Intel8080.register_pairs) u16 {
        return switch (reg_pair) {
            .B => get_word_from_bytes(self.b, self.c),
            .D => get_word_from_bytes(self.d, self.e),
            .H => get_word_from_bytes(self.h, self.l),
            .PSW => get_word_from_bytes(self.a, self.flags.byte),
        };
    }

    // sets the contents of a register pair (0-3)
    fn set_register_pair(self: *Intel8080, reg_pair: Intel8080.register_pairs, value: u16) void {
        //const temp = value >> 8;
        const byte1: u8 = @truncate(value >> 8);
        const byte2: u8 = @truncate(value & 0x00FF);
        switch (reg_pair) {
            .B => {
                self.b = byte1;
                self.c = byte2;
            },
            .D => {
                self.d = byte1;
                self.e = byte2;
            },
            .H => {
                self.h = byte1;
                self.l = byte2;
            },
            .PSW => {
                self.a = byte1;
                self.flags.byte = byte2;
            },
        }
    }

    // sets the zero, sign and parity flags based on a value
    fn set_zsp(self: *Intel8080, value: u8) void {
        self.flags.bits.zero = if (value == 0) 1 else 0;
        self.flags.bits.sign = if (value & 0x80 != 0) 1 else 0;
        self.flags.bits.parity = if (@popCount(value) & 1 == 0) 1 else 0; // Even parity
    }

    // ask for an interrupt
    pub fn interrupt(self: *Intel8080, instruction: u8) void {
        self.interrupt_pending = true;
        self.interrupt_vector = instruction;
    }

    // Stack Ops

    // push word onto the stack
    fn push(self: *Intel8080, word: u16, ram: []u8) void {
        // Pushes a word onto the stack
        ram[self.sp - 1] = @truncate(word >> 8);
        ram[self.sp - 2] = @truncate(word & 0x00FF);
        self.sp -= 2; // Decrement stack pointer
    }

    // pops word from the stack
    fn pop(self: *Intel8080, ram: []u8) u16 {
        // Pops a word from the stack
        const byte1 = ram[self.sp + 1];
        const byte2 = ram[self.sp];
        self.sp += 2;
        return get_word_from_bytes(byte1, byte2);
    }

    // calc carry between bit_no and bit_no - 1
    // when doinf a + b + cy
    fn calc_carry(bit_no: u4, a: u8, b: u8, cy: u8) u1 {
        const result = @as(u16, a) + @as(u16, b) + @as(u16, cy);
        const carry = (result ^ a ^ b) &
            (@as(u16, 0x01) << bit_no); // when there's a <<, the LHS must be large
        // enough to accomodate the RHS
        return if (carry != 0) 1 else 0;
    }

    //
    // Opcode functions
    //

    // LXI
    fn LXI(self: *Intel8080, opcode: Intel8080.opcodes, ram: []u8) void {
        const lo = ram[self.pc];
        const hi = ram[self.pc + 1];
        const word = get_word_from_bytes(hi, lo);

        switch (opcode) {
            .LXI_B => self.set_register_pair(.B, word),
            .LXI_D => self.set_register_pair(.D, word),
            .LXI_H => self.set_register_pair(.H, word),
            .LXI_SP => self.sp = word,
            else => {}, // Invalid opcode for LXI
        }
        self.pc += 2; // Increment PC by 2 to account for the immediate data
    }

    // INR
    fn INR(self: *Intel8080, instruction: u8, ram: []u8) void {
        const reg: u4 = @truncate((instruction & 0b00111000) >> 3);
        var value: u8 = undefined;

        if (reg == 0x06) { // increment memory
            const addr = self.get_register_pair(.H);
            value = ram[addr];
            value = value +% 1;
            ram[addr] = value;
        } else { // increment register
            value = self.get_register(reg);
            value = value +% 1;
            self.set_register(reg, value);
        }
        // Set flags
        self.set_zsp(value);
        self.flags.bits.aux_carry = if ((value & 0x0F) == 0x00) 1 else 0; // Check for half-carry
    }

    // DCR
    fn DCR(self: *Intel8080, instruction: u8, ram: []u8) void {
        const reg: u4 = @truncate((instruction & 0b00111000) >> 3);
        var value: u8 = undefined;

        if (reg == 0x06) { // decrement memory
            const addr = self.get_register_pair(.H);
            value = ram[addr];
            value = value -% 1;
            ram[addr] = value;
        } else { // decrement register
            value = self.get_register(reg);
            value = value -% 1;
            self.set_register(reg, value);
        }
        // Set flags
        self.set_zsp(value);
        self.flags.bits.aux_carry = if ((value & 0x0F) == 0x0F) 1 else 0; // Check for half-borrow
    }

    // MVI
    fn MVI(self: *Intel8080, instruction: u8, ram: []u8) void {
        const reg: u4 = @truncate((instruction & 0b00111000) >> 3);
        const value: u8 = ram[self.pc];

        if (reg == 0x06) { // MVI to memory
            const addr = self.get_register_pair(.H);
            ram[addr] = value;
        } else { // MVI to register
            self.set_register(reg, value);
        }
        self.pc += 1; // Increment PC by 1 to account for the operand data
    }

    // DAD
    fn DAD(self: *Intel8080, opcode: Intel8080.opcodes) void {
        const value = switch (opcode) {
            .DAD_B => self.get_register_pair(.B),
            .DAD_D => self.get_register_pair(.D),
            .DAD_H => self.get_register_pair(.H),
            .DAD_SP => self.sp,
            else => unreachable,
        };
        const H = self.get_register_pair(.H);
        const result = H +% value;
        self.set_register_pair(.H, result);
        self.flags.bits.carry = if (result < H) 1 else 0; // Set carry if overflow
    }

    // MOV
    fn MOV(self: *Intel8080, instruction: u8, ram: []u8) void {
        const dst: u4 = @truncate((instruction & 0b00111000) >> 3);
        const src: u4 = @truncate(instruction & 0b00000111);

        if (dst == src)
            return;

        if (src == 0b110 or dst == 0b110) { // MOV using memory
            // get address from H reg pair
            const H = self.get_register_pair(.H);
            if (src == 0b110) {
                // MOV r, M
                self.set_register(dst, ram[H]);
            } else if (dst == 0b110) {
                // MOV M, r
                ram[H] = self.get_register(src);
            }
        } else // MOV r, r
        self.set_register(dst, self.get_register(src));
    }

    // ADD and ADC
    fn ADD_ADC(self: *Intel8080, instruction: u8, ram: []u8, with_carry: bool) void {
        const reg: u4 = @truncate(instruction & 0b00000111);
        var value: u8 = undefined;

        if (reg == 0x06) { // ADD/ADC memory
            const addr = self.get_register_pair(.H);
            value = ram[addr];
        } else { // ADD/ADC register
            value = self.get_register(reg);
        }

        const carry_in: u8 = if (with_carry) self.flags.bits.carry else 0;
        const result = @as(u16, self.a) + @as(u16, value) + @as(u16, carry_in);
        const result_u8: u8 = @truncate(result);

        // Set flags
        self.set_zsp(result_u8);
        self.flags.bits.carry = Intel8080.calc_carry(8, self.a, value, carry_in);
        self.flags.bits.aux_carry = Intel8080.calc_carry(4, self.a, value, carry_in);

        self.a = result_u8; // Store result in accumulator
    }

    // SUB and SBB
    fn SUB_SBB(self: *Intel8080, instruction: u8, ram: []u8, with_borrow: bool) void {
        const reg: u4 = @truncate(instruction & 0b00000111);
        var value: u8 = undefined;

        if (reg == 0x06) { // SUB/SBB memory
            const addr = self.get_register_pair(.H);
            value = ram[addr];
        } else { // SUB/SBB register
            value = self.get_register(reg);
        }

        const borrow_in: u8 = if (with_borrow) self.flags.bits.carry else 0;
        const result = @as(u16, self.a) -% @as(u16, value) -% @as(u16, borrow_in);
        const result_u8: u8 = @truncate(result);

        // Set flags
        self.set_zsp(result_u8);
        self.flags.bits.carry = if (result > 0xFF) 1 else 0; // Set carry if underflow
        self.flags.bits.aux_carry = if (((self.a & 0x0F) < ((value & 0x0F) + borrow_in))) 1 else 0; // Correct half-borrow calculation

        self.a = result_u8; // Store result in accumulator
    }

    // ANA, XRA, ORA
    fn ANA_XRA_ORA(self: *Intel8080, instruction: u8, ram: []u8) void {
        const reg: u4 = @truncate(instruction & 0b00000111);
        var value: u8 = undefined;

        if (reg == 0x06) { // Memory operation
            const addr = self.get_register_pair(.H);
            value = ram[addr];
        } else { // Register operation
            value = self.get_register(reg);
        }

        const opcode = @as(Intel8080.opcodes, @enumFromInt(instruction));
        switch (opcode) {
            .ANA_B, .ANA_C, .ANA_D, .ANA_E, .ANA_H, .ANA_L, .ANA_M, .ANA_A => {
                self.a = self.a & value; // AND operation
                self.flags.bits.aux_carry = if ((self.a & 0x0F) < (value & 0x0F)) 1 else 0; // Check for half-carry
            },
            .XRA_B, .XRA_C, .XRA_D, .XRA_E, .XRA_H, .XRA_L, .XRA_M, .XRA_A => {
                self.a = self.a ^ value; // XOR operation
                self.flags.bits.aux_carry = 0; // No half-carry for XOR
            },
            .ORA_B, .ORA_C, .ORA_D, .ORA_E, .ORA_H, .ORA_L, .ORA_M, .ORA_A => {
                self.a = self.a | value; // OR operation
                self.flags.bits.aux_carry = 0; // No half-carry for OR
            },
            else => unreachable,
        }
        // Set flags
        self.set_zsp(self.a);
        self.flags.bits.carry = 0; // No carry for logical operations
    }

    // DAA
    fn DAA(self: *Intel8080) void {
        const lsb = self.a & 0x0F;
        const a_orig = self.a;
        var correction: u8 = 0;

        if (lsb > 9 or self.flags.bits.aux_carry == 1) {
            self.a += 6;
            correction += 6;
        }

        const msb = (self.a & 0xF0) >> 4;
        if (msb > 9 or self.flags.bits.carry == 1) {
            self.a = (self.a +% 0x60);
            correction += 0x60;
        }

        // set flags
        self.set_zsp(self.a);
        self.flags.bits.carry = calc_carry(8, a_orig, correction, 0);
        self.flags.bits.aux_carry = calc_carry(4, a_orig, correction, 0);
    }

    // ********************************************************************************
    //
    //      EEEEEEEEEEEEEEEEE XXXXX     XXXXX EEEEEEEEEEEEEEEEE CCCCCCCCCCCCCCC
    //      EEEEEEEEEEEEEEEEE XXXX      XXXXX EEEEEEEEEEEEEEEEE CCCCCCCCCCCCCCC
    //      EEE                XXX       XXX  EEE               CCC
    //      EEEEEEEEEEE         XXX   XXX     EEEEEEEEEE        CCC
    //      EEEEEEEEEEE           XXXXX       EEEEEEEEEE        CCC
    //      EEE                 XXX    XXX    EEE               CCC
    //      EEEEEEEEEEEEEEEEE  XXXX     XXXX  EEEEEEEEEEEEEE    CCCCCCCCCCCCCCC
    //      EEEEEEEEEEEEEEEEE XXXXX     XXXXX EEEEEEEEEEEEEEE   CCCCCCCCCCCCCCC

    //-----------------------------------------------------------------------------
    // execute instruction at pc
    //
    pub fn execute_instruction(self: *Intel8080, ram: []u8, debug: bool) void {
        var instruction: u8 = undefined;

        if (self.running_state != .running)
            return;

        //
        // handle interrupts
        //
        if (self.interrupt_req == 1 and self.inte == 1) {
            self.interrupt_req = 0; // lower interrupt req signal
            self.inte = 0; // reset INTE
            instruction = self.interrupt_vector; // opcode sent by interrupt req
        } else {
            // regular CPU cycle
            instruction = ram[self.pc];
            self.pc += 1;
        }

        if (debug == true) {
            std.debug.print("{X:04}: {s} {X:02} {X:02} \t\t inst={X:02}, cf={d}\n", .{ self.pc - 1, Intel8080.opcode_names[instruction], ram[self.pc], ram[self.pc + 1], instruction, self.flags.bits.carry });
        }

        // decode instructions
        const opcode = @as(Intel8080.opcodes, @enumFromInt(instruction));
        switch (opcode) {
            .NOP, .NOP0, .NOP1, .NOP2, .NOP3, .NOP4, .NOP5, .NOP6 =>
            // do nothing
            _ = 1,

            // LXI: Load register pair immediate
            .LXI_B, .LXI_D, .LXI_H, .LXI_SP => {
                self.LXI(opcode, ram[0..]);
            },

            // STAX: stores A into address in register pair B or D
            .STAX_B => {
                const addr = self.get_register_pair(.B);
                ram[addr] = self.a;
            },
            .STAX_D => {
                const addr = self.get_register_pair(.D);
                ram[addr] = self.a;
            },

            // INX: increment register pair
            .INX_B => {
                const val = self.get_register_pair(.B) +% 1;
                self.set_register_pair(.B, val);
            },
            .INX_D => {
                const val = self.get_register_pair(.D) +% 1;
                self.set_register_pair(.D, val);
            },
            .INX_H => {
                const val = self.get_register_pair(.H) +% 1;
                self.set_register_pair(.H, val);
            },
            .INX_SP => {
                self.sp = self.sp +% 1;
            },

            // INR: increment register
            .INR_B, .INR_D, .INR_H, .INR_M, .INR_C, .INR_E, .INR_L, .INR_A => self.INR(instruction, ram[0..]),

            // DCR: decrement register
            .DCR_B, .DCR_D, .DCR_H, .DCR_M, .DCR_C, .DCR_E, .DCR_L, .DCR_A => self.DCR(instruction, ram[0..]),

            // MVI: Move immediate to register
            .MVI_B, .MVI_C, .MVI_D, .MVI_E, .MVI_H, .MVI_L, .MVI_M, .MVI_A => {
                self.MVI(instruction, ram[0..]);
            },

            // Rotate instructions
            // RLC: Rotate accumulator left w/ carry
            .RLC => {
                const msb = (self.a & 0x80) >> 7;
                self.a = (self.a << 1) | msb;
                self.flags.bits.carry = @truncate(msb);
            },

            // RAL: Rotate accumulator left through carry
            .RAL => {
                const msb = (self.a & 0x80) >> 7;
                self.a = (self.a << 1) | self.flags.bits.carry;
                self.flags.bits.carry = @truncate(msb);
            },

            // RRC: Rotate accumulator right w/ carry
            .RRC => {
                const lsb = self.a & 0x01;
                self.a = (self.a >> 1) | (lsb << 7);
                self.flags.bits.carry = @truncate(lsb);
            },

            // RAR: Rotate accumulator right through carry
            .RAR => {
                const lsb = self.a & 0x01;
                self.a = (self.a >> 1) | (@as(u8, self.flags.bits.carry) << 7);
                self.flags.bits.carry = @truncate(lsb);
            },

            // DAA:
            .DAA => {
                self.DAA();
            },

            // LDAX
            .LDAX_B => {
                const addr = self.get_register_pair(.B);
                self.a = ram[addr];
            },

            .LDAX_D => {
                const addr = self.get_register_pair(.D);
                self.a = ram[addr];
            },

            // DCX: Decrement register pair
            .DCX_B, .DCX_D, .DCX_H, .DCX_SP => {
                if (opcode == .DCX_SP) {
                    self.sp = self.sp -% 1;
                } else {
                    const reg_pair: Intel8080.register_pairs = switch (opcode) {
                        .DCX_B => .B,
                        .DCX_D => .D,
                        .DCX_H => .H,
                        else => unreachable,
                    };
                    const value = self.get_register_pair(reg_pair);
                    self.set_register_pair(reg_pair, value -% 1);
                }
            },

            // DAD: Double add register pair to HL
            .DAD_B, .DAD_D, .DAD_H, .DAD_SP => {
                self.DAD(opcode);
            },

            // SHLD: Store H and L into memory address provided in operands
            .SHLD => {
                const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                ram[addr] = self.l;
                ram[addr + 1] = self.h;
                self.pc += 2;
            },

            // LHLD: Load H and L from memory address provided in operands
            .LHLD => {
                const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                self.l = ram[addr];
                self.h = ram[addr + 1];
                self.pc += 2;
            },

            // CMA: Complement accumulator
            .CMA => {
                self.a = ~self.a;
            },

            // STA: Store accumulator direct
            .STA => {
                const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                ram[addr] = self.a;
                self.pc += 2;
            },

            // STC: Set carry flag
            .STC => {
                self.flags.bits.carry = 1;
            },

            // CMC: Complement carry flag
            .CMC => {
                self.flags.bits.carry = if (self.flags.bits.carry == 0) 1 else 0;
            },

            // LDA: Load accumulator direct
            .LDA => {
                const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                self.a = ram[addr];
                self.pc += 2;
            },

            //HALT
            .HLT => self.running_state = .halted,
            // MOV instructions
            // zig fmt: off
            .MOV_BB, .MOV_BC, .MOV_BD, .MOV_BE, .MOV_BH, .MOV_BL, .MOV_BM, .MOV_BA, .MOV_CB, .MOV_CC, .MOV_CD, .MOV_CE, .MOV_CH, .MOV_CL, .MOV_CM, .MOV_CA,
            .MOV_DB, .MOV_DC, .MOV_DD, .MOV_DE, .MOV_DH, .MOV_DL, .MOV_DM, .MOV_DA, .MOV_EB, .MOV_EC, .MOV_ED, .MOV_EE, .MOV_EH, .MOV_EL, .MOV_EM, .MOV_EA,
            .MOV_HB, .MOV_HC, .MOV_HD, .MOV_HE, .MOV_HH, .MOV_HL, .MOV_HM, .MOV_HA, .MOV_LB, .MOV_LC, .MOV_LD, .MOV_LE, .MOV_LH, .MOV_LL, .MOV_LM, .MOV_LA,
            .MOV_MB, .MOV_MC, .MOV_MD, .MOV_ME, .MOV_MH, .MOV_ML, .MOV_MA, .MOV_AB, .MOV_AC, .MOV_AD, .MOV_AE, .MOV_AH, .MOV_AL, .MOV_AM, .MOV_AA => {
                self.MOV(instruction, ram[0..]);
            },
            // zig fmt: on

            // ADD & ADC: Add with or without carry
            .ADD_B, .ADD_C, .ADD_D, .ADD_E, .ADD_H, .ADD_L, .ADD_M, .ADD_A => self.ADD_ADC(instruction, ram[0..], false),
            .ADC_B, .ADC_C, .ADC_D, .ADC_E, .ADC_H, .ADC_L, .ADC_M, .ADC_A => self.ADD_ADC(instruction, ram[0..], true),

            // SUB AND SBB: subtract with or without borrow
            .SUB_B, .SUB_C, .SUB_D, .SUB_E, .SUB_H, .SUB_L, .SUB_M, .SUB_A => self.SUB_SBB(instruction, ram[0..], false),
            .SBB_B, .SBB_C, .SBB_D, .SBB_E, .SBB_H, .SBB_L, .SBB_M, .SBB_A => self.SUB_SBB(instruction, ram[0..], true),

            // ANA, XRA, ORA: Logical operations
            .ANA_B, .ANA_C, .ANA_D, .ANA_E, .ANA_H, .ANA_L, .ANA_M, .ANA_A, .XRA_B, .XRA_C, .XRA_D, .XRA_E, .XRA_H, .XRA_L, .XRA_M, .XRA_A, .ORA_B, .ORA_C, .ORA_D, .ORA_E, .ORA_H, .ORA_L, .ORA_M, .ORA_A => self.ANA_XRA_ORA(instruction, ram[0..]),

            // RETs: return from subroutine
            .RNZ, .RNC, .RPO, .RPE, .RP, .RZ, .RC, .RM, .RET, .RET1 => {
                const condition = switch (opcode) {
                    .RNZ => self.flags.bits.zero == 0,
                    .RNC => self.flags.bits.carry == 0,
                    .RPO => self.flags.bits.parity == 0,
                    .RPE => self.flags.bits.parity == 1,
                    .RP => self.flags.bits.sign == 0,
                    .RZ => self.flags.bits.zero == 1,
                    .RC => self.flags.bits.carry == 1,
                    .RM => self.flags.bits.sign == 1,
                    .RET => true, // unconditional return
                    else => unreachable,
                };
                if (condition) {
                    const return_address = self.pop(ram[0..]);
                    self.pc = return_address;
                }
            },

            // POPs: Pops stack into register pairs
            .POP_B, .POP_D, .POP_H, .POP_PSW => {
                const reg_pair: Intel8080.register_pairs = switch (opcode) {
                    .POP_B => .B,
                    .POP_D => .D,
                    .POP_H => .H,
                    .POP_PSW => .PSW,
                    else => unreachable,
                };
                const value = self.pop(ram[0..]);
                self.set_register_pair(reg_pair, value);
            },

            // JMPs: Jump to address
            .JMP, .JNZ, .JZ, .JNC, .JC, .JPO, .JPE, .JP, .JM, .JMP1 => {
                const condition = switch (opcode) {
                    .JMP => true,
                    .JNZ => self.flags.bits.zero == 0,
                    .JZ => self.flags.bits.zero == 1,
                    .JNC => self.flags.bits.carry == 0,
                    .JC => self.flags.bits.carry == 1,
                    .JPO => self.flags.bits.parity == 0,
                    .JPE => self.flags.bits.parity == 1,
                    .JP => self.flags.bits.sign == 0,
                    .JM => self.flags.bits.sign == 1,
                    else => unreachable,
                };
                if (condition) {
                    const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                    self.pc = addr;
                } else {
                    self.pc += 2; // Increment PC by 2 to skip the jump address
                }
            },
            // CALLs: Call subroutine
            .CALL, .CNZ, .CZ, .CNC, .CC, .CPO, .CPE, .CP, .CM, .CALL1, .CALL2, .CALL3 => {
                const condition = switch (opcode) {
                    .CALL => true,
                    .CNZ => self.flags.bits.zero == 0,
                    .CZ => self.flags.bits.zero == 1,
                    .CNC => self.flags.bits.carry == 0,
                    .CC => self.flags.bits.carry == 1,
                    .CPO => self.flags.bits.parity == 0,
                    .CPE => self.flags.bits.parity == 1,
                    .CP => self.flags.bits.sign == 0,
                    .CM => self.flags.bits.sign == 1,
                    else => unreachable,
                };
                if (condition) {
                    const addr = get_word_from_bytes(ram[self.pc + 1], ram[self.pc]);
                    self.push(self.pc + 2, ram[0..]); // Push next instruction address
                    self.pc = addr; // Jump to subroutine address
                } else {
                    self.pc += 2; // Increment PC by 2 to skip the call address
                }
            },

            // ACI, ADI: Add immediate with/without carry
            .ACI, .ADI => {
                const value: u8 = ram[self.pc];
                const carry_in: u8 = if (opcode == .ACI) self.flags.bits.carry else 0;
                const result = @as(u16, self.a) + @as(u16, value) + @as(u16, carry_in);
                const result_u8: u8 = @truncate(result);
                // Set flags
                self.set_zsp(result_u8);
                self.flags.bits.carry = Intel8080.calc_carry(8, self.a, value, carry_in);
                self.flags.bits.aux_carry = Intel8080.calc_carry(4, self.a, value, carry_in);
                self.a = result_u8; // Store result in accumulator
                self.pc += 1; // Increment PC by 1 to account for the immediate data
            },

            // RSTs: push PC to stack and jump to address 0000000000EXP00
            .RST_0, .RST_1, .RST_2, .RST_3, .RST_4, .RST_5, .RST_6, .RST_7 => {
                const addr = get_word_from_bytes(0x0000, instruction & 0b00111000);
                self.push(self.pc, ram[0..]); // Push current PC to stack
                self.pc = addr; // Jump to RST vector address
            },

            // PUSH: Push register pair onto stack
            .PUSH_B, .PUSH_D, .PUSH_H, .PUSH_PSW => {
                const reg_pair: Intel8080.register_pairs = switch (opcode) {
                    .PUSH_B => .B,
                    .PUSH_D => .D,
                    .PUSH_H => .H,
                    .PUSH_PSW => .PSW,
                    else => unreachable,
                };
                const value = self.get_register_pair(reg_pair);
                self.push(value, ram[0..]); // Push register pair onto stack
            },

            // SUI, SBI: Subtract immediate with/without borrow
            .SUI, .SBI => {
                const value: u8 = ram[self.pc];
                const borrow_in: u8 = if (opcode == .SBI) self.flags.bits.carry else 0;
                const result = @as(u16, self.a) -% @as(u16, value) -% @as(u16, borrow_in);
                const result_u8: u8 = @truncate(result);
                // Set flags
                self.set_zsp(result_u8);
                self.flags.bits.carry = if (result > 0xFF) 1 else 0; // Set carry if underflow
                self.flags.bits.aux_carry = if (((self.a & 0x0F) < (value & 0x0F) + borrow_in)) 1 else 0; // Check for half-borrow
                self.a = result_u8; // Store result in accumulator
                self.pc += 1; // Increment PC by 1 to account for the immediate data
            },

            // XTHL: Exchange stack top with HL
            .XTHL => {
                const hl_value = self.get_register_pair(.H);
                const sp_value = self.pop(ram[0..]); // Pop top of stack
                self.set_register_pair(.H, sp_value); // Set HL to stack top
                self.push(hl_value, ram[0..]); // Push old HL value onto stack
            },

            // ANI, ORI, XRI: Logical operations with immediate
            .ANI, .ORI, .XRI => {
                const value: u8 = ram[self.pc];
                switch (opcode) {
                    .ANI => {
                        self.a = self.a & value; // AND operation
                        self.flags.bits.aux_carry = if ((self.a & 0x0F) < (value & 0x0F)) 1 else 0; // Check for half-carry
                    },
                    .ORI => {
                        self.a = self.a | value; // OR operation
                        self.flags.bits.aux_carry = 0; // No half-carry for OR
                    },
                    .XRI => {
                        self.a = self.a ^ value; // XOR operation
                        self.flags.bits.aux_carry = 0; // No half-carry for XOR
                    },
                    else => unreachable,
                }
                // Set flags
                self.set_zsp(self.a);
                self.flags.bits.carry = 0; // No carry for logical operations
                self.pc += 1; // Increment PC by 1 to account for the immediate data
            },

            // PCHL: Load program counter from HL
            .PCHL => {
                self.pc = self.get_register_pair(.H); // Set PC to value in HL
            },

            .XCHG => {
                const hl_value = self.get_register_pair(.H);
                const de_value = self.get_register_pair(.D);
                self.set_register_pair(.H, de_value);
                self.set_register_pair(.D, hl_value);
            },

            // DI: Disable interrupts
            .DI => {
                self.inte = 0;
                self.interrupt_req = 0; // Reset interrupt flag
            },

            // EI: Enable interrupts
            .EI => {
                self.inte = 1;
                self.interrupt_req = 1; // Set interrupt flag
            },

            // SPHL: Set stack pointer to HL
            .SPHL => {
                self.sp = self.get_register_pair(.H); // Set stack pointer to value in HL
            },

            // CMP CPI: Compare immediate with accumulator
            .CPI, .CMP_A, .CMP_B, .CMP_C, .CMP_D, .CMP_E, .CMP_H, .CMP_L, .CMP_M => {
                var value: u8 = undefined;
                if (opcode == .CPI) {
                    // CPI: Compare immediate
                    value = ram[self.pc];
                    self.pc += 1; // Increment PC by 1 for CPI
                } else {
                    // CMP: Compare register or memory
                    const reg: u4 = @truncate(instruction & 0b00000111);
                    if (reg == 0x06) { // CMP M
                        const addr = self.get_register_pair(.H);
                        value = ram[addr];
                    } else { // CMP r
                        value = self.get_register(reg);
                    }
                }
                const result = @as(u16, self.a) -% @as(u16, value);
                // Set flags
                self.set_zsp(@truncate(result));
                self.flags.bits.carry = if (result > 0xFF) 1 else 0; // Set carry if underflow
                self.flags.bits.aux_carry = if (((self.a & 0x0F) < (value & 0x0F))) 1 else 0; // Check for half-borrow
            },

            // OUT
            .OUT_ => {
                const device = ram[self.pc];
                self.pc += 1;
                self.ports_out[device] = self.a;
            },

            .IN_ => {
                const device = ram[self.pc];
                self.pc += 1;
                self.a = self.ports_in[device];
            },
        }
        // add how many CPU cycles this instruction took
        self.cycles += opcode_cycles[instruction];
    }
};

fn test_debug() void {
    var in = [256]u8;
    var out = [256]u8;

    var cpu = Intel8080{ .ports_in = &in, .ports_out = &out };
    var ram: [0xFFFF]u8 = undefined;

    // Test CMP with register
    cpu.a = 0x9b; // Set accumulator to 0x9b
    ram[0] = @intFromEnum(Intel8080.opcodes.DAA);

    cpu.execute_instruction(ram[0..], true);
    std.debug.print("zero: {}, carry: {}, aux: {}", .{ cpu.flags.bits.zero, cpu.flags.bits.carry, cpu.flags.bits.aux_carry });
}

pub fn main() void {
    test_debug();
}

//************************************************************************************************************/

// TTTTTTTTTTTTTTTTT EEEEEEEEEEEEEEEEE SSSSSSSSSSSSSSSSS TTTTTTTTTTTTTTTTT
// TTTTTTTTTTTTTTTTT EEEEEEEEEEEEEEEEE SSSSSSSSSSSSSSSSS TTTTTTTTTTTTTTTTT
//       TTT         EEE               SSS                     TTT
//       TTT         EEEEEEEEEE        SSSSSSSSSSSSSS          TTT
//       TTT         EEEEEEEEEE             SSSSSS             TTT
//       TTT         EEE                         SSS           TTT
//       TTT         EEEEEEEEEEEEEE    SSSSSSSSSSSSSSS         TTT
//       TTT         EEEEEEEEEEEEEE    SSSSSSSSSSSSSSS         TTT

//************************************************************************************************************/

test "IN, OUT" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;

    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };
    var ram: [0xFFFF]u8 = undefined;

    cpu.a = 0x9b; // Set accumulator to 0x9b
    ram[0] = @intFromEnum(Intel8080.opcodes.OUT_);
    ram[1] = 2; // write to device #2

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(out[2] == 0x9b);

    in[5] = 5;
    ram[2] = @intFromEnum(Intel8080.opcodes.IN_);
    ram[3] = 5; // read from device #5

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 5);
}

test "DAA" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };
    var ram: [0xFFFF]u8 = undefined;

    // Test CMP with register
    cpu.a = 0x9b; // Set accumulator to 0x9b
    ram[0] = @intFromEnum(Intel8080.opcodes.DAA);

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 1);
    try std.testing.expect(cpu.flags.bits.zero == 0); // Not zero
    try std.testing.expect(cpu.flags.bits.carry == 1); // No carry
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Half-borrow generated
    try std.testing.expect(cpu.flags.bits.sign == 0);
    try std.testing.expect(cpu.flags.bits.parity == 0);
}

test "CMP, CPI" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test CMP with register
    cpu.a = 0x20; // Set accumulator to 0x20
    cpu.b = 0x15; // Set B register to 0x15
    ram[0] = @intFromEnum(Intel8080.opcodes.CMP_B); // CMP B

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.flags.bits.zero == 0); // Not zero
    try std.testing.expect(cpu.flags.bits.carry == 0); // No carry
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Half-borrow generated

    // Test CMP with memory
    cpu.a = 12; // Reset accumulator to 0x40
    ram[1] = @intFromEnum(Intel8080.opcodes.CMP_M); // CMP M
    ram[0x20] = 15; // Set memory location to 0x50
    cpu.h = 0x00; // Set HL register to point to memory location
    cpu.l = 0x20; // Memory address 0x0001
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.flags.bits.zero == 0); // Not zero
    try std.testing.expect(cpu.flags.bits.carry == 1); // Carry generated (40 < 50)
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Half-borrow generated

    // Test CPI with immediate value
    cpu.a = 0x30; // Reset accumulator to 0x30
    ram[2] = @intFromEnum(Intel8080.opcodes.CPI); // CPI
    ram[3] = 0x25; // Immediate value to compare with

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.flags.bits.zero == 0); // Not zero
    try std.testing.expect(cpu.flags.bits.carry == 0); // Carry generated (30 < 25)
}

test "ANI,ORI,XRI" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test ANI
    cpu.a = 0xF0; // Set accumulator to 0xF0
    ram[0] = @intFromEnum(Intel8080.opcodes.ANI); // ANI
    ram[1] = 0x0F; // Immediate value to AND with

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x00); // 0xF0 & 0x0F = 0x00
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Half-carry generated
    try std.testing.expect(cpu.flags.bits.zero == 1);
    try std.testing.expect(cpu.flags.bits.sign == 0);

    // Test ORI
    cpu.a = 0xF0; // Reset accumulator to 0xF0
    ram[2] = @intFromEnum(Intel8080.opcodes.ORI); // ORI
    ram[3] = 0x0F; // Immediate value to OR with

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0xFF); // 0xF0 | 0x0F = 0xFF
    try std.testing.expect(cpu.flags.bits.aux_carry == 0); // Half-carry 0 for OR

    // Test XRI
    cpu.a = 0xF0; // Reset accumulator to 0xF0
    ram[4] = @intFromEnum(Intel8080.opcodes.XRI); // XRI
    ram[5] = 0xFF; // Immediate value to XOR with

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x0F); // 0xF0 ^ 0xFF = 0x0F
    try std.testing.expect(cpu.flags.bits.aux_carry == 0); // Half-carry 0 for XRI
}

test "XTHL" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Initialize registers
    cpu.h = 0x12;
    cpu.l = 0x34;
    cpu.sp = 0x0FFE; // Initialize stack pointer to top of memory

    // Push initial HL value onto stack
    cpu.push(0x3456, ram[0..]); // Push HL (0x1234) onto stack

    // Execute XTHL instruction
    ram[0] = @intFromEnum(Intel8080.opcodes.XTHL); // XTHL
    cpu.execute_instruction(ram[0..], true);

    // Check if HL was exchanged with the top of the stack
    try std.testing.expect(cpu.get_register_pair(.H) == 0x3456);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x1234); // Stack pointer should remain
}

test "SUI, SBI" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test SUI
    cpu.a = 0x20; // Set accumulator to 0x20
    ram[0] = @intFromEnum(Intel8080.opcodes.SUI); // SUI
    ram[1] = 0x05; // Immediate value to subtract

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x1B); // 0x20 - 0x05 = 0x1B
    try std.testing.expect(cpu.flags.bits.carry == 0); // No carry generated

    // Test SBI
    cpu.a = 0x30; // Reset accumulator to 0x30
    cpu.flags.bits.carry = 0; // Reset carry flag
    ram[2] = @intFromEnum(Intel8080.opcodes.SBI); // SBI
    ram[3] = 0x03; // Immediate value to subtract

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x2D); // 0x30 - 0x03 = 0x2D

    // Test SBI with carry
    cpu.a = 0x10; // Reset accumulator to 0x10
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[4] = @intFromEnum(Intel8080.opcodes.SBI); // SBI with carry
    ram[5] = 0x08; // Immediate value to subtract
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x07); // 0x10 - 0x08 - carry(1) = 0x07
    try std.testing.expect(cpu.flags.bits.carry == 0); // No carry generated
}

test "PUSH" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test PUSH B
    cpu.b = 0x12;
    cpu.c = 0x34;
    cpu.sp = 0x0FFE; // Initialize stack pointer to top of memory
    ram[0] = @intFromEnum(Intel8080.opcodes.PUSH_B); // PUSH B

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x1234); // B high byte

    // Test PUSH D
    cpu.d = 0x56;
    cpu.e = 0x78;
    ram[1] = @intFromEnum(Intel8080.opcodes.PUSH_D); // PUSH D

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x5678); // D high byte
}

test "RST" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test RST 0
    cpu.sp = 0x0FFE; // Initialize stack pointer to top of memory
    cpu.pc = 0x1000; // Set program counter to a known value
    ram[0x1000] = @intFromEnum(Intel8080.opcodes.RST_0); // RST 0

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x0000); // PC should jump to 0x0000

    // Test RST 1
    cpu.pc = 0x2000; // Reset program counter
    ram[0x2000] = @intFromEnum(Intel8080.opcodes.RST_1); // RST 1

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x0008); // PC should jump to 0x0008
}

test "ACI, ADI" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test ACI
    cpu.a = 0x10; // Set accumulator to 0x10
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[0] = @intFromEnum(Intel8080.opcodes.ACI); // ACI
    ram[1] = 0x05; // Immediate value to add

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x16); // 0x10 + 0x05 + carry(1) = 0x16
    try std.testing.expect(cpu.flags.bits.carry == 0); // No carry generated

    // Test ADI
    cpu.a = 0x20; // Reset accumulator to 0x20
    ram[2] = @intFromEnum(Intel8080.opcodes.ADI); // ADI
    ram[3] = 0x03; // Immediate value to add

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x23); // 0x20 + 0x03 = 0x23
}

test "CALLs" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    cpu.sp = 0xFFFF; // Initialize stack pointer to top of memory
    // Test unconditional call
    ram[0] = @intFromEnum(Intel8080.opcodes.CALL); // CALL
    ram[1] = 0x12; // Low byte of address
    ram[2] = 0x34; // High byte of address
    cpu.pc = 0;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x3412);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x0003); // Stack pointer should be decremented

    // Test conditional calls
    cpu.pc = 5;
    cpu.flags.bits.zero = 1; // Set zero flag for CZ
    ram[5] = @intFromEnum(Intel8080.opcodes.CZ); // CZ
    ram[6] = 0x56; // Low byte of address
    ram[7] = 0x78; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x7856);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x0008); // Stack pointer should be decremented

    cpu.pc = 0x0600; // Reset program counter for next test
    cpu.flags.bits.zero = 0; // Reset zero flag for CNZ
    ram[0x600] = @intFromEnum(Intel8080.opcodes.CNZ); // CNZ
    ram[0x601] = 0x9A; // Low byte of address
    ram[0x602] = 0xBC; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0xBC9A);
    try std.testing.expect(cpu.pop(ram[0..]) == 0x0603); // Stack pointer should be decremented
}

test "JMPs" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test unconditional jump
    ram[0] = @intFromEnum(Intel8080.opcodes.JMP); // JMP
    ram[1] = 0x12; // Low byte of address
    ram[2] = 0x34; // High byte of address
    cpu.pc = 0;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x3412);

    cpu.pc = 0; // Reset program counter for next test
    // Test conditional jumps
    cpu.flags.bits.zero = 1; // Set zero flag for JZ
    ram[0] = @intFromEnum(Intel8080.opcodes.JZ); // JZ
    ram[1] = 0x56; // Low byte of address
    ram[2] = 0x78; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x7856);

    cpu.pc = 0; // Reset program counter for next test
    cpu.flags.bits.zero = 0; // Reset zero flag for JNZ
    ram[0] = @intFromEnum(Intel8080.opcodes.JNZ); // JNZ
    ram[1] = 0x9A; // Low byte of address
    ram[2] = 0xBC; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0xBC9A);
}

test "POPs" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test POP B
    cpu.sp = 0x0FFE; // Set stack pointer to a known value
    ram[0x0FFF] = 0x12; // High byte of B
    ram[0x0FFE] = 0x34; // Low byte of B
    ram[0] = @intFromEnum(Intel8080.opcodes.POP_B); // POP B

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.b == 0x12);
    try std.testing.expect(cpu.c == 0x34);
    try std.testing.expect(cpu.sp == 0x1000);

    // Test POP D
    cpu.sp = 0x0FFE; // Reset stack pointer
    ram[0x0FFF] = 0x56; // High byte of D
    ram[0x0FFE] = 0x78; // Low byte of D
    ram[1] = @intFromEnum(Intel8080.opcodes.POP_D); // POP D

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.d == 0x56);
    try std.testing.expect(cpu.e == 0x78);
    try std.testing.expect(cpu.sp == 0x1000);

    // Test POP H
    cpu.sp = 0x0FFE; // Reset stack pointer
    ram[0x0FFF] = 0x9A; // High byte of H
    ram[0x0FFE] = 0xBC; // Low byte of H
    ram[2] = @intFromEnum(Intel8080.opcodes.POP_H); // POP H

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.h == 0x9A);
    try std.testing.expect(cpu.l == 0xBC);
    try std.testing.expect(cpu.sp == 0x1000);

    // Test POP PSW
    cpu.sp = 0x0FFE; // Reset stack pointer
    ram[0x0FFF] = 0xFF; // Flags byte (PSW)
    ram[0x0FFE] = 0xAB; // Accumulator
    ram[3] = @intFromEnum(Intel8080.opcodes.POP_PSW); // POP PSW
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0xFF);
    try std.testing.expect(cpu.flags.byte == 0xAB);
    try std.testing.expect(cpu.sp == 0x1000);
}

test "RETs" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0xFFFF]u8 = undefined;

    // Test RNZ
    cpu.flags.bits.zero = 0; // Set zero flag to 0
    ram[0] = @intFromEnum(Intel8080.opcodes.RNZ); // RNZ
    cpu.pc = 0;
    cpu.sp = 0x0FFE; // Set stack pointer to a known value
    cpu.push(0x1234, ram[0..]); // Push return address onto stack

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 0x1234); // Check if PC is set to return address

    // Test RZ
    cpu.flags.bits.zero = 0; // Set zero flag to 1
    ram[1] = @intFromEnum(Intel8080.opcodes.RZ); // RZ
    cpu.pc = 1;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly (no return)

    // Test RNC
    cpu.flags.bits.carry = 1; // Set carry flag to 1
    ram[2] = @intFromEnum(Intel8080.opcodes.RNC); // RNC
    cpu.pc = 2;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 3); // Check if PC is incremented correctly (no return)

    // Test RC
    cpu.flags.bits.carry = 0; // Set carry flag to 0
    ram[3] = @intFromEnum(Intel8080.opcodes.RC); // RC
    cpu.pc = 3;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 4); // Check if PC is incremented correctly (no return)

    // Test RPO
    cpu.flags.bits.parity = 1; // Set parity flag to 1
    ram[4] = @intFromEnum(Intel8080.opcodes.RPO); // RPO
    cpu.pc = 4;

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.pc == 5); // Check if PC is incremented correctly
}

test "ANA_XRA_ORA" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0x3C; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.ANA_B); // ANA B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x28); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.zero == 0); // Check if zero flag is correct
    try std.testing.expect(cpu.flags.bits.sign == 0); // Check if sign flag is correct
    try std.testing.expect(cpu.flags.bits.parity == 1); // Check if parity flag is correct
    try std.testing.expect(cpu.pc == 1); // Check if PC is incremented correctly

    // Test with XOR
    cpu.a = 0x3C; // Set accumulator to a known value
    ram[1] = @intFromEnum(Intel8080.opcodes.XRA_B); // XRA B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x14); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.zero == 0); // Check if zero flag is correct
    try std.testing.expect(cpu.flags.bits.sign == 0); // Check if sign flag is correct
    try std.testing.expect(cpu.flags.bits.parity == 1); // Check if parity flag is correct
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly

    // Test with ORA
    cpu.a = 0x3C; // Set accumulator to a known value
    ram[2] = @intFromEnum(Intel8080.opcodes.ORA_B); // ORA B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x3C); // Check if A is correct (no change)
    try std.testing.expect(cpu.flags.bits.zero == 0); // Check if zero flag is correct
    try std.testing.expect(cpu.flags.bits.sign == 0); // Check if sign flag is correct
    try std.testing.expect(cpu.flags.bits.parity == 1); // Check if parity
}

test "SUB" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0x3C; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.SUB_B); // SUB B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x14); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is correct
    try std.testing.expect(cpu.pc == 1); // Check if PC is incremented correctly

    // Test with borrow
    cpu.a = 0x14; // Set accumulator to a known value
    ram[1] = @intFromEnum(Intel8080.opcodes.SUB_B); // SUB B
    cpu.b = 0x20; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0xF4); // Check if A is correct (with underflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly

    // Test with Memory
    cpu.a = 0x3C; // Set accumulator to a known value
    ram[2] = @intFromEnum(Intel8080.opcodes.SUB_M); // SUB M
    cpu.l = 0x34; // Low byte of address
    cpu.h = 0x00; // High byte of address
    ram[0x0034] = 0x28; // Value to subtract
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x14); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag

    // Test with half-borrow
    cpu.a = 0x10; // Set accumulator to a known value
    ram[3] = @intFromEnum(Intel8080.opcodes.SUB_B); // SUB B
    cpu.b = 0x01; // Set register B to a known value
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x0F); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Check
}

test "SBC" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0x3C; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[0] = @intFromEnum(Intel8080.opcodes.SBB_B); // SBB B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x13); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is correct
    try std.testing.expect(cpu.pc == 1); // Check if PC is incremented correctly

    // Test with borrow resulting in underflow
    cpu.a = 0x14; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[1] = @intFromEnum(Intel8080.opcodes.SBB_B); // SBB B
    cpu.b = 0x20; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0xF3); // Check if A is correct (with underflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly
}

test "ADD" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0x14; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.ADD_B); // ADD B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x3C); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is correct
    try std.testing.expect(cpu.pc == 1); // Check if PC is incremented correctly

    // Test with carry
    cpu.a = 0xF0; // Set accumulator to a known value
    ram[1] = @intFromEnum(Intel8080.opcodes.ADD_B); // ADD B
    cpu.b = 0x20; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x10); // Check if A is correct (with overflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly`

    cpu.pc = 0;

    // Test with Memory
    cpu.a = 0x14; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.ADD_M); // ADD M
    cpu.l = 0x34; // Low byte of address
    cpu.h = 0x00; // High byte of address

    ram[0x0034] = 0x28; // Value to add

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x3C); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is correct

    // Test with Memory and Carry
    cpu.a = 0xF0; // Set accumulator to a known value
    ram[1] = @intFromEnum(Intel8080.opcodes.ADD_M); // ADD M
    cpu.l = 0x34; // Low byte of address
    cpu.h = 0x00; // High byte of address
    ram[0x0034] = 0x20; // Value to add

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x10); // Check if A is correct (with overflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set

    // Test half-carry flag
    cpu.a = 0x0F; // Set accumulator to a known value
    ram[2] = @intFromEnum(Intel8080.opcodes.ADD_B); // ADD B
    cpu.b = 0x01; // Set register B to a known value
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x10); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Check if half-carry flag is set

}

test "ADC" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0x14; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[0] = @intFromEnum(Intel8080.opcodes.ADC_B); // ADC B
    cpu.b = 0x28; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x3D); // Check if A is correct
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is correct
    try std.testing.expect(cpu.pc == 1); // Check if PC is incremented correctly

    // Test with carry resulting in overflow
    cpu.a = 0xF0; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[1] = @intFromEnum(Intel8080.opcodes.ADC_B); // ADC B
    cpu.b = 0x20; // Set register B to a known value

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x11); // Check if A is correct (with overflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set
    try std.testing.expect(cpu.pc == 2); // Check if PC is incremented correctly
}

test "LDA" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    ram[0] = @intFromEnum(Intel8080.opcodes.LDA); // LDA
    ram[1] = 0x56; // Low byte of address
    ram[2] = 0x00; // High byte of address
    ram[0x0056] = 0x42; // Value to load into A

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0x42); // Check if A is loaded correctly
    try std.testing.expect(cpu.pc == 3); // Check if PC is incremented correctly
}

test "STA" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0x1256]u8 = undefined;

    cpu.a = 0x42; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.STA); // STA
    ram[1] = 0x00; // Low byte of address
    ram[2] = 0x10; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x1000] == 0x42); // Check if A is stored correctly
    try std.testing.expect(cpu.pc == 3); // Check if PC is incremented correctly
}

test "CMA" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0b10101010; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.CMA); // CMA
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b01010101); // Check if A is complemented

    cpu.a = 0b11110000; // Set accumulator to another known value
    ram[1] = @intFromEnum(Intel8080.opcodes.CMA); // CMA
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b00001111); // Check if A is complemented
}

test "SHLD" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0x1350]u8 = undefined;

    cpu.h = 0x12; // Set H register
    cpu.l = 0x34; // Set L register
    ram[0] = @intFromEnum(Intel8080.opcodes.SHLD); // SHLD
    ram[1] = 0x50; // Low byte of address
    ram[2] = 0x12; // High byte of address

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x1250] == 0x34); // Check if L is stored
    try std.testing.expect(ram[0x1251] == 0x12); // Check if H is stored
    try std.testing.expect(cpu.pc == 3); // Check if PC is incremented correctly
}

test "LHLD" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0x1250]u8 = undefined;

    ram[0] = @intFromEnum(Intel8080.opcodes.LHLD); // LHLD
    ram[1] = 0x34; // Low byte of address
    ram[2] = 0x12; // High byte of address
    ram[0x1234] = 0x56; // Value to load into L
    ram[0x1235] = 0x78; // Value to load into H

    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.l == 0x56); // Check if L is loaded
    try std.testing.expect(cpu.h == 0x78); // Check if H is loaded
    try std.testing.expect(cpu.pc == 3); // Check if PC is incremented correctly
}

test "RRC" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0b10010110; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.RRC); // RRC
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b01001011); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is set correctly

    cpu.a = 0b01101001; // Set accumulator to another known value
    ram[1] = @intFromEnum(Intel8080.opcodes.RRC); // RRC
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b10110100); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set correctly
}

test "RAR" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0b10010110; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[0] = @intFromEnum(Intel8080.opcodes.RAR); // RAR
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b11001011); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is set correctly

    cpu.a = 0b01101001; // Set accumulator to another known value
    cpu.flags.bits.carry = 0; // Clear carry flag
    ram[1] = @intFromEnum(Intel8080.opcodes.RAR); // RAR
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b00110100); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set correctly
}

test "DCX" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.b = 0x12;
    cpu.c = 0x34; // BC = 0x1234
    ram[0] = @intFromEnum(Intel8080.opcodes.DCX_B); // DCX B
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.b == 0x12); // B should still be 0x12
    try std.testing.expect(cpu.c == 0x33); // C should now be 0x33

    cpu.h = 0x00;
    cpu.l = 0x00; // HL = 0x0000
    ram[1] = @intFromEnum(Intel8080.opcodes.DCX_H); // DCX H
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.h == 0xFF); // H should now be 0xFF (underflow)
    try std.testing.expect(cpu.l == 0xFF); // L should now be 0xFF (underflow)

    cpu.sp = 0x0001; // SP = 0x0001
    ram[2] = @intFromEnum(Intel8080.opcodes.DCX_SP); // DCX SP
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.sp == 0x0000); // SP should now be 0x0000

    ram[3] = @intFromEnum(Intel8080.opcodes.DCX_SP); // DCX SP again
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.sp == 0xFFFF); // SP should now be 0xFFFF (underflow)
}

test "RAL" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0b10010110; // Set accumulator to a known value
    cpu.flags.bits.carry = 1; // Set carry flag
    ram[0] = @intFromEnum(Intel8080.opcodes.RAL); // RAL
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b00101101); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set correctly

    cpu.a = 0b01101001; // Set accumulator to another known value
    cpu.flags.bits.carry = 0; // Clear carry flag
    ram[1] = @intFromEnum(Intel8080.opcodes.RAL); // RAL
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b11010010); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is set correctly
}

test "RLC" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.a = 0b10010110; // Set accumulator to a known value
    ram[0] = @intFromEnum(Intel8080.opcodes.RLC); // RLC
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b00101101); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 1); // Check if carry flag is set correctly

    cpu.a = 0b01101001; // Set accumulator to another known value
    ram[1] = @intFromEnum(Intel8080.opcodes.RLC); // RLC
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.a == 0b11010010); // Check if A is rotated correctly
    try std.testing.expect(cpu.flags.bits.carry == 0); // Check if carry flag is set correctly
}

test "DAD" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [0x2000]u8 = undefined;

    cpu.h = 0x12;
    cpu.l = 0x34; // HL = 0x1234
    cpu.b = 0x00;
    cpu.c = 0x10; // BC = 0x0010
    ram[0] = @intFromEnum(Intel8080.opcodes.DAD_B); // DAD B
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.h == 0x12); // H should still be 0x12
    try std.testing.expect(cpu.l == 0x44); // L should now be 0x44 (0x34 + 0x10)
    try std.testing.expect(cpu.flags.bits.carry == 0); // No carry should be set

    cpu.h = 0xFF;
    cpu.l = 0xFF; // HL = 0xFFFF
    cpu.d = 0x00;
    cpu.e = 0x01; // DE = 0x0001
    ram[1] = @intFromEnum(Intel8080.opcodes.DAD_D); // DAD D
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.h == 0x00); // H should now be 0x00 (overflow)
    try std.testing.expect(cpu.l == 0x00); // L should now be 0x00 (overflow)
    try std.testing.expect(cpu.flags.bits.carry == 1); // Carry should be set
}

test "MVI" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    ram[0] = @intFromEnum(Intel8080.opcodes.MVI_B); // MVI B, 0x56
    ram[1] = 0x56;
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.b == 0x56); // B should now be 0x56

    ram[2] = @intFromEnum(Intel8080.opcodes.MVI_C); // MVI C, 0x34
    ram[3] = 0x34;
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.c == 0x34); // C should now be 0x34

    cpu.h = 0x00;
    cpu.l = 0x34;
    ram[4] = @intFromEnum(Intel8080.opcodes.MVI_M); // MVI M, 0x78
    ram[5] = 0x78;
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0034] == 0x78); // Memory at address HL should now be 0x78
}

test "INR" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.b = 0xFF; // Set register B to 0xFF
    ram[0] = @intFromEnum(Intel8080.opcodes.INR_B); // INR B
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.b == 0x00); // B should now be 0x00
    try std.testing.expect(cpu.flags.bits.zero == 1); // Zero flag should be set
    try std.testing.expect(cpu.flags.bits.sign == 0); // Sign flag should be clear
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Auxiliary carry should be set

    cpu.c = 0x7F; // Set register C to 0x7F
    ram[1] = @intFromEnum(Intel8080.opcodes.INR_C); // INR C
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.c == 0x80); // C should now be 0x80
    try std.testing.expect(cpu.flags.bits.zero == 0); // Zero flag should be clear
    try std.testing.expect(cpu.flags.bits.sign == 1); // Sign flag should be set
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Auxiliary carry should be set

    cpu.h = 0x00;
    cpu.l = 0x34;
    ram[2] = @intFromEnum(Intel8080.opcodes.INR_M); // INR M
    ram[0x0034] = 0xFF; // Set memory at address HL to 0xFF
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0034] == 0x00); // Memory at address HL should now be 0x00
    try std.testing.expect(cpu.flags.bits.zero == 1); // Zero flag should be set
    try std.testing.expect(cpu.flags.bits.sign == 0); // Sign flag should be clear
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Auxiliary carry should be set
}

test "DCR" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.d = 0x00; // Set register D to 0x00
    ram[0] = @intFromEnum(Intel8080.opcodes.DCR_D); // DCR D
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.d == 0xFF); // D should now be 0xFF
    try std.testing.expect(cpu.flags.bits.zero == 0); // Zero flag should be clear
    try std.testing.expect(cpu.flags.bits.sign == 1); // Sign flag should be set
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Auxiliary carry should be set

    cpu.e = 0x01; // Set register E to 0x01
    ram[1] = @intFromEnum(Intel8080.opcodes.DCR_E); // DCR E
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.e == 0x00); // E should now be 0x00
    try std.testing.expect(cpu.flags.bits.zero == 1); // Zero flag should be set
    try std.testing.expect(cpu.flags.bits.sign == 0); // Sign flag should be clear
    try std.testing.expect(cpu.flags.bits.aux_carry == 0); // Auxiliary carry should not be set

    cpu.h = 0x00;
    cpu.l = 0x34;
    ram[2] = @intFromEnum(Intel8080.opcodes.DCR_M); // DCR M
    ram[0x0034] = 0x01; // Set memory at address HL to 0x01
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0034] == 0x00); // Memory at address HL should now be 0x00
    try std.testing.expect(cpu.flags.bits.zero == 1); // Zero flag should be set
    try std.testing.expect(cpu.flags.bits.sign == 0); // Sign flag should be clear
    try std.testing.expect(cpu.flags.bits.aux_carry == 0); // Auxiliary carry should not be set

    cpu.c = 0x10; // Set register E to 0x01
    ram[3] = @intFromEnum(Intel8080.opcodes.DCR_C); // DCR E
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.c == 0x0F); // E should now be 0x00
    try std.testing.expect(cpu.flags.bits.zero == 0); // Zero flag should be set
    try std.testing.expect(cpu.flags.bits.sign == 0); // Sign flag should be clear
    try std.testing.expect(cpu.flags.bits.aux_carry == 1); // Auxiliary carry should be set

}

test "STAX" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.set_register_pair(.B, 0x0034); // Set register pair BC
    cpu.a = 0x56; // Set accumulator A

    ram[0] = @intFromEnum(Intel8080.opcodes.STAX_B); // STAX B
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0034] == 0x56); // Memory at address BC should be 0x56

    cpu.set_register_pair(.D, 0x0078); // Set register pair DE
    cpu.a = 0x9A; // Set accumulator A

    ram[1] = @intFromEnum(Intel8080.opcodes.STAX_D); // STAX D
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0078] == 0x9A); // Memory at address DE should be 0x9A
}

test "INX" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.set_register_pair(.B, 0x0034); // Set register pair BC
    ram[0] = @intFromEnum(Intel8080.opcodes.INX_B); // INX B
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.get_register_pair(.B) == 0x0035); // BC should now be 0x0035

    cpu.set_register_pair(.D, 0x00FF); // Set register pair DE
    ram[1] = @intFromEnum(Intel8080.opcodes.INX_D); // INX D
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.get_register_pair(.D) == 0x0100); // DE should now be 0x0100

    cpu.set_register_pair(.H, 0xFFFF); // Set register pair HL
    ram[2] = @intFromEnum(Intel8080.opcodes.INX_H); // INX H
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.get_register_pair(.H) == 0x0000); // HL should now be 0x0000 (overflow)

    cpu.sp = 0x00FF; // Set stack pointer
    ram[3] = @intFromEnum(Intel8080.opcodes.INX_SP); // INX SP
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.sp == 0x0100); // SP should now be 0x0100
}

test "MOV" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    // MOV B, C: C -> B
    cpu.set_register(0, 0x12); // b
    cpu.set_register(1, 0x34); // c
    cpu.print_registers();
    ram[0] = @intFromEnum(Intel8080.opcodes.MOV_BC); // MOV b, c
    cpu.execute_instruction(ram[0..], true);
    cpu.print_registers();
    try std.testing.expect(cpu.get_register(0) == 0x34); // B should now be 0x34

    // MOV C, D
    cpu.d = 0x56;
    ram[1] = @intFromEnum(Intel8080.opcodes.MOV_CD); // MOV c, d
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(cpu.c == 0x56); // C should now be 0x56

    // MOV M, A (using H and L)
    cpu.h = 0x00;
    cpu.l = 0x34;
    cpu.a = 0x78;
    ram[2] = @intFromEnum(Intel8080.opcodes.MOV_MA); // MOV M, A
    cpu.execute_instruction(ram[0..], true);
    try std.testing.expect(ram[0x0034] == 0x78); // Memory at address HL should be 0x78
}

test "NOP" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;
    ram[0] = 0x0; // Set NOP opcode at address 0

    // Execute the instruction at address 0
    cpu.execute_instruction(ram[0..], true);
    // Check if the opcode at address 0 is NOP
    try std.testing.expect(cpu.pc == 0x01);
}

test "carry calculation" {
    // regular carry
    const carry = Intel8080.calc_carry(8, 0xFA, 0x09, 0);
    try std.testing.expect(carry == 1);

    const carry1 = Intel8080.calc_carry(8, 0xAA, 0x09, 0);
    try std.testing.expect(carry1 == 0);

    // half carry
    const hcarry = Intel8080.calc_carry(4, 0x3F, 0x01, 0);
    try std.testing.expect(hcarry == 1);

    const hcarry1 = Intel8080.calc_carry(4, 0x73, 0x07, 1);
    try std.testing.expect(hcarry1 == 0);
}

test "stack functions" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    var ram: [256]u8 = undefined;

    cpu.sp = 0x20; // Set stack pointer

    cpu.push(Intel8080.get_word_from_bytes(0x12, 0x34), ram[0..]);
    try std.testing.expect(ram[0x20 - 1] == 0x12);
    try std.testing.expect(ram[0x20 - 2] == 0x34);

    cpu.push(Intel8080.get_word_from_bytes(0x56, 0x78), ram[0..]);
    try std.testing.expect(ram[0x20 - 3] == 0x56);
    try std.testing.expect(ram[0x20 - 4] == 0x78);

    _ = cpu.pop(ram[0..]);
    const popped_word = cpu.pop(ram[0..]);
    try std.testing.expect(popped_word == 0x1234);
}

test "register operations" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    cpu.set_register(0, 0x12); // Set register B
    cpu.set_register(1, 0x34); // Set register C

    try std.testing.expect(cpu.get_register(0) == 0x12);
    try std.testing.expect(cpu.get_register(1) == 0x34);
    try std.testing.expect(cpu.b == 0x12); // Register D should be 0
    try std.testing.expect(cpu.c == 0x34); // Register D should be 0

    cpu.set_register_pair(.B, 0x1324); // Set register pair BC
    try std.testing.expect(cpu.get_register_pair(.B) == 0x1324);

    cpu.set_register_pair(.D, 0xccdd);
    try std.testing.expect(cpu.d == 0xcc);
    try std.testing.expect(cpu.e == 0xdd);
}

test "register pair operations" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    cpu.set_register_pair(.B, 0x1234); // Set register pair BC
    try std.testing.expect(cpu.get_register_pair(.B) == 0x1234);
    try std.testing.expect(cpu.get_register(0) == 0x12); // B
    try std.testing.expect(cpu.get_register(1) == 0x34); // C

    cpu.set_register_pair(.D, 0x5678); // Set register pair DE
    try std.testing.expect(cpu.get_register_pair(.D) == 0x5678);

    cpu.set_register_pair(.H, 0x9ABC); // Set register pair HL
    try std.testing.expect(cpu.get_register_pair(.H) == 0x9ABC);

    cpu.set_register_pair(.PSW, 0xDEF0); // Set register pair AF
    try std.testing.expect(cpu.get_register_pair(.PSW) == 0xDEF0);

    cpu.b = 0x01;
    cpu.c = 0x02;
    try std.testing.expect(cpu.get_register_pair(.B) == 0x0102); // B
}

test "flag operations" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    cpu.flags.byte = 0x02; // Clear all flags
    try std.testing.expect(cpu.flags.bits.nu1 == 1);

    cpu.flags.bits.carry = 1; // Set carry flag
    try std.testing.expect(cpu.flags.byte == 0x03); // Carry flag should be set

    cpu.flags.bits.zero = 1; // Set zero flag
    try std.testing.expect(cpu.flags.byte == 67); // Zero flag should be set

}

test "set ZSP" {
    var in: [256]u8 = undefined;
    var out: [256]u8 = undefined;
    var cpu = Intel8080{ .ports_in = in[0..], .ports_out = out[0..] };

    cpu.set_zsp(0x00); // Set zero flag
    try std.testing.expect(cpu.flags.bits.zero == 1);
    try std.testing.expect(cpu.flags.bits.sign == 0);
    try std.testing.expect(cpu.flags.bits.parity == 1); // Even parity

    cpu.set_zsp(0xFF); // Set sign flag
    try std.testing.expect(cpu.flags.bits.zero == 0);
    try std.testing.expect(cpu.flags.bits.sign == 1);
    try std.testing.expect(cpu.flags.bits.parity == 1); // Odd parity

    cpu.set_zsp(0x55); // Set parity flag
    try std.testing.expect(cpu.flags.bits.zero == 0);
    try std.testing.expect(cpu.flags.bits.sign == 0);
    try std.testing.expect(cpu.flags.bits.parity == 1); // Even parity
}

test "get word from bytes" {
    const word = Intel8080.get_word_from_bytes(0x12, 0x34);
    try std.testing.expect(word == 0x1234);
}
