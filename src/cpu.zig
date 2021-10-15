const std = @import("std");
const expectEqual = @import("std").testing.expectEqual;
const stdout = std.io.getStdOut().writer();

pub const Cpu = struct {
    reg: packed union {
        _16: Register16,
        _8: Register8,
        flags: Flags,
    },
    interruptEnabled: bool = false,

    pub fn print(self: *Cpu) !void {
        try stdout.print("AF 0x{x:0>4}\n", .{self.reg._16.AF});
        try stdout.print("BC 0x{x:0>4}\n", .{self.reg._16.BC});
        try stdout.print("DE 0x{x:0>4}\n", .{self.reg._16.DE});
        try stdout.print("HL 0x{x:0>4}\n", .{self.reg._16.HL});
        try stdout.print("PC 0x{x:0>4}\n", .{self.reg._16.PC});
        try stdout.print("SP 0x{x:0>4}\n", .{self.reg._16.SP});
    }
};

pub const Register16 = packed struct {
    AF: u16,
    BC: u16,
    DE: u16,
    HL: u16,
    SP: u16,
    PC: u16,
};

pub const Register8 = packed struct {
    F: u8,
    A: u8,

    C: u8,
    B: u8,

    E: u8,
    D: u8,

    L: u8,
    H: u8,
};

// S - Sign Flag
// Z - Zero Flag
// 0 - Not used, always zero
// A - also called AC, Auxiliary Carry Flag
// 0 - Not used, always zero
// P - Parity Flag
// 1 - Not used, always one
// C - Carry Flag

// 7-0 bit:  SZ0A0P1C
pub const Flags = packed struct {
    CY: bool,
    _padOne: u1 = 1,
    P: bool,
    _padZeroAgain: u1 = 0,
    A: bool,
    _padZero: u1 = 0,
    Z: bool,
    S: bool,
};

test "arrangement" {
    const A = packed struct {
        CY: bool,
        B: bool,
    };

    const Reg = packed union {
        F: u2,
        a: A,
    };

    var reg: Reg = undefined;

    reg.F = 0b01;
    try expectEqual(true, reg.a.CY);
    try expectEqual(false, reg.a.B);
    try expectEqual(@as(u2,1), reg.F);
}

test "register flags" {
    var cpu: Cpu = undefined;

    // 7-0 bit:  SZ0A0P1C

    //SZ0A0P1C
    //00000010
    cpu.reg._8.F = 0b0000_0010;
    try expectEqual(false, cpu.reg.flags.CY);
    try expectEqual(false, cpu.reg.flags.P);
    try expectEqual(false, cpu.reg.flags.A);
    try expectEqual(false, cpu.reg.flags.Z);
    try expectEqual(false, cpu.reg.flags.S);

    //SZ0A0P1C
    //01010011
    cpu.reg._8.F = 0b0101_0011;
    try expectEqual(true, cpu.reg.flags.CY);
    try expectEqual(false, cpu.reg.flags.P);
    try expectEqual(true, cpu.reg.flags.A);
    try expectEqual(true, cpu.reg.flags.Z);
    try expectEqual(false, cpu.reg.flags.S);
}

test "register arrangement" {
    var cpu: Cpu = undefined;

    cpu.reg._8.A = 0x12;
    cpu.reg._8.F = 0x34;
    try expectEqual(@as(u16, 0x1234), cpu.reg._16.AF);

    cpu.reg._8.B = 0x23;
    cpu.reg._8.C = 0x34;
    try expectEqual(@as(u16, 0x2334), cpu.reg._16.BC);

    cpu.reg._8.D = 0x58;
    cpu.reg._8.E = 0x76;
    try expectEqual(@as(u16, 0x5876), cpu.reg._16.DE);

    cpu.reg._8.H = 0xAF;
    cpu.reg._8.L = 0xCD;
    try expectEqual(@as(u16, 0xAFCD), cpu.reg._16.HL);
}