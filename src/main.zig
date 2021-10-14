const std = @import("std");
const time = @import("std").time;
const expectEqual = @import("std").testing.expectEqual;
const stdout = std.io.getStdOut().writer();
const cp = @import("cpu.zig");
const sdl = @cImport({
    @cInclude("SDL2/SDL.h");
});

var cpu: cp.Cpu = undefined;

var memory: []u8 = undefined;

var screen_buf: [224 * 256]u32 = undefined;

pub fn main() !void {
    const file = try std.fs.cwd().openFile("invaders", .{});
    defer file.close();

    const memorySize = 0x4000;

    const allocator = std.heap.page_allocator;
    memory = try allocator.alloc(u8, memorySize);
    defer allocator.free(memory);

    const n = try file.readAll(memory);

    //************************************************
    //************* INIT SDL *************************
    //************************************************
    if (sdl.SDL_Init(sdl.SDL_INIT_VIDEO | sdl.SDL_INIT_JOYSTICK) != 0) {
        @panic("Unable to init sdl");
    }
    defer sdl.SDL_Quit();

    const window = sdl.SDL_CreateWindow("Space Invaders", sdl.SDL_WINDOWPOS_CENTERED, sdl.SDL_WINDOWPOS_CENTERED, @intCast(c_int, 224), @intCast(c_int, 256), sdl.SDL_WINDOW_OPENGL) orelse {
        @panic("Unable to create window");
    };
    defer sdl.SDL_DestroyWindow(window);

    const renderer = sdl.SDL_CreateRenderer(window, -1, 0) orelse {
        @panic("Unable to create renderer");
    };
    defer sdl.SDL_DestroyRenderer(renderer);

    const screentex = sdl.SDL_CreateTexture(renderer, sdl.SDL_PIXELFORMAT_ARGB8888, sdl.SDL_TEXTUREACCESS_STREAMING, @intCast(c_int, 224), @intCast(c_int, 256));

    var dest_rect = sdl.SDL_Rect{ .x = 0, .y = 0, .w = 224, .h = 256 };
    //************************************************
    //************* END INIT SDL *********************
    //************************************************

    // var pc: u16 = 0;
    // const file_size = try file.getEndPos();
    // while (pc < file_size) {
    //     pc += try disassemble8080(file_memory, pc);
    // }

    var lastInterrupt: i64 = 0;
    var sixtyHz: i64 = 16; // 60 Hz = 1 interrupt per 16.66667 ms

    var count: u32 = 0;
    while (true) {
        try emulate8080Op(count);
        count += 1;

        var currentTime = time.milliTimestamp();
        if (cpu.interruptEnabled and (currentTime - lastInterrupt) > sixtyHz) { // enough time elapsed
            generateInterrupt(2); //interrupt 2
            lastInterrupt = currentTime;

            spaceInvaders_vblank(screentex);
            _ = sdl.SDL_RenderClear(renderer);
            const flip = sdl.SDL_FLIP_HORIZONTAL | sdl.SDL_FLIP_VERTICAL;
            _ = sdl.SDL_RenderCopy(renderer, screentex, 0, 0);
            sdl.SDL_RenderPresent(renderer);
        }
    }
}

pub fn spaceInvaders_vblank(screentex: ?*sdl.struct_SDL_Texture) void {
    var vram_base: u16 = 0x2400;
    var i: usize = 0;
    var pitch: usize = 256;

    while (vram_base < 0x4000) {
        var b = memory[vram_base];

        setPixel(b >> 0, position(i));
        i += 1;
        setPixel(b >> 1, position(i));
        i += 1;
        setPixel(b >> 2, position(i));
        i += 1;
        setPixel(b >> 3, position(i));
        i += 1;
        setPixel(b >> 4, position(i));
        i += 1;
        setPixel(b >> 5, position(i));
        i += 1;
        setPixel(b >> 6, position(i));
        i += 1;
        setPixel(b >> 7, position(i));
        i += 1;

        vram_base += 1;
    }

    var pixelsPtr = @ptrCast(*c_void, &screen_buf[0]);
    _ = sdl.SDL_UpdateTexture(screentex, 0, pixelsPtr, @intCast(c_int, 224 * 4));
}

pub fn position(i: usize) usize {
    var x: usize = i / 256;
    var y: usize = 255 - (i % 256);
    return 224 * y + x;
}

pub fn setPixel(byte: u8, position2: usize) void {
    if (byte & 1 == 1) {
        screen_buf[position2] = 0xFFFFFFFF;
    } else {
        screen_buf[position2] = 0xFF000000;
    }
}

pub fn generateInterrupt(number: u8) void {
    //perform "PUSH PC"
    memory[cpu.reg._16.SP - 1] = @truncate(u8, cpu.reg._16.PC >> 8);
    memory[cpu.reg._16.SP - 2] = @truncate(u8, cpu.reg._16.PC);
    cpu.reg._16.SP = cpu.reg._16.SP - 2;

    //Push(state, (state->pc & 0xFF00) >> 8, (state->pc & 0xff));

    //Set the PC to the low memory vector.
    //This is identical to an "RST interrupt_num" instruction.
    cpu.reg._16.PC = 8 * number;
}

pub fn emulate8080Op(count: u32) !void {
    const opCode = fetch8();
    // try stdout.print("{x:0>4} {x}", .{ cpu.reg._16.PC - 1, opCode });
    // try stdout.print("\trun={d}\n", .{count});

    switch (opCode) {
        0x00 => { //NOP
        },
        0x01 => { //LXI   BC,word
            cpu.reg._16.BC = fetch16();
        },
        0x03 => { //INX B     BC <- BC+1
            cpu.reg._16.BC += 1;
        },
        0x05 => { //DCR B   B <- B-1
            cpu.reg._8.B -%= 1;
            updateFlags(cpu.reg._8.B);
        },
        0x06 => { //MVI B, D8
            cpu.reg._8.B = fetch8();
        },
        0x0a => { // LDAX B    A <- (BC)
            cpu.reg._8.A = memory[cpu.reg._16.BC];
        },
        0x0d => { // DCR C  C <-C-1
            cpu.reg._8.C -%= 1;
            updateFlags(cpu.reg._8.C);
        },
        0x09 => { //DAD B   HL = HL + BC
            var result = @as(u32, cpu.reg._16.HL) + @as(u32, cpu.reg._16.BC);
            cpu.reg._16.HL = @truncate(u16, result);
            cpu.reg.flags.CY = result > 0xffff;
        },
        0x0e => { //MVI C,D8
            cpu.reg._8.C = fetch8();
        },
        0x0f => { //RRC 	A = A >> 1; bit 7 = prev bit 0; CY = prev bit 0
            var prevBit = cpu.reg._8.A & 0x01;
            cpu.reg._8.A = (cpu.reg._8.A >> 1) | (prevBit << 7);
            cpu.reg.flags.CY = 1 == prevBit;
        },
        0x11 => { //LXI   DE,word
            cpu.reg._16.DE = fetch16();
        },
        0x13 => { //INX D   DE <- DE + 1
            cpu.reg._16.DE += 1;
        },
        0x15 => { //DCR D   D <- D-1
            cpu.reg._8.D -%= 1;
            updateFlags(cpu.reg._8.D);
        },
        0x16 => { //MVI D, D8
            cpu.reg._8.D = fetch8();
        },
        0x19 => { //DAD D   HL = HL + DE
            var result = @as(u32, cpu.reg._16.HL) + @as(u32, cpu.reg._16.DE);
            cpu.reg._16.HL = @truncate(u16, result);
            cpu.reg.flags.CY = result > 0xffff;
        },
        0x1a => { //LDAX D 	A <- (DE)
            cpu.reg._8.A = memory[cpu.reg._16.DE];
        },
        0x1d => { // DCR E      E <- E-1
            cpu.reg._8.E -%= 1;
            updateFlags(cpu.reg._8.E);
        },
        0x1e => { //MVI E,D8
            cpu.reg._8.E = fetch8();
        },
        0x21 => { //LXI   HL,word
            cpu.reg._16.HL = fetch16();
        },
        0x23 => { //INX H   HL <- HL + 1
            cpu.reg._16.HL += 1;
        },
        0x2e => { //MVI L, D8
            cpu.reg._8.L = fetch8();
        },
        0x25 => { // DCR H  H <- H-1
            cpu.reg._8.H -%= 1;
            updateFlags(cpu.reg._8.H);
        },
        0x26 => { //MVI H,D8
            cpu.reg._8.H = fetch8();
        },
        0x29 => { //DAD H   	HL = HL + HI
            var result = @as(u32, cpu.reg._16.HL) << 1;
            cpu.reg._16.HL = @truncate(u16, result);
            cpu.reg.flags.CY = result > 0xffff;
        },
        0x2d => { // DCR L  L <- L-1
            cpu.reg._8.L -%= 1;
            updateFlags(cpu.reg._8.L);
        },
        0x2f => { //CMA (not)   A <- !A
            cpu.reg._8.A = ~cpu.reg._8.A;
        },
        0x31 => { //LXI   SP,word
            cpu.reg._16.SP = fetch16();
        },
        0x32 => { //STA adr     (adr) <- A
            memory[fetch16()] = cpu.reg._8.A;
        },
        0x35 => { //DCR M   (HL) <- (HL)-1
            memory[cpu.reg._16.HL] = memory[cpu.reg._16.HL] -% 1;
            updateFlags(memory[cpu.reg._16.HL]);
        },
        0x36 => { //MVI M,D8    	(HL) <- byte 2
            memory[cpu.reg._16.HL] = fetch8();
        },
        0x37 => { // STC    CY = 1
            cpu.reg.flags.CY = true;
        },
        0x3a => { //LDA adr     A <- (adr)
            cpu.reg._8.A = memory[fetch16()];
        },
        0x3d => { //DCR A   	A <- A-1
            cpu.reg._8.A -%= 1;
            updateFlags(cpu.reg._8.A);
        },
        0x3e => { //MVI A,D8
            cpu.reg._8.A = fetch8();
        },
        0x4f => { //MOV C,A     C <- A
            cpu.reg._8.C = cpu.reg._8.A;
        },
        0x56 => { // MOV D,M    D <- (HL)
            cpu.reg._8.D = memory[cpu.reg._16.HL];
        },
        0x57 => { // MOV D,A    D <- A
            cpu.reg._8.D = cpu.reg._8.A;
        },
        0x5c => { // MOV E,H    	E <- H
            cpu.reg._8.E = cpu.reg._8.H;
        },
        0x5e => { // MOV E,M    	E <- (HL)
            cpu.reg._8.E = memory[cpu.reg._16.HL];
        },
        0x5f => { //MOV E,A     E <- A
            cpu.reg._8.E = cpu.reg._8.A;
        },
        0x66 => { // MOV H,M    H <- (HL)
            cpu.reg._8.H = memory[cpu.reg._16.HL];
        },
        0x67 => { //MOV H,A     H <- A
            cpu.reg._8.H = cpu.reg._8.A;
        },
        0x6f => { // MOV L,A    L <- A
            cpu.reg._8.L = cpu.reg._8.A;
        },
        0x77 => { // MOV M,A    (HL) <- A
            memory[cpu.reg._16.HL] = cpu.reg._8.A;
        },
        0x7a => { // MOV A,D    A <- D
            cpu.reg._8.A = cpu.reg._8.D;
        },
        0x7b => { // MOV A,E    A <- E
            cpu.reg._8.A = cpu.reg._8.E;
        },
        0x7c => { // MOV A,H    A <- H
            cpu.reg._8.A = cpu.reg._8.H;
        },
        0x7d => { // MOV A,L    A <- L
            cpu.reg._8.A = cpu.reg._8.L;
        },
        0x7e => { // MOV A,M    A <- (HL)
            cpu.reg._8.A = memory[cpu.reg._16.HL];
        },
        0x80 => { //ADD B   A <- A + B
            var result: u16 = @as(u16, cpu.reg._8.A) +% @as(u16, cpu.reg._8.B);
            updateFlags(result);
            cpu.reg._8.A = @truncate(u8, result);
        },
        0x86 => { //ADD M    A <- A + (HL)
            var result: u16 = @as(u16, cpu.reg._8.A) +% memory[cpu.reg._16.HL];
            updateFlags(result);
            cpu.reg._8.A = @truncate(u8, result);
        },
        0xa7 => { //ANA A   A <- A & A
            // try cpu.print();
            cpu.reg._8.A &= cpu.reg._8.A;
            updateFlagsResetCarry(cpu.reg._8.A);

            // try exit(opCode);
        },
        0xaf => { //XRA A   	A <- A ^ A
            cpu.reg._8.A ^= cpu.reg._8.A;
            updateFlagsResetCarry(cpu.reg._8.A);
        },
        0xb6 => { // ORA M      A <- A | (HL)
            var result = @as(u16, cpu.reg._8.A) | cpu.reg._16.HL;
            // FIXME: AC
            updateFlags(result);
            cpu.reg._8.A = @truncate(u8, result);
        },
        0xc1 => { //POP B   C <- (sp); B <- (sp+1); sp <- sp+2
            cpu.reg._8.B = memory[cpu.reg._16.SP + 1];
            cpu.reg._8.C = memory[cpu.reg._16.SP];
            cpu.reg._16.SP = cpu.reg._16.SP + 2;
        },
        0xc2 => { //JNZ address      if NZ, PC <- adr
            var address = fetch16();
            if (!cpu.reg.flags.Z) {
                cpu.reg._16.PC = address;
            }
        },
        0xc3 => { //JMP    ${x}{x}
            cpu.reg._16.PC = fetch16();
        },
        0xc5 => { //PUSH B    	(sp-2)<-C; (sp-1)<-B; sp <- sp - 2
            memory[cpu.reg._16.SP - 1] = cpu.reg._8.B;
            memory[cpu.reg._16.SP - 2] = cpu.reg._8.C;
            cpu.reg._16.SP = cpu.reg._16.SP - 2;
        },
        0xc6 => { //ADI D8  A <- A + byte
            var result: u16 = @as(u16, cpu.reg._8.A) +% @as(u16, fetch8());
            updateFlags(result);
            cpu.reg._8.A = @truncate(u8, result);
        },
        0xc8 => { //RZ     if Z, RET
            if (cpu.reg.flags.Z) {
                ret();
            }
        },
        0xca => { // JZ adr     if Z, PC <- adr
            var address = fetch16();
            if (cpu.reg.flags.Z) {
                cpu.reg._16.PC = address;
            }
        },
        0xcd => { // CALL adr     (SP-1)<-PC.hi;(SP-2)<-PC.lo;SP<-SP-2;PC=adr
            var nextPC = cpu.reg._16.PC + 2;
            memory[cpu.reg._16.SP - 1] = @truncate(u8, nextPC >> 8);
            memory[cpu.reg._16.SP - 2] = @truncate(u8, nextPC);
            cpu.reg._16.SP = cpu.reg._16.SP - 2;
            cpu.reg._16.PC = fetch16();
            try stdout.print("CALL nextPC={x}, newAddress={x}\n", .{ nextPC, cpu.reg._16.PC });
        },
        0xc9 => { //RET
            ret();
        },
        0xd8 => { //RC      if CY, RET
            if (cpu.reg.flags.CY) {
                ret();
            }
        },
        0xd1 => { //POP D   	E <- (sp); D <- (sp+1); sp <- sp+2
            cpu.reg._8.D = memory[cpu.reg._16.SP + 1];
            cpu.reg._8.E = memory[cpu.reg._16.SP];
            cpu.reg._16.SP = cpu.reg._16.SP + 2;
        },
        0xd3 => { //OUT D8
            portOut(fetch8(), cpu.reg._8.A);
        },
        0xd5 => { //PUSH D  	(sp-2)<-E; (sp-1)<-D; sp <- sp - 2
            memory[cpu.reg._16.SP - 1] = cpu.reg._8.D;
            memory[cpu.reg._16.SP - 2] = cpu.reg._8.E;
            cpu.reg._16.SP = cpu.reg._16.SP - 2;
        },
        0xda => { //JC adr      if CY, PC<-adr
            var adr = fetch16();
            if (cpu.reg.flags.CY) {
                cpu.reg._16.PC = adr;
            }
        },
        0xdb => { //IN D8
            cpu.reg._8.A = portIn(fetch8());
        },
        0xe1 => { //POP H   L <- (sp); H <- (sp+1); sp <- sp+2
            cpu.reg._8.H = memory[cpu.reg._16.SP + 1];
            cpu.reg._8.L = memory[cpu.reg._16.SP];
            cpu.reg._16.SP = cpu.reg._16.SP + 2;
        },
        0xe5 => { //PUSH H     	(sp-2)<-L; (sp-1)<-H; sp <- sp - 2
            memory[cpu.reg._16.SP - 1] = cpu.reg._8.H;
            memory[cpu.reg._16.SP - 2] = cpu.reg._8.L;
            cpu.reg._16.SP = cpu.reg._16.SP - 2;
        },
        0xe6 => { //ANI D8 	A <- A & data
            cpu.reg._8.A = cpu.reg._8.A & fetch8();
            updateFlagsResetCarry(cpu.reg._8.A);
        },
        0xeb => { //XCHG    H <-> D; L <-> E
            var tmp = cpu.reg._8.D;
            cpu.reg._8.D = cpu.reg._8.H;
            cpu.reg._8.H = tmp;

            var tmpE = cpu.reg._8.E;
            cpu.reg._8.E = cpu.reg._8.L;
            cpu.reg._8.L = tmpE;
        },
        0xf1 => { //POP PSW 	flags <- (sp); A <- (sp+1); sp <- sp+2
            cpu.reg._8.A = memory[cpu.reg._16.SP + 1];
            cpu.reg._8.F = memory[cpu.reg._16.SP];
            cpu.reg._16.SP = cpu.reg._16.SP + 2;
        },
        0xf5 => { //PUSH PSW    (sp-2)<-flags; (sp-1)<-A; sp <- sp - 2
            memory[cpu.reg._16.SP - 1] = cpu.reg._8.A;
            memory[cpu.reg._16.SP - 2] = cpu.reg._8.F;
            cpu.reg._16.SP = cpu.reg._16.SP - 2;
        },
        0xfb => { //EI  enable interrupts
            cpu.interruptEnabled = true;
        },
        0xfe => { //CPI D8  A - data
            var data = fetch8();
            var result = cpu.reg._8.A -% data;
            cpu.reg.flags.Z = result == 0; // zero flag: if the result is zero (equal)
            cpu.reg.flags.CY = cpu.reg._8.A < data;
            cpu.reg.flags.S = ((result & 0x80) != 0); // // Sign flag: if bit 7 is set
            cpu.reg.flags.P = parity(result); // Parity
        },

        else => {
            try exit(opCode);
        },
    }

    // if(count == 9999999) {
    //     try exit(opCode);
    // }
}

pub fn ret() void {
    cpu.reg._16.PC = @as(u16, memory[cpu.reg._16.SP]) | (@as(u16, memory[cpu.reg._16.SP + 1]) << 8);
    std.debug.print("RET adr={x}\n", .{cpu.reg._16.PC});
    cpu.reg._16.SP += 2;
}

pub fn exit(opCode: u8) !void {
    try stdout.print("{x:0>4} unknown Opcode {x}\n", .{ cpu.reg._16.PC - 1, opCode });
    try cpu.print();
    @panic("ouch");
}

var shift_offset: u4 = 0;
var shift0: u8 = 0;
var shift1: u8 = 0;

pub fn portIn(port: u8) u8 {
    var result: u8 = 0;
    switch (port) {
        3 => {
            var value: u16 = toU16(shift1, shift0);
            var shifted = value >> (8 - shift_offset);
            result = @truncate(u8, shifted);
        },
        else => {},
    }
    return result;
}

pub fn portOut(port: u8, value: u8) void {
    switch (port) {
        2 => shift_offset = @truncate(u4, value) & 0x7,
        4 => {
            shift0 = shift1;
            shift1 = value;
        },
        else => {},
    }
}

pub fn updateFlagsResetCarry(result: u8) void {
    cpu.reg.flags.Z = result == 0; // zero flag: if the result is zero
    cpu.reg.flags.S = ((result & 0x80) != 0); // // Sign flag: if bit 7 is set
    cpu.reg.flags.CY = false;
    cpu.reg.flags.P = parity(result); // Parity
}

pub fn updateFlags(result: u16) void {
    cpu.reg.flags.Z = ((result & 0xff) == 0); // zero flag: if the result is zero
    cpu.reg.flags.S = ((result & 0x80) != 0); // // Sign flag: if bit 7 is set
    cpu.reg.flags.CY = (result > 0xff); // Carry flag
    cpu.reg.flags.P = parity(@truncate(u8, result)); // Parity
}

pub fn parity(v: u8) bool {
    var value = v;
    var result: u8 = 0;
    while (value > 0) {
        result ^= value & 1;
        value >>= 1;
    }

    return result == 0;
}

test "parity" {
    var value: u8 = 0b0000_0001;
    var result = parity(value);
    try expectEqual(false, result);

    value = 0b0100_0001;
    result = parity(value);
    try expectEqual(true, result);

    value = 0b0110_0001;
    result = parity(value);
    try expectEqual(false, result);

    value = 0b0110_1001;
    result = parity(value);
    try expectEqual(true, result);

    value = 0b1111_1111;
    result = parity(value);
    try expectEqual(true, result);
}

pub fn fetch8() u8 {
    var byte = memory[cpu.reg._16.PC];
    cpu.reg._16.PC += 1;
    return byte;
}

pub fn fetch16() u16 {
    var bytes = toU16(memory[cpu.reg._16.PC + 1], memory[cpu.reg._16.PC]);
    cpu.reg._16.PC += 2;
    return bytes;
}

pub fn disassemble8080(memory: []u8, pc: u16) !u16 {
    var opBytes: u8 = 1;
    const opCode = memory[pc];

    try stdout.print("{x:0>4} ", .{pc});

    switch (opCode) {
        0x00 => try stdout.print("NOP", .{}),
        0x01 => {
            try stdout.print("LXI    BC,#${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x02 => try stdout.print("STAX   B", .{}),
        0x03 => try stdout.print("INX    B", .{}),
        0x04 => try stdout.print("INR    B", .{}),
        0x05 => try stdout.print("DCR    B", .{}),
        0x06 => {
            try stdout.print("MVI    B,#${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0x07 => try stdout.print("RLC", .{}),
        0x08 => try stdout.print("NOP", .{}),
        0x09 => try stdout.print("DAD B", .{}),
        0x0d => try stdout.print("DCR C", .{}),
        0x0e => {
            try stdout.print("MVI C,#${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0x0f => try stdout.print("RRC", .{}),
        0x11 => {
            try stdout.print("LXI D,#${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x13 => try stdout.print("INX D", .{}),
        0x19 => try stdout.print("DAD D", .{}),
        0x1a => try stdout.print("LDAX D", .{}),
        0x21 => {
            try stdout.print("LXI H,#${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x23 => try stdout.print("INX H", .{}),
        0x26 => {
            try stdout.print("MVI H,#${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0x27 => try stdout.print("DAA", .{}),
        0x29 => try stdout.print("DAD H", .{}),
        0x31 => {
            try stdout.print("LXI SP,#${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x32 => {
            try stdout.print("STA adr #${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x35 => try stdout.print("DCR    M", .{}),
        0x36 => {
            try stdout.print("MVI M,#${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0x3a => {
            try stdout.print("LDA adr #${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0x3e => {
            try stdout.print("MVI    A,#${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0x56 => try stdout.print("MOV D,M", .{}),
        0x5e => try stdout.print("MOV E,M", .{}),
        0x66 => try stdout.print("MOV H,M", .{}),
        0x6f => try stdout.print("MOV L,A", .{}),
        0x77 => try stdout.print("MOV M,A", .{}),
        0x7a => try stdout.print("MOV A,D", .{}),
        0x7b => try stdout.print("MOV A,E", .{}),
        0x7c => try stdout.print("MOV A,H", .{}),
        0x7e => try stdout.print("MOV A,M", .{}),
        0xa7 => try stdout.print("ANA A", .{}),
        0xaf => try stdout.print("XRA A", .{}),
        0xc1 => try stdout.print("POP B", .{}),
        0xc2 => {
            try stdout.print("JNZ adr #${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0xc3 => {
            try stdout.print("JMP    ${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0xc5 => try stdout.print("PUSH B", .{}),
        0xc6 => {
            try stdout.print("ADI #${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0xc9 => try stdout.print("RET", .{}),
        0xca => {
            try stdout.print("JZ    ${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0xcd => {
            try stdout.print("CALL adr #${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0xd1 => try stdout.print("POP D", .{}),
        0xd3 => {
            try stdout.print("OUT #${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0xd5 => try stdout.print("PUSH D", .{}),
        0xda => {
            try stdout.print("JC ${x}{x}", .{ memory[pc + 2], memory[pc + 1] });
            opBytes = 3;
        },
        0xdb => {
            try stdout.print("IN #${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0xe1 => try stdout.print("POP H", .{}),
        0xe5 => try stdout.print("PUSH H", .{}),
        0xe6 => {
            try stdout.print("ANI #${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        0xeb => try stdout.print("XCHG", .{}),
        0xf1 => try stdout.print("POP PSW", .{}),
        0xf5 => try stdout.print("PUSH PSW", .{}),
        0xfb => try stdout.print("EI", .{}),
        0xfe => {
            try stdout.print("CPI #${x}", .{memory[pc + 1]});
            opBytes = 2;
        },
        else => {
            try stdout.print("Unknown Opcode {x}\n", .{opCode});
            try cpu.print();
            @panic("ouch");
        },
    }

    try stdout.print("\n", .{});

    return opBytes;
}

pub fn toU16(highByte: u8, lowByte: u8) u16 {
    return @as(u16, highByte) << 8 | lowByte;
}

test "combine_two_u8_to_u16" {
    const highByte: u8 = 1;
    const lowByte: u8 = 255;
    try expectEqual(@as(u16, 511), toU16(highByte, lowByte));
}
