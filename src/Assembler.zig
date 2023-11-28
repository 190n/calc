const std = @import("std");
const builtin = @import("builtin");

const Operation = @import("./Program.zig").Operation;
const Constant = @import("./Compiler.zig").Constant;

const Assembler = @This();

pub const calling_convention: std.builtin.CallingConvention = switch (builtin.cpu.arch) {
    // unify callconv between linux and windows
    .x86_64 => .SysV,
    else => .C,
};

pub const CompiledCode = *const fn (
    vm_stack: [*]f64,
    constants: [*]const Constant,
) callconv(calling_convention) void;

const page_size = std.mem.page_size;

target: std.Target,
code: union(enum) {
    writable: std.ArrayListAlignedUnmanaged(u8, page_size),
    executable: []align(page_size) const u8,
},
allocator: std.mem.Allocator,

pub const IntRegister = enum {
    /// register used for the first function argument (where a pointer to the VM stack is passed)
    vm_stack,
    /// register used for the second function argument (where a pointer to the VM constants is
    /// passed)
    constants,
};

pub const FloatRegister = enum {
    /// callee saved (non volatile) FP registers
    a,
    b,
};

pub const BaseRegisterAndOffset = struct {
    base: IntRegister,
    offset: i32,
};

pub const Operand = union(enum) {
    int_register: IntRegister,
    float_register: FloatRegister,
    memory: BaseRegisterAndOffset,
};

pub const Instruction = union(enum) {
    load: struct {
        dst: FloatRegister,
        src: BaseRegisterAndOffset,
    },
    store: struct {
        dst: BaseRegisterAndOffset,
        src: FloatRegister,
    },
    add_float: struct {
        dst: FloatRegister,
        src1: FloatRegister,
        src2: FloatRegister,
    },
    sub_float: struct {
        dst: FloatRegister,
        src1: FloatRegister,
        src2: FloatRegister,
    },
    mul_float: struct {
        dst: FloatRegister,
        src1: FloatRegister,
        src2: FloatRegister,
    },
    div_float: struct {
        dst: FloatRegister,
        src1: FloatRegister,
        src2: FloatRegister,
    },
    call_unary: struct {
        /// index into constants array
        index: u28,
        dst: FloatRegister,
        src: FloatRegister,
    },
};

pub fn init(target: std.Target, allocator: std.mem.Allocator) !Assembler {
    return .{
        .target = target,
        .code = .{
            .writable = try std.ArrayListAlignedUnmanaged(u8, page_size).initCapacity(
                allocator,
                page_size,
            ),
        },
        .allocator = allocator,
    };
}

fn riscvInstructionsToBytes(comptime instructions: []const u16) []const u8 {
    var bytes: []const u8 = &.{};
    for (instructions) |i| {
        bytes = bytes ++ &[_]u8{ @truncate(i), @truncate(i >> 8) };
    }
    return bytes;
}

const riscv64_prologue = riscvInstructionsToBytes(&.{
    // allocate 48 bytes of stack space (5*8, rounded up to 16 byte alignment)
    // c.addi16sp sp, -48
    0x7179,

    // store ra, fs0, fs1, s0, and s1 on stack
    // c.sdsp ra, 0(sp)
    0xe006,
    // c.fsdsp fs0, 8(sp)
    0xa422,
    // c.fsdsp fs1, 16(sp)
    0xa826,
    // c.sdsp s0, 24(sp)
    0xec22,
    // c.sdsp s1, 32(sp)
    0xf026,

    // copy a0 and a1 into s0 and s1
    // c.mv s0, a0
    0x842a,
    // c.mv s1, a1
    0x84ae,
});

const riscv64_epilogue = riscvInstructionsToBytes(&.{
    // restore ra, fs0, fs1, s0, and s1 from stack memory
    // c.ldsp ra, 0(sp)
    0x6082,
    // c.fldsp fs0, 8(sp)
    0x2422,
    // c.fldsp fs1, 16(sp)
    0x24c2,
    // c.ldsp s0, 24(sp)
    0x6462,
    // c.ldsp s1, 32(sp)
    0x7482,

    // pop our 48 byte stack frame
    // c.addi16sp sp, 48
    0x6145,

    // return
    // c.jr ra
    0x8082,
});

pub fn emitPrologue(self: *Assembler) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // TODO save return address?
        },
        .riscv64 => {
            for (riscv64_prologue) |byte| {
                try self.emit(u8, byte);
            }
        },
        else => unreachable,
    }
}

pub fn emitEpilogue(self: *Assembler) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => try self.emit(u8, 0xc3), // ret
        .riscv64 => {
            for (riscv64_epilogue) |byte| {
                try self.emit(u8, byte);
            }
        },
        else => unreachable,
    }
}

pub fn finalize(self: *Assembler) !void {
    // init and emit ensure that the length of the allocated slice is always aligned to pages
    const memory = self.code.writable.allocatedSlice();
    try std.os.mprotect(memory, std.os.PROT.READ | std.os.PROT.EXEC);
    self.code = .{ .executable = memory };
}

pub fn getFunctionPointer(self: *Assembler) CompiledCode {
    return @ptrCast(self.code.executable.ptr);
}

pub fn deinit(self: *Assembler) void {
    switch (self.code) {
        .writable => |_| self.code.writable.deinit(self.allocator),
        .executable => |slice| {
            // we need to make the slice writable and non-executable since zig will try to @memset
            // it to 0xaa in debug builds
            std.os.mprotect(@constCast(slice), std.os.PROT.READ | std.os.PROT.WRITE) catch return;
            self.allocator.free(slice);
        },
    }
    self.* = undefined;
}

pub fn getRegisterNumber(self: Assembler, reg: anytype) u8 {
    return switch (@TypeOf(reg)) {
        IntRegister => switch (self.target.cpu.arch) {
            .x86_64 => switch (reg) {
                .vm_stack => 7, // rdi
                .constants => 6, // rsi
            },
            .riscv64 => switch (reg) {
                .vm_stack => 8, // s0
                .constants => 9, // s1
            },
            else => unreachable,
        },
        FloatRegister => switch (self.target.cpu.arch) {
            .x86_64 => switch (reg) {
                .a => 0, // xmm0
                .b => 1, // xmm1
            },
            .riscv64 => switch (reg) {
                .a => 8, // fs0
                .b => 9, // fs1
            },
            else => unreachable,
        },
        else => unreachable,
    };
}

fn emitX86Offset(self: *Assembler, base: IntRegister, full_offset: i32, register_operand: FloatRegister) !void {
    const base_num = self.getRegisterNumber(base);
    const reg_num = self.getRegisterNumber(register_operand);
    if (full_offset == 0) {
        try self.emit(u8, (reg_num << 3) | base_num);
    } else if (std.math.cast(i8, full_offset)) |offset| {
        try self.emit(u8, 0x40 | (reg_num << 3) | base_num);
        try self.emit(u8, @bitCast(offset));
    } else {
        try self.emit(u8, 0x80 | (reg_num << 3) | base_num);
        try self.emit(u32, @bitCast(full_offset));
    }
}

fn emit(self: *Assembler, comptime T: type, data: T) !void {
    if (T == u8) {
        // allocate a new page if needed to hold the new byte
        try self.code.writable.ensureTotalCapacityPrecise(
            self.allocator,
            std.mem.alignForward(usize, self.code.writable.items.len + 1, page_size),
        );
        self.code.writable.appendAssumeCapacity(data);
    } else {
        const HalfT = switch (T) {
            u64 => u32,
            u32 => u16,
            u16 => u8,
            else => unreachable,
        };
        const shift_amount = 8 * @sizeOf(HalfT);
        switch (self.target.cpu.arch.endian()) {
            .little => {
                try self.emit(HalfT, @truncate(data));
                try self.emit(HalfT, @truncate(data >> shift_amount));
            },
            .big => {
                try self.emit(HalfT, @truncate(data >> shift_amount));
                try self.emit(HalfT, @truncate(data));
            },
        }
    }
}

fn assembleLoad(self: *Assembler, dst: FloatRegister, src: BaseRegisterAndOffset) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // movsd dst, qword [base + offset]
            try self.emit(u8, 0xf2);
            try self.emit(u8, 0x0f);
            try self.emit(u8, 0x10);
            try self.emitX86Offset(src.base, src.offset, dst);
        },
        .riscv64 => {
            if (@mod(src.offset, 8) == 0 and std.math.cast(u8, src.offset) != null) {
                // c.fld dst, offset(base)
                const offset: u8 = @intCast(src.offset);
                try self.emit(
                    u16,
                    0b001_00000000000_00 |
                        @as(u16, self.getRegisterNumber(src.base) & 0b111) << 7 |
                        (self.getRegisterNumber(dst) & 0b111) << 2 |
                        @as(u16, offset & 0b00111000) << 7 |
                        (offset & 0b11000000) >> 1,
                );
            } else if (std.math.cast(i12, src.offset)) |immediate_offset| {
                // fld dst, offset(base)
                try self.emit(
                    u32,
                    0b011_00000_0000111 |
                        @as(u32, @as(u12, @bitCast(immediate_offset))) << 20 |
                        @as(u32, self.getRegisterNumber(src.base)) << 15 |
                        @as(u32, self.getRegisterNumber(dst)) << 7,
                );
            } else {
                const offset: u32 = @bitCast(src.offset);
                // lui a2, offset[31:12]
                try self.emit(
                    u32,
                    offset & 0xfffff000 | 0b01100_0110111,
                );
                // addi a2, a2, offset[11:0]
                try self.emit(
                    u32,
                    (offset & 0x00000fff) << 20 | 0b01100_000_01100_0010011,
                );
                // c.add a2, base
                try self.emit(
                    u16,
                    0b100_1_01100_00000_10 | @as(u16, self.getRegisterNumber(src.base) << 2),
                );
                // c.fld dst, 0(a2)
                try self.emit(
                    u16,
                    0b001_000_100_00_000_00 |
                        @as(u16, self.getRegisterNumber(dst) & 0b111) << 2,
                );
            }
        },
        else => unreachable,
    }
}

fn assembleStore(self: *Assembler, dst: BaseRegisterAndOffset, src: FloatRegister) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // movsd qword [base + offset], src
            try self.emit(u8, 0xf2);
            try self.emit(u8, 0x0f);
            try self.emit(u8, 0x11);
            try self.emitX86Offset(dst.base, dst.offset, src);
        },
        .riscv64 => {
            if (@mod(dst.offset, 8) == 0 and std.math.cast(u8, dst.offset) != null) {
                // c.fsd src, offset(base)
                const offset: u8 = @intCast(dst.offset);
                try self.emit(
                    u16,
                    0b101_00000000000_00 |
                        @as(u16, self.getRegisterNumber(dst.base) & 0b111) << 7 |
                        (self.getRegisterNumber(src) & 0b111) << 2 |
                        @as(u16, offset & 0b111000) << 7 |
                        (offset & 0b11000000) >> 1,
                );
            } else if (std.math.cast(i12, dst.offset)) |immediate_offset| {
                // fsd src, offset(base)
                const offset: u32 = @as(u12, @bitCast(immediate_offset));
                try self.emit(
                    u32,
                    0b011_00000_0100111 |
                        (offset >> 5) << 25 |
                        @as(u32, self.getRegisterNumber(src)) << 20 |
                        @as(u32, self.getRegisterNumber(dst.base)) << 15 |
                        (offset & 0b11111) << 7,
                );
            } else {
                const offset: u32 = @bitCast(dst.offset);
                // lui a2, offset[31:12]
                try self.emit(
                    u32,
                    offset & 0xfffff000 | 0b01100_0110111,
                );
                // addi a2, a2, offset[11:0]
                try self.emit(
                    u32,
                    (offset & 0x00000fff) << 20 | 0b01100_000_01100_0010011,
                );
                // c.add a2, base
                try self.emit(
                    u16,
                    0b100_1_01100_00000_10 | @as(u16, self.getRegisterNumber(dst.base) << 2),
                );
                // c.fsd src, 0(a2)
                try self.emit(
                    u16,
                    0b101_000_100_00_000_00 |
                        @as(u16, self.getRegisterNumber(src) & 0b111) << 2,
                );
            }
        },
        else => unreachable,
    }
}

fn assembleAddFloat(self: *Assembler, dst: FloatRegister, src1: FloatRegister, src2: FloatRegister) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // vaddsd dst, src1, src2
            try self.emit(u8, 0xc5);
            try self.emit(u8, 0xc3 | (~self.getRegisterNumber(src1) & 0b111) << 3);
            try self.emit(u8, 0x58);
            try self.emit(u8, 0xc0 | self.getRegisterNumber(dst) << 3 | self.getRegisterNumber(src2));
        },
        .riscv64 => {
            // fadd.d dst, src1, src2
            try self.emit(u32, 0b0000001_00000_00000_111_00000_1010011 |
                @as(u32, self.getRegisterNumber(src2)) << 20 |
                @as(u32, self.getRegisterNumber(src1)) << 15 |
                @as(u32, self.getRegisterNumber(dst)) << 7);
        },
        else => unreachable,
    }
}

fn assembleSubFloat(self: *Assembler, dst: FloatRegister, src1: FloatRegister, src2: FloatRegister) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // vsubsd dst, src1, src2
            try self.emit(u8, 0xc5);
            try self.emit(u8, 0xc3 | (~self.getRegisterNumber(src1) & 0b111) << 3);
            try self.emit(u8, 0x5c);
            try self.emit(u8, 0xc0 | self.getRegisterNumber(dst) << 3 | self.getRegisterNumber(src2));
        },
        .riscv64 => {
            // fsub.d dst, src1, src2
            try self.emit(u32, 0b0000101_00000_00000_111_00000_1010011 |
                @as(u32, self.getRegisterNumber(src2)) << 20 |
                @as(u32, self.getRegisterNumber(src1)) << 15 |
                @as(u32, self.getRegisterNumber(dst)) << 7);
        },
        else => unreachable,
    }
}

fn assembleMulFloat(self: *Assembler, dst: FloatRegister, src1: FloatRegister, src2: FloatRegister) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // vmulsd dst, src1, src2
            try self.emit(u8, 0xc5);
            try self.emit(u8, 0xc3 | (~self.getRegisterNumber(src1) & 0b111) << 3);
            try self.emit(u8, 0x59);
            try self.emit(u8, 0xc0 | self.getRegisterNumber(dst) << 3 | self.getRegisterNumber(src2));
        },
        .riscv64 => {
            // fmul.d dst, src1, src2
            try self.emit(u32, 0b0001001_00000_00000_111_00000_1010011 |
                @as(u32, self.getRegisterNumber(src2)) << 20 |
                @as(u32, self.getRegisterNumber(src1)) << 15 |
                @as(u32, self.getRegisterNumber(dst)) << 7);
        },
        else => unreachable,
    }
}

fn assembleDivFloat(self: *Assembler, dst: FloatRegister, src1: FloatRegister, src2: FloatRegister) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {
            // vaddsd dst, src1, src2
            try self.emit(u8, 0xc5);
            try self.emit(u8, 0xc3 | (~self.getRegisterNumber(src1) & 0b111) << 3);
            try self.emit(u8, 0x5e);
            try self.emit(u8, 0xc0 | self.getRegisterNumber(dst) << 3 | self.getRegisterNumber(src2));
        },
        .riscv64 => {
            // fdiv.d dst, src1, src2
            try self.emit(u32, 0b0001101_00000_00000_111_00000_1010011 |
                @as(u32, self.getRegisterNumber(src2)) << 20 |
                @as(u32, self.getRegisterNumber(src1)) << 15 |
                @as(u32, self.getRegisterNumber(dst)) << 7);
        },
        else => unreachable,
    }
}

fn assembleCallUnary(
    self: *Assembler,
    index: u28,
    dst: FloatRegister,
    src: FloatRegister,
) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => {},
        .riscv64 => {
            // copy argument register into fa0
            // fsgnj.d fa0, src, src
            try self.emit(
                u32,
                0b0010001_00000_00000_000_01010_1010011 |
                    @as(u32, self.getRegisterNumber(src)) << 20 |
                    @as(u32, self.getRegisterNumber(src)) << 15,
            );
            // load address to call into a0
            if (std.math.cast(u8, 8 * index)) |small_offset| {
                // c.ld a0, small_offset(constants)
                try self.emit(
                    u16,
                    0b011_000_000_00_010_00 |
                        @as(u16, self.getRegisterNumber(IntRegister.constants) & 0b111) << 7 |
                        @as(u16, small_offset & 0b00111000) << 7 |
                        (small_offset & 0b11000000) >> 1,
                );
            } else if (std.math.cast(i12, 8 * index)) |med_offset| {
                // ld a0, med_offset(constants)
                try self.emit(
                    u32,
                    0b011_01010_0000011 |
                        @as(u32, @as(u12, @bitCast(med_offset))) << 20 |
                        @as(u32, self.getRegisterNumber(IntRegister.constants)) << 15,
                );
            } else {
                const offset = 8 * index;
                // lui a2, offset[31:12]
                try self.emit(
                    u32,
                    offset & 0x0ffff000 | 0b01100_0110111,
                );
                // addi a2, a2, offset[11:0]
                try self.emit(
                    u32,
                    (offset & 0x00000fff) << 20 | 0b01100_000_01100_0010011,
                );
                // c.add a2, constants
                try self.emit(
                    u16,
                    0b100_1_01100_00000_10 | @as(u16, self.getRegisterNumber(IntRegister.constants) << 2),
                );
                // c.ld a0, 0(a2)
                try self.emit(u16, 0x6208);
            }
            // call it
            // c.jalr a0
            try self.emit(u16, 0x9502);
            // copy return value into destination register
            // fsgnj.d dst, fa0, fa0
            try self.emit(
                u32,
                0b0010001_01010_01010_000_00000_1010011 |
                    @as(u32, self.getRegisterNumber(dst)) << 7,
            );
        },
        else => unreachable,
    }
}

pub fn assemble(self: *Assembler, inst: Instruction) !void {
    switch (inst) {
        .load => |load| try self.assembleLoad(load.dst, load.src),
        .store => |store| try self.assembleStore(store.dst, store.src),
        .add_float => |add_float| try self.assembleAddFloat(add_float.dst, add_float.src1, add_float.src2),
        .sub_float => |sub_float| try self.assembleSubFloat(sub_float.dst, sub_float.src1, sub_float.src2),
        .mul_float => |mul_float| try self.assembleMulFloat(mul_float.dst, mul_float.src1, mul_float.src2),
        .div_float => |div_float| try self.assembleDivFloat(div_float.dst, div_float.src1, div_float.src2),
        .call_unary => |call_unary| try self.assembleCallUnary(call_unary.index, call_unary.dst, call_unary.src),
    }
}

const AssemblerTestCase = struct {
    instructions: []const Instruction,
    expected_x86_64_code: []const u8,
    expected_riscv64_code: []const u8,
};

const x86_64_target = std.Target{
    .abi = .gnu,
    .cpu = std.Target.Cpu.baseline(.x86_64),
    .os = .{
        .tag = .linux,
        .version_range = std.Target.Os.VersionRange.default(.linux, .x86_64),
    },
    .ofmt = .elf,
};

const riscv64_target = std.Target{
    .abi = .gnu,
    .cpu = std.Target.Cpu.baseline(.riscv64),
    .os = .{
        .tag = .linux,
        .version_range = std.Target.Os.VersionRange.default(.linux, .riscv64),
    },
    .ofmt = .elf,
};

fn runAssemblerTest(info: AssemblerTestCase) !void {
    var x86_64_assembler = try Assembler.init(x86_64_target, std.testing.allocator);
    defer x86_64_assembler.deinit();
    var riscv64_assembler = try Assembler.init(riscv64_target, std.testing.allocator);
    defer riscv64_assembler.deinit();

    try x86_64_assembler.emitPrologue();
    try riscv64_assembler.emitPrologue();
    for (info.instructions) |inst| {
        try x86_64_assembler.assemble(inst);
        try riscv64_assembler.assemble(inst);
    }
    try x86_64_assembler.emitEpilogue();
    try riscv64_assembler.emitEpilogue();

    try std.testing.expectEqualSlices(u8, info.expected_x86_64_code, x86_64_assembler.code.writable.items);
    try std.testing.expectEqualSlices(u8, info.expected_riscv64_code, riscv64_assembler.code.writable.items);
}

test "assemble empty program" {
    // empty/only return instruction
    try runAssemblerTest(.{
        .instructions = &.{},
        .expected_x86_64_code = &.{0xc3},
        .expected_riscv64_code = riscv64_prologue ++ riscv64_epilogue,
    });
}

test "assemble loads" {
    // loads with various offset sizes
    try runAssemblerTest(.{
        .instructions = &.{
            // different registers
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = 0 }, .dst = .a } },
            .{ .load = .{ .src = .{ .base = .constants, .offset = 0 }, .dst = .b } },
            // offset 0x78 = 1 byte for x86, still c.fld for riscv
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = 0x78 }, .dst = .a } },
            // offset -8 = 1 byte for x86, uncompressed load for riscv since it is negative
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = -8 }, .dst = .a } },
            // small offset, but misaligned, so riscv can't use compressed load
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = 1 }, .dst = .a } },
            // large offset. x86 uses 32-bit immediate and riscv uses several instructions.
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = 0x11223344 }, .dst = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = &.{
            // movsd xmm0, qword [rdi]
            0xf2, 0x0f, 0x10, 0x07,
            // movsd xmm1, qword [rsi]
            0xf2, 0x0f, 0x10, 0x0e,
            // movsd xmm0, qword [rdi + 0x78]
            0xf2, 0x0f, 0x10, 0x47, 0x78,
            // movsd xmm0, qword [rdi - 8]
            0xf2, 0x0f, 0x10, 0x47, 0xf8,
            // movsd xmm0, qword [rdi + 1]
            0xf2, 0x0f, 0x10, 0x47, 0x01,
            // movsd xmm0, qword [rdi + 0x11223344]
            0xf2, 0x0f, 0x10, 0x87, 0x44, 0x33, 0x22, 0x11,
            // ret
            0xc3,
        },
        .expected_riscv64_code = riscv64_prologue ++ &[_]u8{
            // c.fld fs0, 0(s0)
            0x00, 0x20,
            // c.fld fs1, 0(s1)
            0x84, 0x20,
            // c.fld fs0, 0x78(s0)
            0x20, 0x3c,
            // fld fs0, -8(s0)
            0x07, 0x34, 0x84, 0xff,
            // fld fs0, 1(s0)
            0x07, 0x34, 0x14, 0x00,

            // lui a2, 0x11223
            0x37, 0x36, 0x22, 0x11,
            // addi a2, a2, 0x344
            0x13, 0x06, 0x46, 0x34,
            // c.add a2, s0
            0x22, 0x96,
            // c.fld fs0, 0(a2)
            0x00, 0x22,
        } ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assemble stores" {
    try runAssemblerTest(.{
        .instructions = &.{
            // different registers
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 0 }, .src = .a } },
            .{ .store = .{ .dst = .{ .base = .constants, .offset = 0 }, .src = .b } },
            // offset 0x78 = 1 byte for x86, still c.fld for riscv
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 0x78 }, .src = .a } },
            // offset -8 = 1 byte for x86, uncompressed store for riscv since it is negative
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = -8 }, .src = .a } },
            // small offset, but misaligned, so riscv can't use compressed store
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 1 }, .src = .a } },
            // large offset. x86 uses 32-bit immediate and riscv uses several instructions.
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 0x11223344 }, .src = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = &.{
            // movsd qword [rdi], xmm0
            0xf2, 0x0f, 0x11, 0x07,
            // movsd qword [rsi], xmm1
            0xf2, 0x0f, 0x11, 0x0e,
            // movsd qword [rdi + 0x78], xmm0
            0xf2, 0x0f, 0x11, 0x47, 0x78,
            // movsd qword [rdi - 8], xmm0
            0xf2, 0x0f, 0x11, 0x47, 0xf8,
            // movsd qword [rdi + 1], xmm0
            0xf2, 0x0f, 0x11, 0x47, 0x01,
            // movsd qword [rdi + 0x11223344], xmm0
            0xf2, 0x0f, 0x11, 0x87, 0x44, 0x33, 0x22, 0x11,
            // ret
            0xc3,
        },
        .expected_riscv64_code = riscv64_prologue ++ &[_]u8{
            // c.fsd fs0, 0(s0)
            0x00, 0xa0,
            // c.fsd fs1, 0(s1)
            0x84, 0xa0,
            // c.fsd fs0, 0x78(s0)
            0x20, 0xbc,
            // fsd fs0, -8(s0)
            0x27, 0x3c, 0x84, 0xfe,
            // fsd fs0, 1(s0)
            0xa7, 0x30, 0x84, 0x00,

            // lui a2, 0x11223
            0x37, 0x36, 0x22, 0x11,
            // addi a2, a2, 0x344
            0x13, 0x06, 0x46, 0x34,
            // c.add a2, s0
            0x22, 0x96,
            // c.fsd fs0, 0(a2)
            0x00, 0xa2,
        } ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assemble adds" {
    try runAssemblerTest(.{
        .instructions = &.{
            .{ .add_float = .{ .dst = .a, .src1 = .a, .src2 = .a } },
            .{ .add_float = .{ .dst = .a, .src1 = .a, .src2 = .b } },
            .{ .add_float = .{ .dst = .a, .src1 = .b, .src2 = .a } },
            .{ .add_float = .{ .dst = .b, .src1 = .a, .src2 = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = &.{
            // vaddsd xmm0, xmm0, xmm0
            0xc5, 0xfb, 0x58, 0xc0,
            // vaddsd xmm0, xmm0, xmm1
            0xc5, 0xfb, 0x58, 0xc1,
            // vaddsd xmm0, xmm1, xmm0
            0xc5, 0xf3, 0x58, 0xc0,
            // vaddsd xmm1, xmm0, xmm0
            0xc5, 0xfb, 0x58, 0xc8,
            // ret
            0xc3,
        },
        .expected_riscv64_code = riscv64_prologue ++ &[_]u8{
            // fadd.d fs0, fs0, fs0
            0x53, 0x74, 0x84, 0x02,
            // fadd.d fs0, fs0, fs1
            0x53, 0x74, 0x94, 0x02,
            // fadd.d fs0, fs1, fs0
            0x53, 0xf4, 0x84, 0x02,
            // fadd.d fs1, fs0, fs0
            0xd3, 0x74, 0x84, 0x02,
        } ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assemble other float arithmetic" {
    try runAssemblerTest(.{
        .instructions = &.{
            .{ .sub_float = .{ .dst = .a, .src1 = .a, .src2 = .b } },
            .{ .sub_float = .{ .dst = .a, .src1 = .b, .src2 = .a } },
            .{ .mul_float = .{ .dst = .a, .src1 = .a, .src2 = .b } },
            .{ .mul_float = .{ .dst = .a, .src1 = .b, .src2 = .a } },
            .{ .div_float = .{ .dst = .a, .src1 = .a, .src2 = .b } },
            .{ .div_float = .{ .dst = .a, .src1 = .b, .src2 = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = &.{
            // vsubsd xmm0, xmm0, xmm1
            0xc5, 0xfb, 0x5c, 0xc1,
            // vsubsd xmm0, xmm1, xmm0
            0xc5, 0xf3, 0x5c, 0xc0,
            // vmulsd xmm0, xmm0, xmm1
            0xc5, 0xfb, 0x59, 0xc1,
            // vmulsd xmm0, xmm1, xmm0
            0xc5, 0xf3, 0x59, 0xc0,
            // vdivsd xmm0, xmm0, xmm1
            0xc5, 0xfb, 0x5e, 0xc1,
            // vdivsd xmm0, xmm1, xmm0
            0xc5, 0xf3, 0x5e, 0xc0,
            // ret
            0xc3,
        },
        .expected_riscv64_code = riscv64_prologue ++ &[_]u8{
            // fsub.d fs0, fs0, fs1
            0x53, 0x74, 0x94, 0x0a,
            // fsub.d fs0, fs1, fs0
            0x53, 0xf4, 0x84, 0x0a,
            // fmul.d fs0, fs0, fs1
            0x53, 0x74, 0x94, 0x12,
            // fmul.d fs0, fs1, fs0
            0x53, 0xf4, 0x84, 0x12,
            // fdiv.d fs0, fs0, fs1
            0x53, 0x74, 0x94, 0x1a,
            // fdiv.d fs0, fs1, fs0
            0x53, 0xf4, 0x84, 0x1a,
        } ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assemble calls" {
    try runAssemblerTest(.{
        .instructions = &.{
            .{ .call_unary = .{ .index = 5, .src = .a, .dst = .b } },
        },
        // zig fmt: off
        .expected_x86_64_code = &.{0xc3},
        .expected_riscv64_code = riscv64_prologue ++ &[_]u8{
            // fsgnj.d fa0, fs0, fs0
            0x53, 0x05, 0x84, 0x22,
            // c.ld a0, 40(s1)
            0x88, 0x74,
            // c.jalr a0
            0x02, 0x95,
            // fsgnj.d fs1, fa0, fa0
            0xd3, 0x04, 0xa5, 0x22,
        } ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assembler buffer length is always aligned" {
    var assembler = try Assembler.init(x86_64_target, std.testing.allocator);
    defer assembler.deinit();
    try std.testing.expectEqual(@as(usize, 4096), assembler.code.writable.capacity);
    for (0..4097) |_| {
        try assembler.emit(u8, 0);
    }
    try std.testing.expectEqual(@as(usize, 8192), assembler.code.writable.capacity);
    for (0..4096) |_| {
        try assembler.emit(u8, 0);
    }
    try std.testing.expectEqual(@as(usize, 12288), assembler.code.writable.capacity);
}
