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

const RiscvInstruction = union(enum) {
    standard: u32,
    compressed: u16,
};

fn rvi32(instruction: u32) RiscvInstruction {
    return .{ .standard = instruction };
}

fn rvi16(instruction: u16) RiscvInstruction {
    return .{ .compressed = instruction };
}

fn riscvInstructionsToBytes(comptime instructions: []const RiscvInstruction) []const u8 {
    var bytes: []const u8 = &.{};
    for (instructions) |i| {
        bytes = bytes ++ switch (i) {
            .standard => |s| &[_]u8{
                @truncate(s),
                @truncate(s >> 8),
                @truncate(s >> 16),
                @truncate(s >> 24),
            },
            .compressed => |c| &[_]u8{ @truncate(c), @truncate(c >> 8) },
        };
    }
    return bytes;
}

// zig fmt: off
const x86_64_prologue = &[_]u8{
    // 24 bytes for saving a FP register across function calls and for saving rbx and rbp
    // for the caller
    // push rbx
    0x53,
    // push rbp
    0x55,
    // use rax as an arbitrary register -- we don't actually care about the pushed value here
    // push rax
    0x50,
    // now copy the arguments into saved registers
    // mov rbx, rdi
    0x48, 0x89, 0xfb,
    // mov rbp, rsi
    0x48, 0x89, 0xf5,
};
// zig fmt: on

const x86_64_epilogue = &[_]u8{
    // restore values from stack
    // pop rax
    0x58,
    // pop rbp
    0x5d,
    // pop rbx
    0x5b,
    // ret
    0xc3,
};

const riscv64_prologue = riscvInstructionsToBytes(&.{
    // allocate 48 bytes of stack space (5*8, rounded up to 16 byte alignment)
    // c.addi16sp sp, -48
    rvi16(0x7179),

    // store ra, fs0, fs1, s0, and s1 on stack
    // c.sdsp ra, 0(sp)
    rvi16(0xe006),
    // c.fsdsp fs0, 8(sp)
    rvi16(0xa422),
    // c.fsdsp fs1, 16(sp)
    rvi16(0xa826),
    // c.sdsp s0, 24(sp)
    rvi16(0xec22),
    // c.sdsp s1, 32(sp)
    rvi16(0xf026),

    // copy a0 and a1 into s0 and s1
    // c.mv s0, a0
    rvi16(0x842a),
    // c.mv s1, a1
    rvi16(0x84ae),
});

const riscv64_epilogue = riscvInstructionsToBytes(&.{
    // restore ra, fs0, fs1, s0, and s1 from stack memory
    // c.ldsp ra, 0(sp)
    rvi16(0x6082),
    // c.fldsp fs0, 8(sp)
    rvi16(0x2422),
    // c.fldsp fs1, 16(sp)
    rvi16(0x24c2),
    // c.ldsp s0, 24(sp)
    rvi16(0x6462),
    // c.ldsp s1, 32(sp)
    rvi16(0x7482),

    // pop our 48 byte stack frame
    // c.addi16sp sp, 48
    rvi16(0x6145),

    // return
    // c.jr ra
    rvi16(0x8082),
});

pub fn emitPrologue(self: *Assembler) !void {
    for (switch (self.target.cpu.arch) {
        .x86_64 => x86_64_prologue,
        .riscv64 => riscv64_prologue,
        else => unreachable,
    }) |byte| {
        try self.emit(u8, byte);
    }
}

pub fn emitEpilogue(self: *Assembler) !void {
    for (switch (self.target.cpu.arch) {
        .x86_64 => x86_64_epilogue,
        .riscv64 => riscv64_epilogue,
        else => unreachable,
    }) |byte| {
        try self.emit(u8, byte);
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
                .vm_stack => 3, // rbx
                .constants => 5, // rbp
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
    if (full_offset == 0 and base_num != 5) {
        try self.emit(u8, (reg_num << 3) | base_num);
        if (base_num == 4) try self.emit(u8, 0x24);
    } else if (std.math.cast(i8, full_offset)) |offset| {
        try self.emit(u8, 0x40 | (reg_num << 3) | base_num);
        if (base_num == 4) try self.emit(u8, 0x24);
        try self.emit(u8, @bitCast(offset));
    } else {
        try self.emit(u8, 0x80 | (reg_num << 3) | base_num);
        if (base_num == 4) try self.emit(u8, 0x24);
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

fn loadRiscv64ImmediateInA2(self: *Assembler, signed_value: i32) !void {
    const value: u32 = @bitCast(signed_value);
    if ((value & 0xfff) < 0x800) {
        // lui a2, value[31:12]
        try self.emit(
            u32,
            value & 0xfffff000 | 0b01100_0110111,
        );
        // addi a2, a2, value[11:0]
        try self.emit(
            u32,
            (value & 0x00000fff) << 20 | 0b01100_000_01100_0010011,
        );
    } else {
        // lui a2, value[31:12] + 1
        try self.emit(
            u32,
            (value + (1 << 12) & 0xfffff000) | 0b01100_0110111,
        );
        // addi a2, a2, value[11:0]
        try self.emit(
            u32,
            (value & 0x00000fff) << 20 | 0b01100_000_01100_0010011,
        );
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
                try self.loadRiscv64ImmediateInA2(src.offset);
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
                try self.loadRiscv64ImmediateInA2(dst.offset);
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
        .x86_64 => {
            // if src != dst, push src to stack
            if (src != dst) {
                // movsd qword [rsp], src
                try self.emit(u8, 0xf2);
                try self.emit(u8, 0x0f);
                try self.emit(u8, 0x11);
                try self.emit(u8, 0x04 | (self.getRegisterNumber(src) << 3));
                try self.emit(u8, 0x24);
            }

            if (src != .a) {
                // movsd xmm0, src
                try self.emit(u8, 0xf2);
                try self.emit(u8, 0x0f);
                try self.emit(u8, 0x10);
                try self.emit(u8, 0xc0 | self.getRegisterNumber(src));
            }

            // call [constants + 8 * index]
            try self.emit(u8, 0xff);
            if (8 * index < 128) {
                try self.emit(u8, 0x50 | self.getRegisterNumber(IntRegister.constants));
                try self.emit(u8, @intCast(8 * index));
            } else {
                try self.emit(u8, 0x90 | self.getRegisterNumber(IntRegister.constants));
                try self.emit(u32, @intCast(8 * index));
            }

            if (dst != .a) {
                // movsd dst, xmm0
                try self.emit(u8, 0xf2);
                try self.emit(u8, 0x0f);
                try self.emit(u8, 0x10);
                try self.emit(u8, 0xc0 | (self.getRegisterNumber(dst) << 3));
            }

            // if src != dst, restore src from stack
            if (src != dst) {
                // movsd src, qword [rsp]
                try self.emit(u8, 0xf2);
                try self.emit(u8, 0x0f);
                try self.emit(u8, 0x10);
                try self.emit(u8, 0x04 | (self.getRegisterNumber(src) << 3));
                try self.emit(u8, 0x24);
            }
        },
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
                try self.loadRiscv64ImmediateInA2(8 * index);
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
        .expected_x86_64_code = x86_64_prologue ++ x86_64_epilogue,
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
            // large offset and bit 11 is 1, so riscv has to account for sign extension
            .{ .load = .{ .src = .{ .base = .vm_stack, .offset = 0x0abbccdd }, .dst = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = x86_64_prologue ++ &[_]u8{
            // movsd xmm0, qword [rbx]
            0xf2, 0x0f, 0x10, 0x03,
            // movsd xmm1, qword [rbp]
            0xf2, 0x0f, 0x10, 0x4d, 0x00,
            // movsd xmm0, qword [rbx + 0x78]
            0xf2, 0x0f, 0x10, 0x43, 0x78,
            // movsd xmm0, qword [rbx - 8]
            0xf2, 0x0f, 0x10, 0x43, 0xf8,
            // movsd xmm0, qword [rbx + 1]
            0xf2, 0x0f, 0x10, 0x43, 0x01,
            // movsd xmm0, qword [rbx + 0x11223344]
            0xf2, 0x0f, 0x10, 0x83, 0x44, 0x33, 0x22, 0x11,
            // movsd xmm0, qword [rbx + 0x0abbccdd]
            0xf2, 0x0f, 0x10, 0x83, 0xdd, 0xcc, 0xbb, 0x0a,
        } ++ x86_64_epilogue,
        .expected_riscv64_code = riscv64_prologue ++ comptime riscvInstructionsToBytes(&.{
            // c.fld fs0, 0(s0)
            rvi16(0x2000),
            // c.fld fs1, 0(s1)
            rvi16(0x2084),
            // c.fld fs0, 0x78(s0)
            rvi16(0x3c20),
            // fld fs0, -8(s0)
            rvi32(0xff843407),
            // fld fs0, 1(s0)
            rvi32(0x00143407),

            // lui a2, 0x11223
            rvi32(0x11223637),
            // addi a2, a2, 0x344
            rvi32(0x34460613),
            // c.add a2, s0
            rvi16(0x9622),
            // c.fld fs0, 0(a2)
            rvi16(0x2200),

            // lui a2, 0x0abbd
            rvi32(0x0abbd637),
            // addi a2, a2, -803
            rvi32(0xcdd60613),
            // c.add a2, s0
            rvi16(0x9622),
            // c.fld fs0, 0(a2)
            rvi16(0x2200),
        }) ++ riscv64_epilogue,
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
            // large offset where riscv accounts for sign extension
            .{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 0x0abbccdd }, .src = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = x86_64_prologue ++ &[_]u8{
            // movsd qword [rbx], xmm0
            0xf2, 0x0f, 0x11, 0x03,
            // movsd qword [rbp], xmm1
            0xf2, 0x0f, 0x11, 0x4d, 0x00,
            // movsd qword [rbx + 0x78], xmm0
            0xf2, 0x0f, 0x11, 0x43, 0x78,
            // movsd qword [rbx - 8], xmm0
            0xf2, 0x0f, 0x11, 0x43, 0xf8,
            // movsd qword [rbx + 1], xmm0
            0xf2, 0x0f, 0x11, 0x43, 0x01,
            // movsd qword [rbx + 0x11223344], xmm0
            0xf2, 0x0f, 0x11, 0x83, 0x44, 0x33, 0x22, 0x11,
            // movsd qword [rbx + 0x0abbccdd], xmm0
            0xf2, 0x0f, 0x11, 0x83, 0xdd, 0xcc, 0xbb, 0x0a,
        } ++ x86_64_epilogue,
        .expected_riscv64_code = riscv64_prologue ++ comptime riscvInstructionsToBytes(&.{
            // c.fsd fs0, 0(s0)
            rvi16(0xa000),
            // c.fsd fs1, 0(s1)
            rvi16(0xa084),
            // c.fsd fs0, 0x78(s0)
            rvi16(0xbc20),
            // fsd fs0, -8(s0)
            rvi32(0xfe843c27),
            // fsd fs0, 1(s0)
            rvi32(0x008430a7),

            // lui a2, 0x11223
            rvi32(0x11223637),
            // addi a2, a2, 0x344
            rvi32(0x34460613),
            // c.add a2, s0
            rvi16(0x9622),
            // c.fsd fs0, 0(a2)
            rvi16(0xa200),

            // lui a2, 0x0abbd
            rvi32(0x0abbd637),
            // addi a2, a2, -803
            rvi32(0xcdd60613),
            // c.add a2, s0
            rvi16(0x9622),
            // c.fsd fs0, 0(a2)
            rvi16(0xa200),
        }) ++ riscv64_epilogue,
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
        .expected_x86_64_code = x86_64_prologue ++ &[_]u8{
            // vaddsd xmm0, xmm0, xmm0
            0xc5, 0xfb, 0x58, 0xc0,
            // vaddsd xmm0, xmm0, xmm1
            0xc5, 0xfb, 0x58, 0xc1,
            // vaddsd xmm0, xmm1, xmm0
            0xc5, 0xf3, 0x58, 0xc0,
            // vaddsd xmm1, xmm0, xmm0
            0xc5, 0xfb, 0x58, 0xc8,
        } ++ x86_64_epilogue,
        .expected_riscv64_code = riscv64_prologue ++ comptime riscvInstructionsToBytes(&.{
            // fadd.d fs0, fs0, fs0
            rvi32(0x02847453),
            // fadd.d fs0, fs0, fs1
            rvi32(0x02947453),
            // fadd.d fs0, fs1, fs0
            rvi32(0x0284f453),
            // fadd.d fs1, fs0, fs0
            rvi32(0x028474d3),
        }) ++ riscv64_epilogue,
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
        .expected_x86_64_code = x86_64_prologue ++ &[_]u8{
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
        } ++ x86_64_epilogue,
        .expected_riscv64_code = riscv64_prologue ++ comptime riscvInstructionsToBytes(&.{
            // fsub.d fs0, fs0, fs1
            rvi32(0x0a947453),
            // fsub.d fs0, fs1, fs0
            rvi32(0x0a84f453),
            // fmul.d fs0, fs0, fs1
            rvi32(0x12947453),
            // fmul.d fs0, fs1, fs0
            rvi32(0x1284f453),
            // fdiv.d fs0, fs0, fs1
            rvi32(0x1a947453),
            // fdiv.d fs0, fs1, fs0
            rvi32(0x1a84f453),
        }) ++ riscv64_epilogue,
        // zig fmt: on
    });
}

test "assemble calls" {
    try runAssemblerTest(.{
        .instructions = &.{
            // compressed load for risc-v
            .{ .call_unary = .{ .index = 5, .src = .a, .dst = .b } },
            // uncompressed load + different regs
            .{ .call_unary = .{ .index = 32, .src = .b, .dst = .a } },
            // long sequence
            .{ .call_unary = .{ .index = 256, .src = .a, .dst = .a } },
        },
        // zig fmt: off
        .expected_x86_64_code = x86_64_prologue ++ &[_]u8{
            // movsd qword [rsp], xmm0
            0xf2, 0x0f, 0x11, 0x04, 0x24,
            // call [rbp + 40]
            0xff, 0x55, 0x28,
            // movsd xmm1, xmm0
            0xf2, 0x0f, 0x10, 0xc8,
            // movsd xmm0, qword [rsp]
            0xf2, 0x0f, 0x10, 0x04, 0x24,

            // movsd qword [rsp], xmm1
            0xf2, 0x0f, 0x11, 0x0c, 0x24,
            // movsd xmm0, xmm1
            0xf2, 0x0f, 0x10, 0xc1,
            // call [rbp + 256]
            0xff, 0x95, 0x00, 0x01, 0x00, 0x00,
            // movsd xmm1, qword [rsp]
            0xf2, 0x0f, 0x10, 0x0c, 0x24,

            // call [rbp + 2048]
            0xff, 0x95, 0x00, 0x08, 0x00, 0x00,
        } ++ x86_64_epilogue,
        .expected_riscv64_code = riscv64_prologue ++ comptime riscvInstructionsToBytes(&.{
            // fsgnj.d fa0, fs0, fs0
            rvi32(0x22840553),
            // c.ld a0, 40(s1)
            rvi16(0x7488),
            // c.jalr a0
            rvi16(0x9502),
            // fsgnj.d fs1, fa0, fa0
            rvi32(0x22a504d3),

            // fsgnj.d fa0, fs1, fs1
            rvi32(0x22948553),
            // ld a0, 256(s1)
            rvi32(0x1004b503),
            // c.jalr a0
            rvi16(0x9502),
            // fsgnj.d fs0, fa0, fa0
            rvi32(0x22a50453),

            // fsgnj.d fa0, fs1, fs1
            rvi32(0x22840553),
            // lui a2, 0
            rvi32(0x00001637),
            // addi a2, a2, 0x800
            rvi32(0x80060613),
            // c.add a2, s1
            rvi16(0x9626),
            // c.ld a0, 0(a2)
            rvi16(0x6208),
            // c.jalr a0
            rvi16(0x9502),
            // fsgnj.d fs0, fa0, fa0
            rvi32(0x22a50453),
        }) ++ riscv64_epilogue,
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
