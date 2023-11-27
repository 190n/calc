const std = @import("std");

const Operation = @import("./Program.zig").Operation;

const Assembler = @This();

pub const CompiledCode = *const fn (vm_stack: [*]f64, constants: [*]const f64) callconv(.SysV) void;

const page_size = std.mem.page_size;
const page_allocator = std.heap.page_allocator;

target: std.Target,
code: union(enum) {
    writable: std.ArrayListAlignedUnmanaged(u8, page_size),
    executable: []align(page_size) const u8,
},

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
};

pub fn init(target: std.Target) !Assembler {
    return .{
        .target = target,
        .code = .{
            .writable = try std.ArrayListAlignedUnmanaged(u8, page_size).initCapacity(
                page_allocator,
                page_size,
            ),
        },
    };
}

pub fn emitPrologue(self: *Assembler) !void {
    // TODO save return address?
    switch (self.target.cpu.arch) {
        .x86_64 => {},
        .riscv64 => {},
        else => unreachable,
    }
}

pub fn emitEpilogue(self: *Assembler) !void {
    switch (self.target.cpu.arch) {
        .x86_64 => try self.emit(u8, 0xc3), // ret
        .riscv64 => try self.emit(u16, 0x8082), // c.jr ra
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
        .writable => |_| self.code.writable.deinit(page_allocator),
        .executable => |slice| {
            // we need to make the slice writable and non-executable since zig will try to @memset
            // it to 0xaa in debug builds
            std.os.mprotect(@constCast(slice), std.os.PROT.READ | std.os.PROT.WRITE) catch return;
            page_allocator.free(slice);
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
                .vm_stack => 10, // a0
                .constants => 11, // a1
            },
            else => unreachable,
        },
        FloatRegister => switch (self.target.cpu.arch) {
            .x86_64 => switch (reg) {
                .a => 0, // xmm0
                .b => 1, // xmm1
            },
            .riscv64 => switch (reg) {
                .a => 10, // fa0
                .b => 11, // fa1
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
            page_allocator,
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
            .Little => {
                try self.emit(HalfT, @truncate(data));
                try self.emit(HalfT, @truncate(data >> shift_amount));
            },
            .Big => {
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
            try self.emit(u8, 0xc3 | (~self.getRegisterNumber(src2) & 0b111) << 3);
            try self.emit(u8, 0x58);
            try self.emit(u8, 0xc0 | self.getRegisterNumber(src1) | self.getRegisterNumber(dst) << 3);
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

pub fn assemble(self: *Assembler, inst: Instruction) !void {
    switch (inst) {
        .load => |load| try self.assembleLoad(load.dst, load.src),
        .store => |store| try self.assembleStore(store.dst, store.src),
        .add_float => |add_float| try self.assembleAddFloat(add_float.dst, add_float.src1, add_float.src2),
        else => unreachable,
        // .sub_float => |sub_float| try self.assembleSubFloat(sub_float.dst, sub_float.src1, sub_float.src2),
        // .mul_float => |mul_float| try self.assembleMulFloat(mul_float.dst, mul_float.src1, mul_float.src2),
        // .div_float => |div_float| try self.assembleDivFloat(div_float.dst, div_float.src1, div_float.src2),
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
    var x86_64_assembler = try Assembler.init(x86_64_target);
    defer x86_64_assembler.deinit();
    var riscv64_assembler = try Assembler.init(riscv64_target);
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
        .expected_riscv64_code = &.{ 0x82, 0x80 },
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
        .expected_riscv64_code = &.{
            // c.fld fa0, 0(a0)
            0x08, 0x21,
            // c.fld fa1, 0(a1)
            0x8c, 0x21,
            // c.fld fa0, 0x78(a0)
            0x28, 0x3d,
            // fld fa0, -8(a0)
            0x07, 0x35, 0x85, 0xff,
            // fld fa0, 1(a0)
            0x07, 0x35, 0x15, 0x00,

            // lui a2, 0x11223
            0x37, 0x36, 0x22, 0x11,
            // addi a2, a2, 0x344
            0x13, 0x06, 0x46, 0x34,
            // c.add a2, a0
            0x2a, 0x96,
            // c.fld fa0, 0(a2)
            0x08, 0x22,

            // ret
            0x82, 0x80,
        },
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
        .expected_riscv64_code = &.{
            // c.fsd fa0, 0(a0)
            0x08, 0xa1,
            // c.fsd fa1, 0(a1)
            0x8c, 0xa1,
            // c.fsd fa0, 0x78(a0)
            0x28, 0xbd,
            // fsd fa0, -8(a0)
            0x27, 0x3c, 0xa5, 0xfe,
            // fsd fa0, 1(a0)
            0xa7, 0x30, 0xa5, 0x00,

            // lui a2, 0x11223
            0x37, 0x36, 0x22, 0x11,
            // addi a2, a2, 0x344
            0x13, 0x06, 0x46, 0x34,
            // c.add a2, a0
            0x2a, 0x96,
            // c.fsd fa0, 0(a2)
            0x08, 0xa2,

            // ret
            0x82, 0x80,
        },
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
            0xc5, 0xf3, 0x58, 0xc0,
            // vaddsd xmm0, xmm1, xmm0
            0xc5, 0xfb, 0x58, 0xc1,
            // vaddsd xmm1, xmm0, xmm0
            0xc5, 0xfb, 0x58, 0xc8,
            // ret
            0xc3,
        },
        .expected_riscv64_code = &.{
            // fadd.d fa0, fa0, fa0
            0x53, 0x75, 0xa5, 0x02,
            // fadd.d fa0, fa0, fa1
            0x53, 0x75, 0xb5, 0x02,
            // fadd.d fa0, fa1, fa0
            0x53, 0xf5, 0xa5, 0x02,
            // fadd.d fa1, fa0, fa0
            0xd3, 0x75, 0xa5, 0x02,
            // ret
            0x82, 0x80,
        },
        // zig fmt: on
    });
}

test "assembler buffer length is always aligned" {
    var assembler = try Assembler.init(x86_64_target);
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
