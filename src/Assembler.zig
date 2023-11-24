const std = @import("std");

const Operation = @import("./Program.zig").Operation;

const Assembler = @This();

const page_size = std.mem.page_size;
const page_allocator = std.heap.page_allocator;

target: std.Target,
code: union(enum) {
    writable: std.ArrayListAlignedUnmanaged(u8, page_size),
    executable: []align(page_size) const u8,
},

pub const IntRegister = enum {
    /// register used for the first function argument (where a pointer to the VM stack is passed)
    stack,
    /// register used for the second function argument (where a pointer to the VM constants is
    /// passed)
    constants,
};

pub const FloatRegister = enum {
    /// callee saved (non volatile) FP registers
    a,
    b,
};

pub const Operand = union(enum) {
    int_register: IntRegister,
    float_register: FloatRegister,
    memory: struct {
        base: IntRegister,
        offset: i32,
    },
};

pub const Instruction = union(enum) {
    move: struct {
        dst: Operand,
        src: Operand,
    },
    add_float: struct {
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

pub fn finalize(self: *Assembler) !void {
    // extract memory allocated by array list
    // round up to a full page
    const new_slice = try page_allocator.alignedAlloc(
        u8,
        page_size,
        std.mem.alignForward(usize, self.code.writable.items.len, page_size),
    );
    errdefer page_allocator.free(new_slice);
    @memcpy(new_slice[0..self.code.writable.items.len], self.code.writable.items);
    // make it executable
    try std.os.mprotect(new_slice, std.os.PROT.READ | std.os.PROT.EXEC);
    self.code.writable.deinit(page_allocator);
    self.code = .{ .executable = new_slice };
}

pub fn getFunctionPointer(self: *Assembler, comptime T: type) T {
    return @ptrCast(self.code.executable.ptr);
}

pub fn deinit(self: *Assembler) void {
    switch (self.code) {
        .writable => |_| self.code.writable.deinit(page_allocator),
        .executable => |slice| page_allocator.free(slice),
    }
    self.* = undefined;
}

pub fn getRegisterNumber(self: Assembler, reg: anytype) u8 {
    return switch (@TypeOf(reg)) {
        IntRegister => switch (self.target.cpu.arch) {
            .x86_64 => switch (reg) {
                .stack => 7, // rdi
                .constants => 6, // rsi
            },
            .riscv64 => switch (reg) {
                .stack => 10, // a0
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
    if (std.math.cast(i8, full_offset)) |offset| {
        try self.emit(u8, 0x40 | (reg_num << 3) | base_num);
        try self.emit(u8, @bitCast(offset));
    } else {
        try self.emit(u8, 0x80 | (reg_num << 3) | base_num);
        try self.emit(u32, @bitCast(full_offset));
    }
}

fn emit(self: *Assembler, comptime T: type, data: T) !void {
    if (T == u8) {
        try self.code.writable.append(page_allocator, data);
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

pub fn assemble(self: *Assembler, inst: Instruction) !void {
    switch (inst) {
        .move => |move| {
            const dst = move.dst;
            const src = move.src;
            switch (self.target.cpu.arch) {
                .x86_64 => {
                    if (src == .memory and dst == .float_register) {
                        // movsd dst, qword [base + offset]
                        try self.emit(u8, 0xf2);
                        try self.emit(u8, 0x0f);
                        try self.emit(u8, 0x10);
                        try self.emitX86Offset(src.memory.base, src.memory.offset, dst.float_register);
                    } else if (src == .float_register and dst == .memory) {
                        // movsd qword [base + offset], src
                        try self.emit(u8, 0xf2);
                        try self.emit(u8, 0x0f);
                        try self.emit(u8, 0x11);
                        try self.emitX86Offset(dst.memory.base, dst.memory.offset, src.float_register);
                    } else unreachable;
                },
                .riscv64 => {
                    if (src == .memory and dst == .float_register) {
                        if (@mod(src.memory.offset, 8) == 0 and src.memory.offset >= 0 and src.memory.offset < 256) {
                            // c.fld dst, offset(base)
                            const offset: u8 = @intCast(src.memory.offset);
                            try self.emit(
                                u16,
                                0x2000 |
                                    @as(u16, self.getRegisterNumber(src.memory.base) & 0b111) << 7 |
                                    (self.getRegisterNumber(dst.float_register) & 0b111) << 2 |
                                    @as(u16, offset & 0b111000) << 7 |
                                    (offset & 0b11000000) >> 1,
                            );
                        } else if (std.math.cast(i12, src.memory.offset)) |immediate_offset| {
                            // fld dst, offset(base)
                            try self.emit(
                                u32,
                                0b011000000000111 |
                                    @as(u32, @as(u12, @bitCast(immediate_offset))) << 25 |
                                    @as(u32, self.getRegisterNumber(src.memory.base)) << 15 |
                                    @as(u32, self.getRegisterNumber(dst.float_register)) << 7,
                            );
                        } else {
                            // lui s0, offset[31:12]
                            // addi s0, s0, offset[11:0]
                            // c.add s0, s0, base
                            // c.fld dst, 0(s0)
                            unreachable;
                        }
                    } else if (src == .float_register and dst == .memory) {
                        //
                    } else unreachable;
                },
                else => unreachable,
            }
        },
        else => unreachable,
    }
}

test "assemble short programs" {
    var a = try Assembler.init(.{
        .abi = .gnu,
        .cpu = std.Target.Cpu.baseline(.x86_64),
        .os = .{
            .tag = .linux,
            .version_range = std.Target.Os.VersionRange.default(.linux, .x86_64),
        },
        .ofmt = .elf,
    });
    defer a.deinit();

    try a.assemble(.{
        .move = .{
            .src = .{ .memory = .{ .base = .stack, .offset = -0x7fffffff } },
            .dst = .{ .float_register = .a },
        },
    });

    std.debug.print("{}\n", .{std.fmt.fmtSliceHexLower(a.code.writable.items)});
}
