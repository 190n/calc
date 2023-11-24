const std = @import("std");
const builtin = @import("builtin");

const Program = @import("./Program.zig");
const AsmBuf = @import("./AsmBuf.zig").AsmBuf;

const Compiler = @This();

pub const CompiledCode = if (builtin.os.tag == .windows and builtin.cpu.arch == .x86_64)
    *const fn (stack: [*]f64, constants: [*]const f64) callconv(.SysV) void
else
    *const fn (stack: [*]f64, constants: [*]const f64) callconv(.C) void;

constants: std.ArrayList(f64),
stack_top: u16,

pub fn initWithConstants(allocator: std.mem.Allocator, program: Program) !Compiler {
    var compiler = Compiler{
        .constants = std.ArrayList(f64).init(allocator),
        .stack_top = program.num_args,
    };

    for (program.code.items) |inst| {
        switch (inst) {
            .constant => |c| {
                if (std.mem.indexOfScalar(f64, compiler.constants.items, c) == null) {
                    try compiler.constants.append(c);
                }
            },
            else => {},
        }
    }

    return compiler;
}

fn writeOffset(asm_buf: *AsmBuf, register: u8, offset: u16) !void {
    const byte_offset = @sizeOf(f64) * offset;

    if (byte_offset <= 0x7f) {
        try asm_buf.addImmediate(&.{0x40 | register});
        try asm_buf.addImmediate(&.{@truncate(byte_offset)});
    } else {
        try asm_buf.addImmediate(&.{0x80 | register});
        try asm_buf.addImmediate(std.mem.asBytes(&std.mem.nativeToLittle(u32, byte_offset)));
    }
}

pub fn compileOperator(self: *Compiler, asm_buf: *AsmBuf, op: Program.Operation) !void {
    switch (builtin.cpu.arch) {
        .x86_64 => {},
        .riscv64 => {
            std.debug.assert(builtin.cpu.features.isSuperSetOf(std.Target.riscv.cpu.baseline_rv64.features));
        },
        else => @compileError("only x86_64 and riscv64 are supported"),
    }

    // linux/mac: stack in rdi, constants in rsi
    // windows: stack in rcx, constants in rdx
    const stack = if (builtin.os.tag == .windows) 0x1 else 0x7;
    const constants = if (builtin.os.tag == .windows) 0x2 else 0x6;

    switch (op) {
        .add => {
            if (builtin.cpu.arch == .riscv64) {
                // c.fld fa0, (8*(top-2))(a0)
                try asm_buf.addInstruction(2, true, 0x2000 | (((self.stack_top - 2) & 0b111) << 10) | 0x0100 | (((self.stack_top - 2) & 0b11000) << 2) | 0b01000);
                // c.fld fa1, (8*(top-1))(a0)
                try asm_buf.addInstruction(2, true, 0x2000 | (((self.stack_top - 1) & 0b111) << 10) | 0x0100 | (((self.stack_top - 1) & 0b11000) << 2) | 0b01100);
                // fadd.d fa0, fa0, fa1
                try asm_buf.addInstruction(4, true, 0x02B57553);
                // c.fsd fa0, (8*(top-2))(a0)
                try asm_buf.addInstruction(2, true, 0b101_000_010_00_010_00 | (((self.stack_top - 2) & 0b111) << 10) | (((self.stack_top - 2) & 0b11000) << 2));
            } else {
                std.log.debug("movsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 2)});
                std.log.debug("addsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 1)});
                std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * (self.stack_top - 2)});
                // movsd (8*(top-2))(%rdi), %xmm0
                try asm_buf.addInstruction(3, false, 0xf20f10);
                try writeOffset(asm_buf, stack, self.stack_top - 2);
                // addsd (8*(top-1))(%rdi), %xmm0
                try asm_buf.addInstruction(3, false, 0xf20f58);
                try writeOffset(asm_buf, stack, self.stack_top - 1);
                // movsd %xmm0, (8*(top-2))(%rdi)
                try asm_buf.addInstruction(3, false, 0xf20f11);
                try writeOffset(asm_buf, stack, self.stack_top - 2);
            }
            self.stack_top -= 1;
        },
        .sub => {
            std.log.debug("movsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 2)});
            std.log.debug("subsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 1)});
            std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * (self.stack_top - 2)});
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // subsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f5c);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, false, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },
        .mul => {
            std.log.debug("movsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 2)});
            std.log.debug("mulsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 1)});
            std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * (self.stack_top - 2)});
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // mulsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f59);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, false, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },
        .div => {
            std.log.debug("movsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 2)});
            std.log.debug("divsd {}(%rdi), %xmm0", .{8 * (self.stack_top - 1)});
            std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * (self.stack_top - 2)});
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // divsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f5e);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, false, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },

        .constant => |c| {
            const index = std.mem.indexOfScalar(f64, self.constants.items, c).?;
            std.log.debug("movsd {}(%rsi), %xmm0", .{8 * index});
            std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * self.stack_top});
            // movsd (8*index)(%rsi), %xmm0
            try asm_buf.addInstruction(3, false, 0xf20f10);
            try writeOffset(asm_buf, constants, @intCast(index));
            // movsd %xmm0, (8*top)(%rdi)
            try asm_buf.addInstruction(3, false, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top);
            self.stack_top += 1;
        },
    }
}

pub fn addReturn(self: *const Compiler, asm_buf: *AsmBuf) !void {
    _ = self;
    // ret
    switch (builtin.cpu.arch) {
        .x86_64 => try asm_buf.addInstruction(1, false, 0xc3),
        .riscv64 => try asm_buf.addInstruction(2, true, 0x8082),
        else => @compileError("only x86_64 and riscv64 are supported"),
    }
}

pub fn getConstants(self: *Compiler) ![]f64 {
    return self.constants.toOwnedSlice();
}
