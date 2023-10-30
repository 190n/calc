const std = @import("std");
const builtin = @import("builtin");

const Program = @import("./program.zig");
const AsmBuf = @import("./asmbuf.zig").AsmBuf;

const Compiler = @This();

pub const CompiledCode = *const fn (stack: [*]f64, constants: [*]const f64) callconv(.C) void;

constants: [256]f64 = .{0.0} ** 256,
stack_top: u16,

pub fn initWithConstants(program: Program) Compiler {
    var compiler = Compiler{
        .stack_top = program.num_args,
    };
    var num_constants: usize = 0;

    for (program.code.slice()) |inst| {
        switch (inst) {
            .constant => |c| {
                if (std.mem.indexOfScalar(f64, compiler.constants[0..num_constants], c) == null) {
                    compiler.constants[num_constants] = c;
                    num_constants += 1;
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

pub fn compileOperator(self: *Compiler, asm_buf: *AsmBuf, op: Program.Op) !void {
    if (builtin.cpu.arch != .x86_64) @compileError("only x86_64 is supported");

    // linux/mac: stack in rdi, constants in rsi
    // windows: stack in rcx, constants in rdx
    const stack = if (builtin.os.tag == .windows) 0x1 else 0x7;
    const constants = if (builtin.os.tag == .windows) 0x2 else 0x6;

    switch (op) {
        .add => {
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // addsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f58);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },
        .sub => {
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // subsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f5c);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },
        .mul => {
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // mulsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f59);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },
        .div => {
            // movsd (8*(top-2))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f10);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            // divsd (8*(top-1))(%rdi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f5e);
            try writeOffset(asm_buf, stack, self.stack_top - 1);
            // movsd %xmm0, (8*(top-2))(%rdi)
            try asm_buf.addInstruction(3, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top - 2);
            self.stack_top -= 1;
        },

        .constant => |c| {
            const index = std.mem.indexOfScalar(f64, &self.constants, c).?;
            // movsd (8*index)(%rsi), %xmm0
            try asm_buf.addInstruction(3, 0xf20f10);
            try writeOffset(asm_buf, constants, @intCast(index));
            // movsd %xmm0, (8*top)(%rdi)
            try asm_buf.addInstruction(3, 0xf20f11);
            try writeOffset(asm_buf, stack, self.stack_top);
            self.stack_top += 1;
        },
    }
}

pub fn addReturn(self: *const Compiler, asm_buf: *AsmBuf) !void {
    _ = self;
    if (builtin.cpu.arch != .x86_64) @compileError("only x86_64 is supported");

    // ret
    try asm_buf.addInstruction(1, 0xc3);
}
